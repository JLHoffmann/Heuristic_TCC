// EPOS Thread Component Declarations

#ifndef __thread_h
#define __thread_h

#include <utility/queue.h>
#include <utility/handler.h>
#include <cpu.h>
#include <machine.h>
#include <system.h>
#include <scheduler.h>
#include <segment.h>
#include <architecture/ia32/monitoring_capture.h>
extern "C" { void __exit(); }

__BEGIN_SYS

class Thread
{
    friend class Init_First;
    friend class Init_System;
    friend class System;
    friend class Scheduler<Thread>;
    friend class Synchronizer_Common;
    friend class Alarm;
    friend class Task;
    friend class Agent;

protected:
    static const bool smp = Traits<Thread>::smp;
    static const bool preemptive = Traits<Thread>::Criterion::preemptive;
    static const bool multitask = Traits<System>::multitask;
    static const bool reboot = Traits<System>::reboot;

    static const unsigned int QUANTUM = Traits<Thread>::QUANTUM;
    static const unsigned int STACK_SIZE = multitask ? Traits<System>::STACK_SIZE : Traits<Application>::STACK_SIZE;
    static const unsigned int USER_STACK_SIZE = Traits<Application>::STACK_SIZE;

    typedef CPU::Log_Addr Log_Addr;
    typedef CPU::Context Context;

public:
    // Thread State
    enum State {
        RUNNING,
        READY,
        SUSPENDED,
        WAITING,
        FINISHING
    };

    // Thread Priority
    typedef Scheduling_Criteria::Priority Priority;

    // Thread Scheduling Criterion
    typedef Traits<Thread>::Criterion Criterion;
    enum {
        HIGH    = Criterion::HIGH,
        NORMAL  = Criterion::NORMAL,
        LOW     = Criterion::LOW,
        MAIN    = Criterion::MAIN,
        IDLE    = Criterion::IDLE
    };

    // Thread Configuration
    // t = 0 => Task::self()
    // ss = 0 => user-level stack on an auto expand segment
    struct Configuration {
        Configuration(const State & s = READY, const Criterion & c = NORMAL, const Color & a = WHITE, Task * t = 0, unsigned int ss = STACK_SIZE)
        : state(s), criterion(c), color(a), task(t), stack_size(ss) {}

        State state;
        Criterion criterion;
        Color color;
        Task * task;
        unsigned int stack_size;
    };

    // Thread Queue
    typedef Ordered_Queue<Thread, Criterion, Scheduler<Thread>::Element> Queue;

public:
    template<typename ... Tn>
    Thread(int (* entry)(Tn ...), Tn ... an);
    template<typename ... Tn>
    Thread(const Configuration & conf, int (* entry)(Tn ...), Tn ... an);
    ~Thread();

    const volatile State & state() const { return _state; }

    const volatile Priority & priority() const { return _link.rank(); }
    void priority(const Priority & p);

    Task * task() const { return _task; }

    int join();
    void pass();
    void suspend() { suspend(false); }
    void resume();

    static Thread * volatile self() { return running(); }
    static void yield();
    static void exit(int status = 0);

protected:
    void constructor_prologue(const Color & color, unsigned int stack_size);
    void constructor_epilogue(const Log_Addr & entry, unsigned int stack_size);

    static Thread * volatile running() { return _scheduler.chosen(); }

    Queue::Element * link() { return &_link; }

    Criterion & criterion() { return const_cast<Criterion &>(_link.rank()); }

    static void lock() {
        CPU::int_disable();
        if(smp)
            _lock.acquire();
    }

    static void unlock() {
        if(smp)
            _lock.release();
        CPU::int_enable();
    }

    static bool locked() { return CPU::int_disabled(); }

    void suspend(bool locked);

    static void sleep(Queue * q);
    static void wakeup(Queue * q);
    static void wakeup_all(Queue * q);

    static void suspend(Thread * t, Queue* q);
    static void resume(Queue* q);
    static void reschedule();
    static void reschedule(unsigned int cpu);
    static void rescheduler(const IC::Interrupt_Id & interrupt);
    static void time_slicer(const IC::Interrupt_Id & interrupt);

    static void dispatch(Thread * prev, Thread * next, bool charge = true);

    static int idle();

private:
    static void init();

protected:
    Task * _task;
    Segment * _user_stack;

    char * _stack;
    Context * volatile _context;
    volatile State _state;
    Queue * _waiting;
    Thread * volatile _joining;
    Queue::Element _link;
    Alarm* _alarm_times;

    static volatile unsigned int _thread_count;
    static Scheduler_Timer * _timer;
    static Scheduler<Thread> _scheduler;
    static Spin _lock;
public:
    volatile int _missed_deadlines; //it's not unsigned because of the calculating process
    int _times_p_count; //counts the number of executions to be used on deadline misses calc
    static Monitoring_Capture* _thread_monitor; //system monitor 
    static volatile bool _end_capture; //stops capture when True
    static unsigned int _global_deadline_misses;
    static unsigned int _prev_global_deadline_misses; //used to calc the difference when using heuristics
    static volatile unsigned int _clock_factor[Traits<Build>::CPUS]; //stores the current clock duty cycle. (8 ranges, 12.5% on each)
    static volatile unsigned int _slowdown[Traits<Build>::CPUS]; //Assist in decisions on increase or decrease clock
    static volatile float _bef_channel5[Traits<Build>::CPUS]; //PMU Channel 5 last difference capture
    static volatile float _bef_channel6[Traits<Build>::CPUS]; //PMU Channel 6 last difference capture

};

__END_SYS

#include <task.h>

__BEGIN_SYS

template<typename ... Tn>
inline Thread::Thread(int (* entry)(Tn ...), Tn ... an)
: _task(Task::self()), _user_stack(0), _state(READY), _waiting(0), _joining(0), _link(this, NORMAL)
{
    constructor_prologue(WHITE, STACK_SIZE);
    _context = CPU::init_stack(0, _stack + STACK_SIZE, &__exit, entry, an ...);
    constructor_epilogue(entry, STACK_SIZE);
}

template<typename ... Tn>
inline Thread::Thread(const Configuration & conf, int (* entry)(Tn ...), Tn ... an)
: _task(conf.task ? conf.task : Task::self()), _state(conf.state), _waiting(0), _joining(0), _link(this, conf.criterion)
{
    if(multitask && !conf.stack_size) { // Auto-expand, user-level stack
        constructor_prologue(conf.color, STACK_SIZE);
        _user_stack = new (SYSTEM) Segment(USER_STACK_SIZE);

        // Attach the thread's user-level stack to the current address space so we can initialize it
        Log_Addr ustack = Task::self()->address_space()->attach(_user_stack);

        // Initialize the thread's user-level stack and determine a relative stack pointer (usp) from the top of the stack
        Log_Addr usp = ustack + USER_STACK_SIZE;
        if(conf.criterion == MAIN)
            usp -= CPU::init_user_stack(usp, 0, an ...); // the main thread of each task must return to crt0 to call _fini (global destructors) before calling __exit
        else
            usp -= CPU::init_user_stack(usp, &__exit, an ...); // __exit will cause a Page Fault that must be properly handled

        // Attach the thread's user-level stack from the current address space
        Task::self()->address_space()->detach(_user_stack, ustack);

        // Attach the thread's user-level stack to its task's address space so it will be able to access it when it runs
        ustack = _task->address_space()->attach(_user_stack);

        // Determine an absolute stack pointer (usp) from the top of the thread's user-level stack considering the address it will see it when it runs
        usp = ustack + USER_STACK_SIZE - usp;

        // Initialize the thread's system-level stack
        _context = CPU::init_stack(usp, _stack + STACK_SIZE, &__exit, entry, an ...);
    } else {
        constructor_prologue(conf.color, conf.stack_size);
        _user_stack = 0;
        _context = CPU::init_stack(0, _stack + conf.stack_size, &__exit, entry, an ...);
    }

    constructor_epilogue(entry, STACK_SIZE);
}


// An event handler that triggers a thread (see handler.h)
class Thread_Handler : public Handler
{
public:
    Thread_Handler(Thread * h) : _handler(h) {}
    ~Thread_Handler() {}

    void operator()() { _handler->resume(); }

private:
    Thread * _handler;
};

__END_SYS

#endif
