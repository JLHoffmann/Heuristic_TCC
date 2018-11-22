// EPOS Thread Component Implementation

#include <machine.h>
#include <system.h>
#include <thread.h>
#include <alarm.h> // for FCFS

// This_Thread class attributes
__BEGIN_UTIL
bool This_Thread::_not_booting;
__END_UTIL

__BEGIN_SYS

// Class attributes
Monitoring_Capture* Thread::_thread_monitor;
unsigned int Thread::_global_deadline_misses;
unsigned int Thread::_prev_global_deadline_misses;
volatile unsigned int Thread::_thread_count;
volatile bool Thread::_end_capture;
volatile unsigned int Thread::_clock_factor[Traits<Build>::CPUS];
volatile unsigned int Thread::_slowdown[Traits<Build>::CPUS];
volatile float Thread::_bef_channel5[Traits<Build>::CPUS];
volatile float Thread::_bef_channel6[Traits<Build>::CPUS];

Scheduler_Timer * Thread::_timer;
Scheduler<Thread> Thread::_scheduler;
Spin Thread::_lock;

// Methods
void Thread::constructor_prologue(const Color & color, unsigned int stack_size)
{
    lock();

    _thread_count++;
    _scheduler.insert(this);
    if(Traits<MMU>::colorful && color != WHITE)
        _stack = new (color) char[stack_size];
    else
        _stack = new (SYSTEM) char[stack_size];
}


void Thread::constructor_epilogue(const Log_Addr & entry, unsigned int stack_size)
{
    db<Thread>(TRC) << "Thread(task=" << _task
                    << ",entry=" << entry
                    << ",state=" << _state
                    << ",priority=" << _link.rank()
                    << ",stack={b=" << reinterpret_cast<void *>(_stack)
                    << ",s=" << stack_size
                    << "},context={b=" << _context
                    << "," << *_context << "}) => " << this << "@" << _link.rank().queue() << endl;

    if(multitask)
        _task->insert(this);

    if((_state != READY) && (_state != RUNNING))
        _scheduler.suspend(this);

    if(preemptive && (_state == READY) && (_link.rank() != IDLE))
        reschedule(_link.rank().queue());
    else
        unlock();

    _missed_deadlines = 0;
    _times_p_count = 0;
    _alarm_times = 0;
}


Thread::~Thread()
{
    lock();

    db<Thread>(TRC) << "~Thread(this=" << this
                    << ",state=" << _state
                    << ",priority=" << _link.rank()
                    << ",stack={b=" << reinterpret_cast<void *>(_stack)
                    << ",context={b=" << _context
                    << "," << *_context << "})" << endl;

    // The running thread cannot delete itself!
    assert(_state != RUNNING);

    switch(_state) {
    case RUNNING:  // For switch completion only: the running thread would have deleted itself! Stack wouldn't have been released!
        exit(-1);
        break;
    case READY:
        _scheduler.remove(this);
        _thread_count--;
        break;
    case SUSPENDED:
        _scheduler.resume(this);
        _scheduler.remove(this);
        _thread_count--;
        break;
    case WAITING:
        _waiting->remove(this);
        _scheduler.resume(this);
        _scheduler.remove(this);
        _thread_count--;
        break;
    case FINISHING: // Already called exit()
        break;
    }

    if(multitask) {
        _task->remove(this);
        delete _user_stack;
    }

    if(_joining)
        _joining->resume();

    unlock();

    delete _stack;
}


void Thread::priority(const Priority & c)
{
    lock();

    db<Thread>(TRC) << "Thread::priority(this=" << this << ",prio=" << c << ")" << endl;

    unsigned int old_cpu = _link.rank().queue();

    _link.rank(Criterion(c));

    if(_state != RUNNING) {
        _scheduler.remove(this);
        _scheduler.insert(this);
    }

    if(preemptive) {
        reschedule(old_cpu);
        if(smp) {
            lock();
            reschedule(_link.rank().queue());
        }
    }
}


int Thread::join()
{
    lock();

    db<Thread>(TRC) << "Thread::join(this=" << this << ",state=" << _state << ")" << endl;

    // Precondition: no Thread::self()->join()
    assert(running() != this);

    // Precondition: a single joiner
    assert(!_joining);

    if(_state != FINISHING) {
        _joining = running();
        _joining->suspend(true);
    } else
        unlock();

    return *reinterpret_cast<int *>(_stack);
}


void Thread::pass()
{
    lock();

    db<Thread>(TRC) << "Thread::pass(this=" << this << ")" << endl;

    Thread * prev = running();
    Thread * next = _scheduler.choose(this);

    if(next)
        dispatch(prev, next, false);
    else {
        db<Thread>(WRN) << "Thread::pass => thread (" << this << ") not ready!" << endl;
        unlock();
    }
}

void Thread::suspend(bool locked)
{
    if(!locked)
        lock();

    db<Thread>(TRC) << "Thread::suspend(this=" << this << ")" << endl;

    Thread * prev = running();

    _scheduler.suspend(this);
    _state = SUSPENDED;

    Thread * next = running();

    dispatch(prev, next);
}


void Thread::resume()
{
    lock();

    db<Thread>(TRC) << "Thread::resume(this=" << this << ")" << endl;

    if(_state == SUSPENDED) {
        _state = READY;
        _scheduler.resume(this);

        if(preemptive)
            reschedule(_link.rank().queue());
    } else {
        db<Thread>(WRN) << "Resume called for unsuspended object!" << endl;

        unlock();
    }
}


// Class methods
void Thread::yield()
{
    lock();

    db<Thread>(TRC) << "Thread::yield(running=" << running() << ")" << endl;

    Thread * prev = running();
    Thread * next = _scheduler.choose_another();

    dispatch(prev, next);
}


void Thread::exit(int status)
{
    lock();

    db<Thread>(TRC) << "Thread::exit(status=" << status << ") [running=" << running() << "]" << endl;

    Thread * prev = running();
    _scheduler.remove(prev);
    *reinterpret_cast<int *>(prev->_stack) = status;
    prev->_state = FINISHING;

    _thread_count--;

    if(prev->_joining) {
        prev->_joining->_state = READY;
        _scheduler.resume(prev->_joining);
        prev->_joining = 0;
    }

    dispatch(prev, _scheduler.choose());
}


void Thread::sleep(Queue * q)
{
    db<Thread>(TRC) << "Thread::sleep(running=" << running() << ",q=" << q << ")" << endl;

    // lock() must be called before entering this method
    assert(locked());

    Thread * prev = running();
    _scheduler.suspend(prev);
    prev->_state = WAITING;
    q->insert(&prev->_link);
    prev->_waiting = q;

    dispatch(prev, _scheduler.chosen());
}


void Thread::wakeup(Queue * q)
{
    db<Thread>(TRC) << "Thread::wakeup(running=" << running() << ",q=" << q << ")" << endl;

    // lock() must be called before entering this method
    assert(locked());

    if(!q->empty()) {
        Thread * t = q->remove()->object();
        t->_state = READY;
        t->_waiting = 0;
        _scheduler.resume(t);

        if(preemptive)
            reschedule(t->_link.rank().queue());
    } else
        unlock();
}


void Thread::wakeup_all(Queue * q)
{
    db<Thread>(TRC) << "Thread::wakeup_all(running=" << running() << ",q=" << q << ")" << endl;

    // lock() must be called before entering this method
    assert(locked());

    if(!q->empty())
        while(!q->empty()) {
            Thread * t = q->remove()->object();
            t->_state = READY;
            t->_waiting = 0;
            _scheduler.resume(t);

            if(preemptive) {
                reschedule(t->_link.rank().queue());
                lock();
            }
         }
    else
        unlock();
}


void Thread::reschedule()
{
    db<Scheduler<Thread> >(TRC) << "Thread::reschedule()" << endl;

    // lock() must be called before entering this method
    assert(locked());

    Thread * prev = running();
    Thread * next = _scheduler.choose();

    dispatch(prev, next);
}


void Thread::reschedule(unsigned int cpu)
{
    if(!smp || (cpu == Machine::cpu_id()))
        reschedule();
    else {
        db<Scheduler<Thread> >(TRC) << "Thread::reschedule(cpu=" << cpu << ")" << endl;
        IC::ipi_send(cpu, IC::INT_RESCHEDULER);
        unlock();
    }
}


void Thread::rescheduler(const IC::Interrupt_Id & interrupt)
{
    lock();

    reschedule();
}


void Thread::time_slicer(const IC::Interrupt_Id & i)
{
    lock();

    reschedule();
}


void Thread::dispatch(Thread * prev, Thread * next, bool charge)
{
    
    if(charge) {
        if(Criterion::timed)
            _timer->reset();

        //Monitoring Capture
        if(Criterion::monitoring && !_end_capture) {
            unsigned int entry_temp = CPU::temperature();
            if (prev->priority() != IDLE && prev->priority() != MAIN) {
                prev->_missed_deadlines = prev->_times_p_count - (prev->_alarm_times->_times + 1);
            } else 
                 prev->_missed_deadlines = 0;

            if (next->priority() != IDLE && next->priority() != MAIN) {
                next->_missed_deadlines = next->_times_p_count - (next->_alarm_times->_times + 1);
            } else 
                next->_missed_deadlines = 0;

            if (prev->_missed_deadlines < 0)
                    prev->_missed_deadlines = 0;
                
            if (next->_missed_deadlines < 0)
                    next->_missed_deadlines = 0;

            float channel_3 = 0;
            float channel_5 = 0;
            float channel_6 = 0;

            if ( next->priority() != IDLE && Criterion::heuristic) {

                if (prev->_missed_deadlines > 0 || next->_missed_deadlines > 0) {
                    _clock_factor[Machine::cpu_id()] = 8;
                } else if (_thread_monitor->last_capture(Machine::cpu_id(), 0) > 0) {
                    unsigned long long ts = _thread_monitor->last_capture(Machine::cpu_id(), 7);
                    unsigned long long ts_dif = _thread_monitor->time() - ts;
                    if (ts_dif > 999) {
                        channel_3 = (PMU::read(3) - _thread_monitor->last_capture(Machine::cpu_id(), 3)) / (ts_dif * 1.0);
                        channel_5 = (PMU::read(5) - _thread_monitor->last_capture(Machine::cpu_id(), 5)) / (ts_dif);
                        channel_6 = (PMU::read(6) - _thread_monitor->last_capture(Machine::cpu_id(), 6)) / (ts_dif);
                        if (channel_3 <= 20 && prev->priority() != IDLE) {
                            if (channel_3 >= 8 && _clock_factor[Machine::cpu_id()] == 8) {
                                if (_slowdown[Machine::cpu_id()] > 0)
                                    _slowdown[Machine::cpu_id()]--;
                            } else if (channel_3 > 10 && _clock_factor[Machine::cpu_id()] == 7) {
                                _clock_factor[Machine::cpu_id()]++;
                            } else if (channel_3 >= 0.8 && _clock_factor[Machine::cpu_id()] == 6) {
                                _clock_factor[Machine::cpu_id()]++;
                            } else if (channel_3 >= 0.2 && _clock_factor[Machine::cpu_id()] < 6) {
                                _clock_factor[Machine::cpu_id()]+= 2;
                            } else {
                                if (_clock_factor[Machine::cpu_id()] > 5) {
                                    if (_slowdown[Machine::cpu_id()] > 0)
                                        _slowdown[Machine::cpu_id()]--;
                                    else {
                                        _clock_factor[Machine::cpu_id()]--;
                                    }
                                }
                            }
                        } else if (channel_6 > 0 || channel_5 > 0) {
                            if (_clock_factor[Machine::cpu_id()] == 8) {
                                if (_slowdown[Machine::cpu_id()] > 0){
                                    _slowdown[Machine::cpu_id()]--;
                                } else if ((channel_5 >= 30 && channel_6 >= 30) ||
                                    (channel_5 >= _bef_channel5[Machine::cpu_id()]+2 && channel_6 >= _bef_channel6[Machine::cpu_id()]+2)) {
                                    _clock_factor[Machine::cpu_id()]--;
                                }
                            } else if (_clock_factor[Machine::cpu_id()] == 7) {
                                if (_slowdown[Machine::cpu_id()] > 0) {
                                    _slowdown[Machine::cpu_id()]--;
                                    if (channel_5 <= _bef_channel5[Machine::cpu_id()] - 5 && channel_6 <= _bef_channel6[Machine::cpu_id()] - 5 && channel_3 > 250) {
                                        _clock_factor[Machine::cpu_id()]++;
                                    }
                                } else if ((channel_5 >= 25 && channel_6 >= 25) ||
                                 (channel_5 >= _bef_channel5[Machine::cpu_id()]+2 && channel_6 >= _bef_channel6[Machine::cpu_id()]+2)) {
                                    _clock_factor[Machine::cpu_id()]--;
                                } else if (channel_5 <= _bef_channel5[Machine::cpu_id()] - 5 && channel_6 <= _bef_channel6[Machine::cpu_id()] - 5 && channel_3 > 250) {
                                    _clock_factor[Machine::cpu_id()]++;
                                }
                            } else if (_clock_factor[Machine::cpu_id()] < 7 && (_bef_channel5[Machine::cpu_id()] > 0 || _bef_channel6[Machine::cpu_id()] > 0)) {
                                if (channel_5 <= _bef_channel5[Machine::cpu_id()] - 3 && channel_6 <= _bef_channel6[Machine::cpu_id()] - 2) {
                                    if (_clock_factor[Machine::cpu_id()] < 6) {
                                        if (channel_3 > 60) {
                                            _clock_factor[Machine::cpu_id()]+= 2;
                                            _slowdown[Machine::cpu_id()] = 2;
                                        } else if (channel_3 > 20) {
                                            _clock_factor[Machine::cpu_id()]++;
                                            _slowdown[Machine::cpu_id()] = 2;
                                        }
                                    } else if (channel_3 > 40 && channel_6 <= 20 && channel_5 <= 20) {
                                        _clock_factor[Machine::cpu_id()]++;
                                        _slowdown[Machine::cpu_id()] = 2;
                                    } else if (_slowdown[Machine::cpu_id()] > 0) {
                                        _slowdown[Machine::cpu_id()]--;
                                    }
                                } else {
                                    if (_slowdown[Machine::cpu_id()] > 0) {
                                        _slowdown[Machine::cpu_id()]--;
                                    } else if (channel_5 >= _bef_channel5[Machine::cpu_id()]+2 && channel_6 >= _bef_channel6[Machine::cpu_id()]+2 
                                        && channel_3 < 110 && _clock_factor[Machine::cpu_id()] > 5){
                                        _clock_factor[Machine::cpu_id()]--;
                                    }
                                }
                            
                            }
                        }
                        _bef_channel6[Machine::cpu_id()] = channel_6;
                        _bef_channel5[Machine::cpu_id()] = channel_5;
                    }
                }
                if (Machine::cpu_id() % 2) {
                    if ((_clock_factor[Machine::cpu_id()-1] - 1) > _clock_factor[Machine::cpu_id()]) {
                        _clock_factor[Machine::cpu_id()] = _clock_factor[Machine::cpu_id()-1] -1;
                    }
                } else {
                    if ((_clock_factor[Machine::cpu_id()+1] - 1) > _clock_factor[Machine::cpu_id()]) {
                        _clock_factor[Machine::cpu_id()] = _clock_factor[Machine::cpu_id()+1] -1;
                    }
                }
                if (next->priority() == IDLE) {
                    _bef_channel6[Machine::cpu_id()] = 0;
                    _bef_channel5[Machine::cpu_id()] = 0;
                }
                CPU::clock((CPU::clock()/8 * _clock_factor[Machine::cpu_id()]));
            }
            unsigned long long pkg = CPU::pkg_energy_status();
            unsigned long long pp0 = CPU::pp0_energy_status();
            //unsigned long long energy_unit = CPU::rapl_energy_unit(); // use only to know the power unit value
            _thread_monitor->capture(entry_temp, PMU::read(0), PMU::read(1), PMU::read(2), PMU::read(3), PMU::read(4), PMU::read(5),
            PMU::read(6), reinterpret_cast<volatile unsigned int>(prev), prev->priority(), Machine::cpu_id(), prev->_missed_deadlines, prev->_global_deadline_misses, pkg, pp0);
            // capture next -> not necessary, it's possible to infer most of the data
            // _thread_monitor->capture(entry_temp, PMU::read(0), PMU::read(1), PMU::read(2), PMU::read(3), PMU::read(4), PMU::read(5),
            // PMU::read(6), reinterpret_cast<volatile unsigned int>(next), next->priority(), Machine::cpu_id(), next->_missed_deadlines, next->_global_deadline_misses);
        }
    }

    if(prev != next) {
        if(prev->_state == RUNNING)
            prev->_state = READY;
        next->_state = RUNNING;

        db<Thread>(TRC) << "Thread::dispatch(prev=" << prev << ",next=" << next << ")" << endl;
        db<Thread>(INF) << "prev={" << prev << ",ctx=" << *prev->_context << "}" << endl;
        db<Thread>(INF) << "next={" << next << ",ctx=" << *next->_context << "}" << endl;

        if(smp)
            _lock.release();

        if(multitask && (next->_task != prev->_task))
            next->_task->activate();

        CPU::switch_context(&prev->_context, next->_context);
    } else
        if(smp)
            _lock.release();

    // TODO: could this be moved to right after the switch_context?
    CPU::int_enable();
}


int Thread::idle()
{
    while(_thread_count > Machine::n_cpus()) { // someone else besides idles
        if(Traits<Thread>::trace_idle)
            db<Thread>(TRC) << "Thread::idle(CPU=" << Machine::cpu_id() << ",this=" << running() << ")" << endl;
        CPU::int_enable();
        CPU::halt();
        if(_scheduler.schedulables() > 0)// A thread might have been woken up by another CPU
            yield();
    }

    CPU::int_disable();
    if(Machine::cpu_id() == 0) {
        if (Criterion::monitoring) {
            db<Thread>(WRN) << "temp: "<< CPU::temperature() <<endl;
            _thread_monitor->datas();
        }
        db<Thread>(WRN) << "The last thread has exited!" << endl;

        if(reboot) {
            db<Thread>(WRN) << "Rebooting the machine ..." << endl;
            Machine::reboot();
        } else
            db<Thread>(WRN) << "Halting the machine ..." << endl;
    }
    CPU::halt();

    return 0;
}

__END_SYS

// Id forwarder to the spin lock
__BEGIN_UTIL
unsigned int This_Thread::id()
{
    return _not_booting ? reinterpret_cast<volatile unsigned int>(Thread::self()) : Machine::cpu_id() + 1;
}
__END_UTIL
