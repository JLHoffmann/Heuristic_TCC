// EPOS Thread Component Initialization

#include <system.h>
#include <thread.h>
#include <alarm.h>
#include <clock.h>
#include <segment.h>
#include <mmu.h>

__BEGIN_SYS

void Thread::init()
{
    // The installation of the scheduler timer handler must precede the
    // creation of threads, since the constructor can induce a reschedule
    // and this in turn can call timer->reset()
    // Letting reschedule() happen during thread creation is harmless, since
    // MAIN is created first and dispatch won't replace it nor by itself
    // neither by IDLE (which has a lower priority)
    if(Criterion::timed && (Machine::cpu_id() == 0))
        _timer = new (SYSTEM) Scheduler_Timer(QUANTUM, time_slicer);

    // Install an interrupt handler to receive forced reschedules
    if(smp) {
        if(Machine::cpu_id() == 0)
            db<Thread>(WRN)<<"DATE:: "<<RTC::date()<<endl;
        if(Machine::cpu_id() == 0)
            IC::int_vector(IC::INT_RESCHEDULER, rescheduler);
        IC::enable(IC::INT_RESCHEDULER);
    }
    // Checks if monitoring execution is enable
    if (Criterion::monitoring) {
        // Only one CPU needs to initialize the Monitoring Capture System
        if (Machine::cpu_id() == 0) {
            _end_capture = true;
            // Allocating a memeory region (117MB) for store the capture
            // For this to work, we increased heap size in 120MB. --> This can handle 1024000 captures
            _thread_monitor = new Monitoring_Capture(128000, new (SYSTEM) Moment[1024000]);
        }
    }

    Criterion::init();
}

__END_SYS
