// EPOS Periodic Thread Component Declarations

// Periodic threads are achieved by programming an alarm handler to invoke
// p() on a control semaphore after each job (i.e. task activation). Base
// threads are created in BEGINNING state, so the scheduler won't dispatch
// them before the associate alarm and semaphore are created. The first job
// is dispatched by resume() (thus the _state = SUSPENDED statement)

#ifndef __periodic_thread_h
#define __periodic_thread_h

#include <utility/handler.h>
#include <thread.h>
#include <alarm.h>

__BEGIN_SYS

// Aperiodic Thread
typedef Thread Aperiodic_Thread;

// Periodic Thread
class Periodic_Thread: public Thread
{
protected:
    // Alarm Handler for periodic threads under static scheduling policies
    class Static_Handler: public Semaphore_Handler
    {
    public:
        Static_Handler(Semaphore * s, Periodic_Thread * t): Semaphore_Handler(s) {}
        ~Static_Handler() {}
    };

    // Alarm Handler for periodic threads under dynamic scheduling policies
    class Dynamic_Handler: public Semaphore_Handler
    {
    public:
        Dynamic_Handler(Semaphore * s, Periodic_Thread * t): Semaphore_Handler(s), _thread(t) {}
        ~Dynamic_Handler() {}

        void operator()() {
            _thread->criterion().update();

            Semaphore_Handler::operator()();
        }

    private:
        Periodic_Thread * _thread;
    };

    typedef IF<Criterion::dynamic, Dynamic_Handler, Static_Handler>::Result Handler;

public:
    typedef RTC::Microsecond Microsecond;

    enum { INFINITE = RTC::INFINITE };

    struct Configuration: public Thread::Configuration {
        Configuration(const Microsecond & p, int n = INFINITE, const State & s = READY, const Criterion & c = Criterion::PERIODIC, const Color & a = WHITE, Task * t = 0, unsigned int ss = STACK_SIZE)
        : Thread::Configuration(s, c, a, t, ss), period(p), times(n) {}

        Microsecond period;
        int times;
    };

public:
    template<typename ... Tn>
    Periodic_Thread(const Microsecond & p, int (* entry)(Tn ...), Tn ... an)
    : Thread(Thread::Configuration(SUSPENDED, Criterion(p)), entry, an ...),
      _semaphore(0), _handler(&_semaphore, this), _alarm(p, &_handler, INFINITE) { resume(); }

    template<typename ... Tn>
    Periodic_Thread(const Configuration & conf, int (* entry)(Tn ...), Tn ... an)
    : Thread(Thread::Configuration(SUSPENDED, (conf.criterion != NORMAL) ? conf.criterion : Criterion(conf.period), conf.color, conf.task, conf.stack_size), entry, an ...),
      _semaphore(0), _handler(&_semaphore, this), _alarm(conf.period, &_handler, conf.times) {
        if (Criterion::monitoring) {
            _times_p_count = conf.times;
            _alarm_times = &_alarm;
        }
        if((conf.state == READY) || (conf.state == RUNNING)) {
            _state = SUSPENDED;
            resume();
        } else
            _state = conf.state;
    }

    const Microsecond & period() const { return _alarm.period(); }
    void period(const Microsecond & p) { _alarm.period(p); }

    static volatile bool wait_next() {
        
        Periodic_Thread * t = reinterpret_cast<Periodic_Thread *>(running());
        //calc ddl misses and capture
        if (Criterion::monitoring && !_end_capture) {
            unsigned int entry_temp = CPU::temperature();   
            t->_missed_deadlines = t->_times_p_count - (t->_alarm_times->_times);

            float channel_3 = 0;
            float channel_5 = 0;
            float channel_6 = 0;

            if (Criterion::heuristic) {
                if (t->_missed_deadlines > 0) {
                    _clock_factor[Machine::cpu_id()] = 8;
                } else if (_thread_monitor->last_capture(Machine::cpu_id(), 0) > 0) {
                    unsigned long long ts = _thread_monitor->last_capture(Machine::cpu_id(), 7);
                    unsigned long long ts_dif = _thread_monitor->time() - ts;
                    if (ts_dif > 999) {
                        channel_3 = (PMU::read(3) - _thread_monitor->last_capture(Machine::cpu_id(), 3)) / (ts_dif * 1.0);
                        channel_5 = (PMU::read(5) - _thread_monitor->last_capture(Machine::cpu_id(), 5)) / (ts_dif);
                        channel_6 = (PMU::read(6) - _thread_monitor->last_capture(Machine::cpu_id(), 6)) / (ts_dif);
                        if (channel_3 <= 20) {
                             //if (Machine::cpu_id() == 6)
                             //    db<Thread> (WRN) << 30 << endl;
                            if (channel_3 >= 8 && _clock_factor[Machine::cpu_id()] == 8) {
                                 if (_slowdown[Machine::cpu_id()] > 0)
                                     _slowdown[Machine::cpu_id()]--;
                            } else if (channel_3 > 10 && _clock_factor[Machine::cpu_id()] == 7) {
                                _clock_factor[Machine::cpu_id()]++;
                            } else if (channel_3 >= 0.8 && _clock_factor[Machine::cpu_id()] == 6) {
                                _clock_factor[Machine::cpu_id()]++;
                            }  else if (channel_3 >= 0.2 && _clock_factor[Machine::cpu_id()] < 6) {
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
                CPU::clock((CPU::clock()/8 * _clock_factor[Machine::cpu_id()]));
            }
            
            unsigned long long pkg = CPU::pkg_energy_status();
            unsigned long long pp0 = CPU::pp0_energy_status();
            _thread_monitor->capture(entry_temp, PMU::read(0), PMU::read(1), PMU::read(2), PMU::read(3), PMU::read(4), PMU::read(5),
                PMU::read(6), reinterpret_cast<volatile unsigned int>(t), t->priority(), Machine::cpu_id(), t->_missed_deadlines, t->_global_deadline_misses, pkg, pp0);
        }    
        //end capture code
        
        if(t->_alarm._times) {
            t->_semaphore.p();
            //db<Thread>(WRN)<<"times: "<<t->_alarm._times<<endl;
            t->_times_p_count--;
        }

        return t->_alarm._times;
    }

protected:
    Semaphore _semaphore;
    Handler _handler;
    Alarm _alarm;
};

class RT_Thread: public Periodic_Thread
{
public:
    enum {
        SAME        = Scheduling_Criteria::RT_Common::SAME,
        NOW         = Scheduling_Criteria::RT_Common::NOW,
        UNKNOWN     = Scheduling_Criteria::RT_Common::UNKNOWN,
        ANY         = Scheduling_Criteria::RT_Common::ANY
    };

public:
    RT_Thread(void (* function)(), const Microsecond & deadline, const Microsecond & period = SAME, const Microsecond & capacity = UNKNOWN, const Microsecond & activation = NOW, int times = INFINITE, int cpu = ANY, const Color & color = WHITE, unsigned int stack_size = STACK_SIZE)
    : Periodic_Thread(Configuration(activation ? activation : period ? period : deadline, activation ? 1 : times, SUSPENDED, Criterion(deadline, period ? period : deadline, capacity, cpu), color, 0, stack_size), &entry, this, function, activation, times) {
        if(activation && Criterion::dynamic)
            // The priority of dynamic criteria will be adjusted to the correct value by the
            // update() in the operator()() of Handler
            const_cast<Criterion &>(_link.rank())._priority = Criterion::PERIODIC;
        resume();
    }

private:
    static int entry(RT_Thread * t, void (*function)(), const Microsecond activation, int times) {
        if(activation) {
            // Wait for activation time
            t->_semaphore.p();

            // Adjust alarm's period
            t->_alarm.~Alarm();
            new (&t->_alarm) Alarm(t->criterion()._period, &t->_handler, times);
        }

        // Periodic execution loop
        do {
//            Alarm::Tick tick;
//            if(Traits<Periodic_Thread>::simulate_capacity && t->criterion()._capacity)
//                tick = Alarm::_elapsed + Alarm::ticks(t->criterion()._capacity);

            // Release job
            function();

//            if(Traits<Periodic_Thread>::simulate_capacity && t->criterion()._capacity)
//                while(Alarm::_elapsed < tick);
        } while (wait_next());

        return 0;
    }
};

typedef Periodic_Thread::Configuration RTConf;

__END_SYS

#endif
