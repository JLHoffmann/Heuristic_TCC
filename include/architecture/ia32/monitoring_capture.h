#ifndef __ia32_monitoring_capture_h
#define __ia32_monitoring_capture_h

#include <machine.h>
#include <pmu.h>
#include <nic.h>
#include <tsc.h>

__BEGIN_SYS
struct Moment {
    unsigned long long _temperature;
    unsigned long long _pmu0;
    unsigned long long _pmu1;
    unsigned long long _pmu2;
    unsigned long long _pmu3;
    unsigned long long _pmu4;
    unsigned long long _pmu5;
    unsigned long long _pmu6;
    unsigned long long _time_stamp;
    unsigned long long _thread_id;
    long long _thread_priority;
    unsigned long long _deadline;
    unsigned long long _global_deadline;
    unsigned long long _pkg_energy;
    unsigned long long _pp0_energy;    

    //"constructor"
    Moment() {}

    // used to make a copy of values
    // static void copy (Moment from, Moment *to) {
    //     to->_temperature = from._temperature;
    //     to->_pmu0 = from._pmu0;
    //     to->_pmu1 = from._pmu1;
    //     to->_pmu2 = from._pmu2;
    //     to->_pmu3 = from._pmu3;
    //     to->_pmu4 = from._pmu4;
    //     to->_pmu5 = from._pmu5;
    //     to->_pmu6 = from._pmu6;
    //     to->_time_stamp = from._time_stamp;
    //     to->_thread_id = from._thread_id;
    //     to->_thread_priority = from._thread_priority;
    //     to->_deadline = from._deadline;
    //     to->_global_deadline = from._global_deadline;

    // }
};

class Monitoring_Capture {

public:
	Moment * _mem_moment; //pointer to vector of moments
	unsigned int _init_pos[Traits<Build>::CPUS]; //initial positions of each cpu
    unsigned int _mem_pos[Traits<Build>::CPUS]; // number of stored captures of each cpu
    int _max_size; //max size of the cpu subvectors
	unsigned int _over[Traits<Build>::CPUS]; //number of captures unstored per cpu
    
    // smart_data_info
    unsigned long long _t0; //initial execution time
    unsigned long long _t1; //final execution time
    NIC::Address _mac; //mac address -> not used anymore
    unsigned long long _tsc_base; //base tsc time
    unsigned int _units[12]; //units vector (ignoring rapl)
    //coordinates
    int _x; 
    int _y;
    int _z;
    unsigned int _r; //radius - distance from the central point (x,y,z)
    unsigned int _errorsmart; //smardata error value
    unsigned int _confidence; //confidence smartdata value

public:

    //constructor
    Monitoring_Capture(int size, Moment * init) {
    	//storing config
        _max_size = size;
    	for (unsigned int i = 0; i < Traits<Build>::CPUS; i++) {
    		_mem_pos[i] = i * size;
    		_init_pos[i] = _mem_pos[i];
    		_over[i] = 0;
    	}
        _mem_moment = init;
        //metadata config
        _t0 = RTC::seconds_since_epoch() * 1000000;
        _t1 = _t0 + (5*60*1000000);
        _tsc_base = _t0 - (TSC::time_stamp() * 1000000 / TSC::frequency());
        NIC nic;
        _mac = nic.address();
        for (int i = 0; i < 8; i++)
            _units[i] = (i+1) << 16 | 8;
        _units[8] = ((PMU::_channel_3+8)+1) << 16 | 8;
        _units[9] = ((PMU::_channel_4+8)+1) << 16 | 8;
        _units[10] = ((PMU::_channel_5+8)+1) << 16 | 8;
        _units[11] = ((PMU::_channel_6+8)+1) << 16 | 8;
        _x =  3746654;//3765.829 * 100000;
        _y = -4237592;
        _z = -293735;
        _r = 0;
        _errorsmart = 0;
        _confidence = 1;
        //print basic metadata before system execution
        print_smart_params();
        series();
    }

    //basic destructor
    ~Monitoring_Capture () {
        delete _mem_moment;
    }

    //stores a capture using the params
    void capture(unsigned int temperature, unsigned long long pmu0, unsigned long long pmu1, unsigned long long pmu2, unsigned long long pmu3, unsigned long long pmu4, unsigned long long pmu5,
                unsigned long long pmu6, unsigned int thread_id, long long thread_priority, unsigned int cpu_id, unsigned int deadline, unsigned int global_deadline, unsigned long long pkg, unsigned long long pp0) {
        if (_mem_pos[cpu_id] < ((cpu_id+1) * _max_size)) {
            unsigned int pos = _mem_pos[cpu_id];
            _mem_moment[pos]._temperature = (unsigned long long)temperature;
            _mem_moment[pos]._pmu0 = pmu0;
            _mem_moment[pos]._pmu1 = pmu1;
            _mem_moment[pos]._pmu2 = pmu2;
            _mem_moment[pos]._pmu3 = pmu3;
            _mem_moment[pos]._pmu4 = pmu4;
            _mem_moment[pos]._pmu5 = pmu5;
            _mem_moment[pos]._pmu6 = pmu6;
            _mem_moment[pos]._time_stamp = _tsc_base + (TSC::time_stamp() * 1000000 / TSC::frequency());
            _mem_moment[pos]._thread_id = (unsigned long long)thread_id;
            _mem_moment[pos]._thread_priority = (long long)thread_priority;
            _mem_moment[pos]._deadline = (unsigned long long)deadline;
            _mem_moment[pos]._global_deadline = (unsigned long long)global_deadline;
            _mem_moment[pos]._pkg_energy = pkg;
            _mem_moment[pos]._pp0_energy = pp0;
            //Moment::copy(m, &_mem_moment[_mem_pos[cpu_id]]);
            _mem_pos[cpu_id]++;
           // if (!_circular_cpu_data[cpu_id]->next()) {
           //      send(cpu_id);
           // }
           // _circular_cpu_data[cpu_id]->insert(m);
        }
         /*else {
            _over[cpu_id]++;
        }//*/
    }

    //returns time of the system (reuse of the values)
    unsigned long long time () {
        return _tsc_base + (TSC::time_stamp() * 1000000 / TSC::frequency());
    }

    //returns the value of the last capture of a channel/event on a cpu
    unsigned long long last_capture(unsigned int cpu_id, unsigned int channel) {
        unsigned int pos = _mem_pos[cpu_id] - 1;
        if (pos < 0) {
            return 0;
        }
        switch (channel) {
            case 0:
                return _mem_moment[pos]._pmu0;
            case 1:
                return _mem_moment[pos]._pmu1;
            case 2:
                return _mem_moment[pos]._pmu2;
            case 3:
                return _mem_moment[pos]._pmu3;
            case 4:
                return _mem_moment[pos]._pmu4;
            case 5:
                return _mem_moment[pos]._pmu5;
            case 6:
                return _mem_moment[pos]._pmu6;
            default:
                return _mem_moment[pos]._time_stamp;
        }
        // if (channel == 0) 
        //     return _mem_moment[pos]._pmu0;
        // if (channel == 1) 
        //     return _mem_moment[pos]._pmu1;
        // if (channel == 2) 
        //     return _mem_moment[pos]._pmu2;
        // if (channel == 3) 
        //     return _mem_moment[pos]._pmu3;
        // if (channel == 4) 
        //     return _mem_moment[pos]._pmu4;
        // if (channel == 5) 
        //     return _mem_moment[pos]._pmu5;
        // if (channel == 6) 
        //     return _mem_moment[pos]._pmu6;
    }

    void series();

    void datas();

    void print_smart_params();

    //returns the position of the storing of a cpu
    unsigned int  get_mem_pos(int cpu_id) {
        return _mem_pos[cpu_id];
    }
    
};

__END_SYS

#endif //monitoring_capture_h
