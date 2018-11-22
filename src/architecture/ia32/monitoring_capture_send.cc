#include <architecture/ia32/monitoring_capture.h>
#include <utility/ostream.h>
__BEGIN_SYS

OStream cout;
//printing captured data -> comment line 9 to print all the data
void Monitoring_Capture::datas() {
	unsigned long long final_time = RTC::seconds_since_epoch() *1000000;
	/*
	cout << "<end_capture>" << endl;
	for (unsigned int i = 0; i < Machine::n_cpus(); i++) {
		cout << "init_temp" << i << ": " << _mem_moment[_init_pos[i]]._temperature << endl;
		cout << "cap  CPU" << i << ": " << _mem_pos[i] - _init_pos[i] << endl;
		cout << "over CPU" << i << ": " << _over[i] << endl;
		cout << "deadlines " << i << ": " <<_mem_moment[_mem_pos[i]-1]._deadline << endl;
	}
	cout << "global_ddl_m" << ": " <<_mem_moment[_mem_pos[0]-1]._global_deadline << endl;
	//cout << "final time | Elapsed time: " << final_time << " | " << (final_time - _t0)/1000000 << endl;
	while(1) cout<<"simulation ended"<<endl;
	//*/
	cout << "<begin_capture>" << endl;
	Moment m;
	for (unsigned int i = 0; i < Machine::n_cpus(); i++) {
		//continue; //discomment if you don't want to print the collected data
		for (unsigned int j = _init_pos[i]; j < _mem_pos[i]; j++) {
			m = _mem_moment[j];
			unsigned long long value[] = {m._temperature, m._pmu0, m._pmu1, m._pmu2, m._pmu3, m._pmu4, m._pmu5, m._pmu6,
						m._time_stamp, m._thread_id, m._thread_priority, i, m._deadline, m._global_deadline, m._pkg_energy, m._pp0_energy};
			cout << "<";
			for (int i = 0; i < 15; i++) {
				cout << value[i] << ",";
			}
			cout << value[15] << ">" << endl;
		}
		//break;
	}
	cout << "<end_capture>" << endl;
	for (unsigned int i = 0; i < Machine::n_cpus(); i++) {
		cout << "cap  CPU" << i << ": " << _mem_pos[i] - _init_pos[i] << endl;
		cout << "over CPU" << i << ": " << _over[i] << endl;
	}
	cout << "final time | Elapsed time: " << final_time << " | " << (final_time - _t0)/1000000 << endl;
	while(1) cout<<"simulation ended"<<endl;
}

//print series(metadata)
void Monitoring_Capture::series() {
    //*
    cout << "<begin_series>" << endl;
    // cout << "+\n"
    // << "{\n"
    // << "\t\"series\" :\n"
    // << "\t{\n"
    // << "\t\t\"version\" : \"1.1\",\n"
    // << "\t\t\"unit\" : " << _units[0] << ",\n"
    // << "\t\t\"x\" : " << _x << ",\n"
    // << "\t\t\"y\" : " << _y << ",\n"
    // << "\t\t\"z\" : " << _z << ",\n"
    // << "\t\t\"r\" : " << _r << ",\n"
    // << "\t\t\"t0\" : " << _t0 << ",\n"
    // << "\t\t\"t1\" : " << _t1 << ",\n"
    // << "\t\t\"dev\" : " << 0 << "\n"
    // << "\t}\n"
    // << "}\n"
    // << "+" << endl;
    for (unsigned int cpu = 0; cpu < Machine::n_cpus(); cpu++)
		for (int i = 0; i < 12; i++) {
				if (i != 4) {
					cout << "+\n"
					<< "{\n"
					<< "\t\"series\" :\n"
					<< "\t{\n"
					<< "\t\t\"version\" : \"1.1\",\n"
					<< "\t\t\"unit\" : " << _units[i] << ",\n"
					<< "\t\t\"x\" : " << _x + cpu*10 << ",\n"
					<< "\t\t\"y\" : " << _y << ",\n"
					<< "\t\t\"z\" : " << _z << ",\n"
					<< "\t\t\"r\" : " << _r << ",\n"
					<< "\t\t\"t0\" : " << _t0 << ",\n"
					<< "\t\t\"t1\" : " << _t1 << ",\n"
					<< "\t\t\"dev\" : " << cpu << "\n"
					<< "\t}\n"
					<< "}\n"
					<< "+" << endl;
				}
		}
	cout << "<end_series>" << endl;//*/
}
//print the parameters to mount the smartdatas
void Monitoring_Capture::print_smart_params() {
	//*
	cout << "<begin_params>" << endl;
	cout << "<" << _mac << ",";
	for (int i = 0; i < 12; i++)
		cout << _units[i] << ",";
	cout << _t0 << ",";
	cout << _t1 << ">" << endl;
	cout << "<end_params>" << endl;
	//*/
}

__END_SYS

