#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include "ListaEnc.hpp"
#include <unistd.h>
#include <string.h>
#include <bitset>
#include <math.h>
//params
//unsigned char _mac[16];
std::string _mac = "0000000000000000";
std::string _units[12];
int _x = 3746654;
int _y =-4237592;
int _z =-293735;
int binaries = 0; 
std::string binary_string = "";
int chars_capturados = 0;
char vetor_de_impressao[510];

std::string reverterBytes(std::string src) {
	std::string dst = "";
	std::string aux[src.size()/8];

	for (int i = 0; i < src.size(); i+=8) {
		aux[i/8] = "";
		aux[i/8] += src[i];
		aux[i/8] += src[i+1];
		aux[i/8] += src[i+2];
		aux[i/8] += src[i+3];
		aux[i/8] += src[i+4];
		aux[i/8] += src[i+5];
		aux[i/8] += src[i+6];
		aux[i/8] += src[i+7];
	}

	for (int i = (src.size()/8)-1; i >= 0 ; i--) {
		dst += aux[i];
	}

	return dst;
}


struct DB_Series {
    unsigned char version;
    unsigned long unit;
    long x;
    long y;
    long z;
    unsigned long r;
    unsigned long long t0;
    unsigned long long t1;
};

struct DB_Record {
    unsigned char version;
    unsigned long unit;
    double value;
    unsigned char error;
    unsigned char confidence;
    long x;
    long y;
    long z;
    unsigned long long t;
    unsigned char mac[16];
};

char *rec = new char[sizeof(DB_Record)];

struct Moment {
	//attributes
    unsigned int _temperature;
    unsigned int _pmu0;
    unsigned int _pmu1;
    unsigned int _pmu2;
    unsigned int _pmu3;
    unsigned int _pmu4;
    unsigned int _pmu5;
    unsigned int _pmu6;
    unsigned long _time_stamp;
    unsigned int _thread_id;
    unsigned int _thread_priority;
    unsigned int _cpu_id;
    unsigned int _deadline;
    unsigned int _global_deadline;

    //methods
    Moment() {}
};

void initialize_params(std::string _param_text) {
	std::string params[13];
	int caught = 0;
	std::string temp = "";
	for (int i = 0; i < _param_text.length(); i++){
		if ((char)_param_text[i] != ',')
			temp+=_param_text[i];
		else {
			params[caught] = temp;
			temp = "";
			caught++;
		}
	}
	params[caught] = temp;
	_mac = params[0];
	_units[0] = params[1];
	_units[1] = params[2];
	_units[2] = params[3];
	_units[3] = params[4];
	_units[4] = params[5];
	_units[5] = params[6];
	_units[6] = params[7];
	_units[7] = params[8];
	_units[8] = params[9];
	_units[9] = params[10];
	_units[10] = params[11];
	_units[11] = params[12];

}

Moment from_string_to_moment(std::string vector[]) {
	Moment m;
	m._temperature = (unsigned int)std::stoi(vector[0], nullptr, 0);
	m._pmu0 = (unsigned int)std::stoull(vector[1], nullptr, 0);
	m._pmu1 = (unsigned int)std::stoull(vector[2], nullptr, 0);
	m._pmu2 = (unsigned int)std::stoull(vector[3], nullptr, 0);
	m._pmu3 = (unsigned int)std::stoull(vector[4], nullptr, 0);
	m._pmu4 = (unsigned int)std::stoull(vector[5], nullptr, 0);
	m._pmu5 = (unsigned int)std::stoull(vector[6], nullptr, 0);
	m._pmu6 = (unsigned int)std::stoull(vector[7], nullptr, 0);
	m._time_stamp = std::stoull(vector[8], nullptr, 0);
	m._thread_id = (unsigned int)std::stoul(vector[9], nullptr, 0);
	m._thread_priority = (unsigned int)std::stoul(vector[10], nullptr, 0);
	m._cpu_id = (unsigned int)std::stoul(vector[11], nullptr, 0);
	m._deadline = (unsigned int)std::stoul(vector[12], nullptr, 0);
	m._global_deadline = (unsigned int)std::stoul(vector[13], nullptr, 0);
	return m;
}
	
ListaEnc<std::string> from_moment_to_json(Moment m) {
	ListaEnc<std::string>* _jsons = new ListaEnc<std::string>();
	unsigned long long value[] = {m._temperature, m._pmu0, m._pmu1, m._pmu2, m._pmu3, m._pmu4, m._pmu5, m._pmu6,
                     m._thread_id, m._thread_priority, m._deadline, m._global_deadline};
    for (int j = 0; j < 11; j++) {
        //continue;
   		std::string str = ("{\n"
            + (std::string)("\t\"smartdata\" :\n")
            + (std::string)("\t[\n")
            + (std::string)("\t\t{\n")
            + (std::string)("\t\t\t\"version\" : \"1.1\",\n")
            + (std::string)("\t\t\t\"unit\" : ") + _units[j] + (",\n")
            + (std::string)("\t\t\t\"error\" : ") + std::to_string(0) + (std::string)(",\n")
            + (std::string)("\t\t\t\"confidence\" : ") + std::to_string(1) + ",\n"
            + "\t\t\t\"x\" : " + std::to_string(_x + m._cpu_id*10) + ",\n"
            + "\t\t\t\"y\" : " + std::to_string(_y) + ",\n"
            + "\t\t\t\"z\" : " + std::to_string(_z) + ",\n"
            + "\t\t\t\"value\" : " + std::to_string(value[j]) + ",\n"
            + "\t\t\t\"time\" : " + std::to_string(m._time_stamp) + ",\n"
            + "\t\t\t\"mac\" : " + "\"" + _mac + "\"" + "\n"
            + "\t\t}\n"
            + "\t]\n"
    		+ "}\n");
        _jsons->adiciona(str);
    }
    std::string str = ("{\n"
        + (std::string)("\t\"smartdata\" :\n")
        + (std::string)("\t[\n")
        + (std::string)("\t\t{\n")
        + (std::string)("\t\t\t\"version\" : \"1.1\",\n")
        + (std::string)("\t\t\t\"unit\" : ") + _units[11] + (",\n")
        + (std::string)("\t\t\t\"error\" : ") + std::to_string(0) + (std::string)(",\n")
        + (std::string)("\t\t\t\"confidence\" : ") + std::to_string(1) + ",\n"
        + "\t\t\t\"x\" : " + std::to_string(_x) + ",\n"
        + "\t\t\t\"y\" : " + std::to_string(_y) + ",\n"
        + "\t\t\t\"z\" : " + std::to_string(_z) + ",\n"
        + "\t\t\t\"value\" : " + std::to_string(value[11]) + ",\n"
        + "\t\t\t\"time\" : " + std::to_string(m._time_stamp) + ",\n"
        + "\t\t\t\"mac\" : " + "\"" + _mac + "\"" + "\n"
        + "\t\t}\n"
        + "\t]\n"
		+ "}\n");
    _jsons->adiciona(str);
    return *_jsons;
}

void send_binary_records(/*std::string to_send*/ DB_Record reg) {
	std::string r("");
	std::string a[] = {"A", "B", "C", "D", "E", "F"};
	std::string aux = "";
	int v = 0;
	int acum = 0;
	if (binaries < 108) { 
		// converting DB_Record.value to bytes 
		char *bytes = reinterpret_cast<char *>(&reg.value);

		// the following code is used to convert to a complete binary 
		// (we use this because only converting to char * ignore the left '0's)
		// and ignoring it will cause to the data sended via http request to be out of format
		r += reverterBytes(std::bitset<1*8>(reg.version).to_string());
		r += reverterBytes(std::bitset<4*8>(reg.unit).to_string());
		for(int i = 0; i < 8; i++) {
			r += reverterBytes(std::bitset<1*8>(bytes[i]).to_string());
		}
		r += reverterBytes(std::bitset<1*8>(reg.error).to_string());
		r += reverterBytes(std::bitset<1*8>(reg.confidence).to_string());
		r += reverterBytes(std::bitset<4*8>(reg.x).to_string());
		r += reverterBytes(std::bitset<4*8>(reg.y).to_string());
		r += reverterBytes(std::bitset<4*8>(reg.z).to_string());
		r += reverterBytes(std::bitset<8*8>(reg.t).to_string());
		r += reverterBytes(std::bitset<16*8>(0).to_string());
		
		// as we get the binary above, we still need to convert it to a byte form (\xvv)
		// where \x correspons to hexa format and v is a hexa number (0...F).
		// The problem is that we can't write it direct as a hexa number, because python can't read it perfectly
		// So, we write it as an integer that represent that byte (0...255) and after convert it to a byte.
		for (int i = 0; i < r.size(); i+=8) {
			aux += r[i];
			aux += r[i+1];
			aux += r[i+2];
			aux += r[i+3];
			aux += r[i+4];
			aux += r[i+5];
			aux += r[i+6];
			aux += r[i+7];
			
			for (int j = 0; j < 8; j++) {
				v = aux[j]-48;
				acum += v * pow(2, 8-(j+1));
			}
			
			// for (int j = 4; j < 8; j++) {
			// 	v = aux[j]-48;
			// 	acum += v * pow(2, 8-(j+1));
			// }
			
			binary_string += std::to_string(acum);
			if (i < r.size() -8)
				binary_string += ",";
			acum = 0;
			aux = "";
		}
		binary_string+=",";
	} else {
		char *bytes = reinterpret_cast<char *>(&reg.value);
		r += reverterBytes(std::bitset<1*8>(reg.version).to_string());
		r += reverterBytes(std::bitset<4*8>(reg.unit).to_string());
		for(int i = 0; i < 8; i++) {
			r += reverterBytes(std::bitset<1*8>(bytes[i]).to_string());
		}
		r += reverterBytes(std::bitset<1*8>(reg.error).to_string());
		r += reverterBytes(std::bitset<1*8>(reg.confidence).to_string());
		r += reverterBytes(std::bitset<4*8>(reg.x).to_string());
		r += reverterBytes(std::bitset<4*8>(reg.y).to_string());
		r += reverterBytes(std::bitset<4*8>(reg.z).to_string());
		r += reverterBytes(std::bitset<8*8>(reg.t).to_string());
		r += reverterBytes(std::bitset<16*8>(0).to_string());

		for (int i = 0; i < r.size(); i+=8) {
			aux += r[i];
			aux += r[i+1];
			aux += r[i+2];
			aux += r[i+3];
			aux += r[i+4];
			aux += r[i+5];
			aux += r[i+6];
			aux += r[i+7];

			for (int j = 0; j < 4; j++) {
				v = aux[j]-48;
				acum += v * pow(2, 8-(j+1));
			}

			for (int j = 4; j < 8; j++) {
				v = aux[j]-48;
				acum += v * pow(2, 8-(j+1));
			}
			
			binary_string += std::to_string(acum);
			if (i < r.size() -8)
				binary_string += ",";
			acum = 0;
			aux = "";
		}
		std::ofstream out;
		out.open("strings.bin", std::ios::out | std::ios::binary);
		out<<binary_string;
		out.close();
		binary_string="";
		system("./smartdata_sender -c client-8-A7B64D415BD3E982");
		binaries=0;
		//system("rm -f strings.bin");*/
	}
	binaries++;
}

void from_rec_to_string(DB_Record r) {
	// for(unsigned int i = 0; i < sizeof(DB_Record); i++) 
 //        (rec[i]) = (reinterpret_cast<const char *>(&r)[i]);
    send_binary_records(r);
}

void from_moment_to_db_record(Moment m) {
	unsigned int value[] = {m._temperature, m._pmu0, m._pmu1, m._pmu2, m._pmu3, m._pmu4, m._pmu5, m._pmu6,
                     m._thread_id, m._thread_priority, m._deadline, m._global_deadline};
    DB_Record r;

	for (int i = 0; i < _mac.length(); i++)
		r.mac[i] = reinterpret_cast<unsigned char*>(&_mac)[i];
    for (int j = 0; j < 11; j++) {
        //continue;
		r.version = (unsigned char)((1<<4)|1<<0);
		r.unit = std::stoul(_units[j]);
		r.value = (value[j]);
		r.error = (unsigned char)0;
		r.confidence = (unsigned char)1;
		r.x = (long)_x+m._cpu_id*10;
		r.y = (long)_y;
		r.z = (long)_z;
		r.t = (unsigned long long)m._time_stamp;
		//r.mac = reinterpret_cast<unsigned char>(*strdup(_mac.c_str()));
		//r.mac = _mac;
		from_rec_to_string(r);
        //_recs->adiciona(r);
    }
	r.version = (unsigned char)((1 << 4) | (1 << 0));
	r.unit = std::stoul(_units[11]);
	r.value = (double)value[11];
	r.error = (unsigned char)0;
	r.confidence = (unsigned char)1;
	r.x = (long)_x;
	r.y = (long)_y;
	r.z = (long)_z;
	r.t = (unsigned long long)m._time_stamp;
	//strcpy(r.mac, _mac.c_str());
	//&r.mac = &_mac;
	//strcpy( static_cast <unsigned char*>( r.mac ), _mac );
    //_recs->adiciona(r);

}

void send_series(std::string to_send) {
	system("rm strings.json");
	std::ofstream out;
	out.open("strings.json");
	out<<to_send;
	out.close();
	system("./series.py -c client-8-A7B64D415BD3E982");
}


void send_smart_data(std::string to_send) {
	// system("rm strings.json");
	// std::ofstream out;
	// out.open("strings.json");
	// out<<to_send;
	// out.close();
	// system("python smartdata.py");
}

void test_json(std::string to_send) {
	system("rm strings.json");
	std::ofstream out;
	out.open("strings.json");
	out<<to_send;
	out.close();
	system("python code.py");
}

void from_file_to_json(std::string filename) {
	std::ifstream t(filename);
	int initial_position = 0;
	std::string str((std::istreambuf_iterator<char>(t)),std::istreambuf_iterator<char>());
	if (str == "\00") {return;}
	for (int i = 0; i < str.length(); i++) {
		if ((char)str[i] == '<') {
			if (str.compare(i+1, 13, "begin_capture") == 0) {
				initial_position = i+15+1;
				//std::cout << "ACHO" << std::endl;
			}
		}
	}
	//std::cout<< (char)str[initial_position] <<std::endl;
	std::string moment_line;
	std::string temp = "";
	std::string string_moment_vector[14];
	int moment_pos = 0;
	while (str.compare(initial_position, 13, "<end_capture>") != 0) {
		int j = initial_position+2;
		while ((char)str[j] != '+' && (char)str[j] != '<') {
			temp+=str[j];
			j++;
		}
		initial_position = j;
		//std::cout<<temp<<std::endl;
		if (temp != "") {
			//std::cout<<temp<<std::endl;
			test_json(temp);
		}
		//std::cin.ignore();
		//usleep(250000);
		temp = "";
		//initial_position++;
		if (initial_position + 13 >= str.length()){break;}
	}

}

class converter {
protected:
	ListaEnc<Moment> * captured;
	int printed = 0;
public:
	ListaEnc<Moment> read(std::string filename) {
		captured->destroiLista();
		std::ifstream t(filename);
		int initial_position = 0;
		std::string temp = "";
		std::string str((std::istreambuf_iterator<char>(t)),std::istreambuf_iterator<char>());
		if (str != "\00") {
			//caminha até <begin_params>
			for (int i = 0; i < str.length(); i++) {
				if ((char)str[i] == '<') {
					if (str.compare(i+1, 12, "begin_params") == 0) {
						initial_position = i+14+1;
						//std::cout << "ACHO" << std::endl;
					}
				}
			}
			std::string param = "";
			while (str.compare(initial_position, 12, "<end_params>") != 0) {
				if (!((char)str[initial_position] == '<' || (char)str[initial_position] == '>' || (char)str[initial_position] == '\n' || (char)str[initial_position] == '\r' )) {
					param += str[initial_position];
				}
				//std::cout<<param<<std::endl;
				initial_position+=1;
			}
			//std::cout<<"SAIU"<<std::endl;
			initialize_params(param);
			//std::cout<<"SAIU"<<std::endl;
			//std::cout<<" x = " << str[initial_position] <<std::endl;
			initial_position+=13;
			//std::cout<<" x = " << str[initial_position] <<std::endl;
			//caminha até <begin_series>
			for (int i = initial_position; i < str.length(); i++) {
				if ((char)str[i] == '<') {
					if (str.compare(i+1, 12, "begin_series") == 0) {
						initial_position = i+14+1;
						//std::cout << "ACHO" << std::endl;
					}
				}
			}
			while (str.compare(initial_position, 12, "<end_series>") != 0) {
				int j = initial_position+2;
				while ((char)str[j] != '+' && (char)str[j] != '<') {
					temp+=str[j];
					j++;
				}
				initial_position = j;
				//std::cout<<temp<<std::endl;
				if (temp != "" && temp.length() > 1) {
					//std::cout<<temp<<std::endl;
					send_series(temp);
				}
				//std::cin.ignore();
				//usleep(250000);
				temp = "";
				//initial_position++;
				if (initial_position + 12 >= str.length()){break;}
			}
			//caminha até <begin_capture>
			initial_position += 13;
			if (!(initial_position + 13 >= str.length())) {
				for (int i = initial_position; i < str.length(); i++) {
					//std::cout<<"char: "<<str[i]<<std::endl;
					if ((char)str[i] == '<') {
						if (str.compare(i+1, 13, "begin_capture") == 0) {
							initial_position = i+15+1;
							break;
							//std::cout << "ACHO" << std::endl;
						}
					}
				}
				//std::cout<<"SAIU: "<<str[initial_position]<<std::endl;
				//std::cout<< (char)str[initial_position] <<std::endl;
				std::string moment_line;
				std::string string_moment_vector[14];
				int moment_pos = 0;
				while (str.compare(initial_position, 13, "<end_capture>") != 0) {
					//definir captura do moment
					moment_pos = 0;
					temp = "";
					if ((char)str[initial_position] != '<') {while((char)str[initial_position] != '<')initial_position++;}
					int j = initial_position+1;
					while ((char)str[j] != '>') {j++;}
					moment_line = "";
					for (int i = initial_position+1; i < j; i++) {
						if ((char)str[j] != '\n' && (char)str[j] != '\r') {moment_line+=str[i];}
					}
					//std::cout<<moment_line<<std::endl;
					//fazer split do moment pela string em ','
					for (int i = 0; i < moment_line.length(); i++) {
						if ((char)moment_line[i] != ',' ) {
							if ((char)moment_line[i] != '\n' && (char)moment_line[i] != '\r') {
								temp+=moment_line[i];
								if(i == moment_line.length() - 1) {
									string_moment_vector[moment_pos] = temp;
									temp = "";
									moment_pos++;	
								}
							}
						} else {
							string_moment_vector[moment_pos] = temp;
							temp = "";
							moment_pos++;
						}
					}
					//converter do vetor de string para um moment valido
					//std::cout<<"find error to moment"<<std::endl;
					Moment k = from_string_to_moment(string_moment_vector);
					//std::cout<<"find error adding"<<std::endl;
					captured->adiciona(k);
					//std::cout<<"dados: "<<captured->getSize()<<std::endl;
					//std::cout<<"find error to json"<<std::endl;
					from_moment_to_db_record(k);
					ListaEnc<std::string> _json_string = from_moment_to_json(k);
					//std::cout<<"find error - not found"<<std::endl;
					while(!_json_string.listaVazia()) {
						send_smart_data(_json_string.retira());
						//_json_string.retira();
					}
					initial_position = j+2;
					if (initial_position + 13 >= str.length()){break;}
				}
			}
			std::cout<<"capture finalized: "<<printed<<std::endl;
		}

		return *captured;
	}

	converter() {captured = new ListaEnc<Moment>();}

};

int main() {
	converter *_conv = new converter();
	ListaEnc<Moment> _m = _conv->read("series.log");
	std::cout<<"Size: "<<_m.getSize()<<std::endl;
	int counters[8] = {0,0,0,0,0,0,0,0};
	while (!_m.listaVazia()) {
		Moment _mom = _m.retira();
		counters[_mom._cpu_id]+=1;
        // std::cout << "Moment: "       << std::endl;
        // std::cout << "Temperature = " << _mom._temperature << std::endl;
        // std::cout << "Canal 0     = " << _mom._pmu0 << std::endl;
        // std::cout << "Canal 1     = " << _mom._pmu1 << std::endl;
        // std::cout << "Canal 2     = " << _mom._pmu2 << std::endl;
        // std::cout << "Canal 3     = " << _mom._pmu3 << std::endl;
        // std::cout << "Canal 4     = " << _mom._pmu4 << std::endl;
        // std::cout << "Canal 5     = " << _mom._pmu5 << std::endl;
        // std::cout << "Canal 6     = " << _mom._pmu6 << std::endl;
        // std::cout << "Time_Stamp  = " << _mom._time_stamp << std::endl;
        // std::cout << "Thread::id  = " << _mom._thread_id << std::endl;
        // std::cout << "Thread::pri = " << _mom._thread_priority << std::endl;
        // std::cout << "CPU::id  = " << _mom._cpu_id << std::endl;
        // std::cout << "Deadline    = " << _mom._deadline << std::endl;
        // std::cout << "Global Deadlines = " << _mom._global_deadline << std::endl;
        // std::cout << "\n" << std::endl;
	}
	for (int i = 0; i < 8; i++) {
		std::cout<<"cpu "<<i<<" = "<<counters[i]<<std::endl;
	}
	return 0;
}
