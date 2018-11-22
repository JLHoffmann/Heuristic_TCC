#include <periodic_thread.h>
#include <utility/random.h>
#include <clock.h>
///*
#include <machine/pc/rtc.h>
#include <chronometer.h>
#include <utility/ostream.h>
#include <tsc.h>
//*/

using namespace EPOS;


#define BASE_MEMCPY_T 30
#define BASE_RECURSIVE_FIB_T 600 
#define THREADS 12 //number of real-time tasks, usually, threads_parameters size
#define SECONDS 600 //execution time in seconds

OStream cout;
// period (microsecond), deadline, execution time (microsecond), cpu (partitioned)

unsigned int iterations[THREADS];
/*
unsigned int threads_parameters[][4] = { //heavy weight taskset
{ 50000 , 50000 , 27098 , 0 },
{ 25000 , 25000 , 5504 , 1 },
{ 100000 , 100000 , 68919 , 2 },
{ 100000 , 100000 , 64664 , 3 },
{ 50000 , 50000 , 9310 , 4 },
{ 200000 , 200000 , 105758 , 5 },
{ 200000 , 200000 , 29326 , 6 },
{ 200000 , 200000 , 67222 , 7 },
{ 50000 , 50000 , 21151 , 6 },
{ 50000 , 50000 , 6757 , 4 },
{ 50000 , 50000 , 34329 , 1 },
{ 50000 , 50000 , 8203 , 4 },
{ 100000 , 100000 , 44566 , 7 },
{ 25000 , 25000 , 8853 , 4 }
};
//*/
//*
unsigned int threads_parameters[][4] = { //light weight taskset
{ 25000 , 25000 , 13958 , 0 },
{ 100000 , 100000 , 15135 , 1 },
{ 200000 , 200000 , 136986 , 2 },
{ 50000 , 50000 , 25923 , 3 },
{ 25000 , 25000 , 11637 , 4 },
{ 100000 , 100000 , 20072 , 5 },
{ 50000 , 50000 , 30484 , 6 },
{ 200000 , 200000 , 25220 , 7 },
{ 200000 , 200000 , 23924 , 7 },
{ 100000 , 100000 , 31920 , 1 },
{ 50000 , 50000 , 18343 , 5 },
{ 50000 , 50000 , 19205 , 7 } };
//*/

//calculates number of iterations to be executed to accomplish execution time in SECONDS
int calc_iterations() {
    int sum = 0;
    //float base = 0;
    for (int i = 0; i < THREADS; i++) {
        if (iterations[i] == 0) {
    //        cout<<i<<" "<<threads_parameters[i][0]<<endl;
            iterations[i] = (SECONDS * 1000000) / threads_parameters[i][0];
        }
        sum+=iterations[i];
        cout<<threads_parameters[i][0]<<" "<<iterations[i]<<endl;
    }
    return sum;
}

//fibonacci recursive method
int fib(int pos) {
    if (pos == 1 || pos == 0) {
        return 1;
    } else {
        return (fib(pos - 1) + fib(pos - 2));
    }

}
/* //This is used to collect information about the time one execution */

/*unsigned int chronos[THREADS][ITERATIONS]; //capture time of each iteration
float mean_chronos[THREADS];
unsigned int pos_chronos[THREADS];
unsigned int total_chronos[THREADS];
unsigned int id_Thread[THREADS];

void mean() {
	cout << "Mean Execution Time per Iteration: " << endl;
	int greater = 0;
    for (int i = 0; i < THREADS; i++) {
    	cout << "Thread[" << i << "] " << " - Cluster = " << threads_parameters[i][3]*2 << endl;
        mean_chronos[i] = 0;
        for (int j = 0; j < pos_chronos[i]; j++) {
        	if (chronos[i][j] > greater)
        		greater = chronos[i][j];
 			cout << "Iter[" << j << "] = " << chronos[i][j] << endl;
            mean_chronos[i] += chronos[i][j];
        }
        mean_chronos[i] /= pos_chronos[i];
        cout << "Mean      = " << mean_chronos[i] << "us; Worst Case = " << greater << "us" << endl;
        cout << "Hard Mean = " << total_chronos[i]/ITERATIONS << "us" << endl;
        cout << "Max iter  = " << (threads_parameters[i][2]/BASE_MEMCPY_T)+1 << endl; 
        cout << "--------------------------------------" << endl;
    }

    
}
//*/

//memcpy vector
int vectors[Traits<Build>::CPUS][32768];

//high priority threads vector (REAL-TIME)
Periodic_Thread * threads[THREADS];

//low priority threads vector
Thread *low_Threads[8];

//definition of clock variable used to calculate random seed
Clock clock;

//thread id vector
unsigned int threads_id[THREADS];

//method that configures iterative fibonacci execution
int iterative_fib_test (int id) {
    float ret = 1.33;
    int fib = 1;
    int temp = 1;
    int prev = 1;
    int max = (int) ((int(threads_parameters[id][2])));
    for (int i = 0; i < iterations[id]; i++) {
        Periodic_Thread::wait_next();
        for (int x = 0; x < max; x++) {
            fib = 1;
            prev = 1;
            for (int j = 1; j < 1000; j++) {
                temp = prev+fib;
                prev = fib;
                fib = temp;
            }
            ret *= fib;
        }
    }
    return int(ret);
}

//method that runs fibonacci recursively
int fib_test (int id) {
    float ret = 1.33;
    int max = (int) ((int(threads_parameters[id][2])/BASE_RECURSIVE_FIB_T));
    for (int i = 0; i < iterations[id]; i++) {
        Periodic_Thread::wait_next();
        for (int x = 0; x < max; x++) {
            ret *= fib(25);
        }
    }
    return int(ret);
}

//method that runs memcpy method
int test(int id) {
    Random * rand;
    float ret = 1.33;
    int max = (int) ((threads_parameters[id][2]/BASE_MEMCPY_T));
    for (int i = 0; i < iterations[id]; i++) {
        Periodic_Thread::wait_next();
        rand->seed(clock.now());
        for (int x = 0; x < max; x++) {
            memcpy(reinterpret_cast<void *>(&vectors[(x*3)%3]),reinterpret_cast<void *>(&vectors[rand->random() % Machine::n_cpus()]),sizeof(vectors[0]));
        }
    }
    return int(ret);
}

//method to execute low_priority tasks, uncomment what you want to be executed
int low_priority() {
    int x = 0;

//*
    float result = 1.33;
    //cout << Machine::cpu_id() << endl;
    while (!Thread::_end_capture) {
        result = result*fib(25);
        x++;
    }
    return int(result); //*/
/*
    Random * rand;
    while (!Thread::_end_capture) {
        memcpy(reinterpret_cast<void *>(&vectors[(x*3)%3]),reinterpret_cast<void *>(&vectors[rand->random() % Machine::n_cpus()]),sizeof(vectors[0]));
        memcpy(reinterpret_cast<void *>(&vectors[(x*2)%3]),reinterpret_cast<void *>(&vectors[rand->random() % Machine::n_cpus()]),sizeof(vectors[0]));
        memcpy(reinterpret_cast<void *>(&vectors[(x)%3]),reinterpret_cast<void *>(&vectors[rand->random() % Machine::n_cpus()]),sizeof(vectors[0]));
        x++;
    }
    return 0; //*/

/*
    int fib = 1;
    int prev = 1;
    int temp = 1;
    while (!Thread::_end_capture) {
        temp = fib+prev;
        prev = fib;
        fib = temp;
    }

    return fib; //*/
}


int main()
{    
    Thread::_end_capture = true;
	// initializing the vectors used to stress CPU
    for (int m = 0; m < Machine::n_cpus(); m++) {
        for (int i = 0; i < 32768; i++) {
            vectors[m][i] = i * m - 2 * m;
        }
    }
    
    int numero = calc_iterations();
    Thread::_end_capture = false;
    // start
    unsigned long long init_time = ((TSC::time_stamp() * 1000000)/TSC::frequency());
    /*
    *task configuration, to execute memcpy operations change "fib_test" to "test", 
    *for iterative method replace to "iterative_fib_test"
    */
    for(int i=0;i<THREADS;i++){
        threads[i] = new Periodic_Thread(RTConf(threads_parameters[i][0], iterations[i], Thread::READY ,
            Thread::Criterion((RTC::Microsecond) threads_parameters[i][0], (RTC::Microsecond) threads_parameters[i][1],
            (RTC::Microsecond) int(threads_parameters[i][2]), (threads_parameters[i][3]))), &fib_test, i);
        threads_id[i] = reinterpret_cast<volatile unsigned int>(threads[i]);
    }
    /*
    for(int i=0;i<8;i++){
        Thread::Configuration   conf(Thread::READY,
                        Thread::Criterion(Thread::LOW,i)
                        );
        low_Threads[i] = new Thread(conf, &low_priority);
    }
    //*/
    // sync
    for(int i=0; i<THREADS;i++) {
        threads[i]->join();
    }

    //mean(); //use to calculate execution mean time

    // stop capturing
    Thread::_end_capture = true;
    
    /* uncomment this and line 238
    for(int i = 0; i < 8; i++)
       low_Threads[i]->join();
    //*/

    // used to print a series to current deadline misses, wich is thread bound, not cpu bound.
    cout<<"time = "<<(((TSC::time_stamp() * 1000000)/TSC::frequency()) - init_time)<<endl;
    
    //used to print thread_ids for sending algorithms
    /* 
    cout << "<begin_tseries>" << endl;
    cout << "<";
    for (int i = 0; i < THREADS-1; i++)
        cout << threads_id[i] << ",";
    cout << threads_id[THREADS-1] << ">" << endl;
    cout << "<end_tseries>" << endl;
    //end */

    for(int i = 0; i < THREADS; i++)
        delete threads[i];

    return 0;
}
