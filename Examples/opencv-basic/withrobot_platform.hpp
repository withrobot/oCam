
#include <ctime>
#include <sstream>

#ifdef _MSC_VER
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace Withrobot {
#ifdef _MSC_VER
    class Mutex
    {
        CRITICAL_SECTION cs;

    public:
        Mutex() {
            InitializeCriticalSection(&cs);
        }

        ~Mutex() {
            DeleteCriticalSection(&cs);
        }

        inline void lock() {
            EnterCriticalSection(&cs);
        }

        inline void unlock() {
            LeaveCriticalSection(&cs);
        }
    };

    class Thread
    {
        DWORD thread_id;
        HANDLE thread;

    public:
        Thread() : thread(NULL), thread_id(0){}

        bool start(void*(*thread_proc)(void*), void* arg, size_t stack_size=16*1024) {
            thread = CreateThread(NULL, (DWORD)stack_size, (LPTHREAD_START_ROUTINE)thread_proc, arg, 0, &thread_id);
            return (thread != NULL);
        }

        void join() {
            if(thread != NULL) {
                WaitForSingleObject(thread, INFINITE);
                CloseHandle(thread);
            }
            thread = NULL;
            thread_id = 0;
        }
    };

    static void msleep(unsigned int msec) {
        Sleep(msec);
    }

#else

    class Mutex
    {
        pthread_mutex_t mutex;

    public:
        Mutex() {
            pthread_mutex_init(&mutex, NULL);
        }

        ~Mutex() {
            pthread_mutex_destroy(&mutex);
        }

        inline void lock() {
            pthread_mutex_lock(&mutex);
        }

        inline void unlock() {
            pthread_mutex_unlock(&mutex);
        }
    };

    class Thread
    {
        pthread_t thread;

    public:
        Thread() : thread(0){}

        bool start(void*(*thread_proc)(void*), void* arg, size_t stack_size=16*1024) {
            pthread_attr_t attr;
            size_t stacksize;
            pthread_attr_init(&attr);
            //pthread_attr_getstacksize(&attr, &stacksize);
            pthread_attr_setstacksize(&attr, stack_size);
            int res = pthread_create(&thread, &attr, thread_proc, (void*)arg);
            if(res != 0) {
                thread = 0;
            }

            return (res == 0);
        }

        void join() {
            if(thread != 0) {
                pthread_join(thread, NULL);
            }
            thread = 0;
        }
    };

    static void msleep(unsigned int msec) {
        usleep(msec*1000);
    }

#endif  /* _MSC_VER */


    template<typename T>
    std::string to_string(const T num)
    {
        std::ostringstream oss;
        oss << num;
        return oss.str();
    }

    class Timer
    {
        std::time_t t_start;
        std::time_t t_end;
    public:
        Timer() : t_end(0) {
            std::time(&t_start);
        }

        int now() {
            std::time(&t_end);
            int elapsed = static_cast<int>(t_end - t_start);
            t_start = t_end;
            return elapsed;
        }


    };

}   /* namespace Withrobot */
