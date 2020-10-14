#include <ros/ros.h>

#include <iostream>
#include <thread>

class PipeCommsManager {

    public:
        PipeCommsManager(const std::string& name, const int* pipes, const int pid);
        ~PipeCommsManager();

        void run_listener();
        void listen_to_program();
        bool close_pipes();

        const int get_read_fd();

    private:
        // int pipe_cpp_to_py[2];
        int pipe_py_to_cpp[2];
        const std::string program_name;

        pid_t pid_result;
        bool status;
        bool run;

        std::thread listener_thread;

        //set up stuff to read msgs across pipes
        bool read_uint32(uint32_t &value);
        bool read_string(std::string& output,uint32_t size);

    
};