#include <iostream>

class PipeCommsManager {

    public:
        PipeCommsManager();
        ~PipeCommsManager();
        int pipe_cpp_to_py[2];
        int pipe_py_to_cpp[2];

        pid_t pid_result;
        bool status;

    
};