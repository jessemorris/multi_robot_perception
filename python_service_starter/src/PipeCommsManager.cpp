#include "PipeCommsManager.hpp"
#include <signal.h>

#include <stdexcept>

PipeCommsManager::PipeCommsManager(const std::string& name, const int* pipes, const int pid) :
    program_name(name),
    pid_result(pid),
    run(true)
{
    //this one should be read_fd
    pipe_py_to_cpp[0] = pipes[0];
    pipe_py_to_cpp[1] = pipes[1];

}

PipeCommsManager::~PipeCommsManager() {}

void PipeCommsManager::run_listener() {
    ROS_INFO_STREAM("Beginning listener thread for python script: " << program_name);
    listener_thread = std::thread(&PipeCommsManager::listen_to_program, this);

}

const int PipeCommsManager::get_read_fd() {
    return pipe_py_to_cpp[0];
}

void PipeCommsManager::listen_to_program() {

    int read_fd = get_read_fd();
    uint32_t message_size;
    std::string python_message;


    while (run) {
        try {

            if (!read_uint32(message_size)) {
                throw std::runtime_error("Could not read uint32");
            }

            if (!read_string(python_message, message_size)) {
                throw std::runtime_error("Could not read message");
            }

            ROS_INFO_STREAM("[" << program_name << "] " << python_message);

        }
        catch(std::exception& e) {
            ROS_WARN_STREAM("Listen to program (" << program_name << ") exception. Closing pipes and exiting");
            shutdown(); 
            return;   
        }
    }


    //doing this is dangerous as we create the memory for pipe_py_to_cpp in a different space and then close it here
    //but cant think of another way of doing this as we need to listen in a thread!
    shutdown();    

}

//https://claytonrichey.com/post/c-cpp-python-pipe/
/* return true if val is set, false for EOF */
bool PipeCommsManager::read_uint32(uint32_t &value) {
    unsigned char msgSizeBuf[4];
    unsigned iBuf = 0;
    int read_fd = get_read_fd();

    while (iBuf < sizeof(msgSizeBuf))
    {
        ssize_t rc = ::read(read_fd, msgSizeBuf + iBuf, sizeof(msgSizeBuf) - iBuf);

        if (rc == 0)
        {
            return false;
        }
        else if (rc < 0 )
        {
            ROS_ERROR_STREAM(__func__ << "@" << __LINE__ << ":::Read ERROR");
            return false;
        }
        else
        {
            iBuf += rc;
        }
    }

    value = *(static_cast<uint32_t *>(static_cast<void *>(&msgSizeBuf[0])));
    
    return true;

}
bool PipeCommsManager::read_string(std::string& output,uint32_t size) {
    std::vector<char> msgBuf( size + 1 );
    msgBuf[ size ] = '\0';
    unsigned iBuf = 0;

    int read_fd = get_read_fd();

    while (iBuf < size)
    {
        ssize_t rc = ::read(read_fd, &(msgBuf[0]) + iBuf, size - iBuf);

        if ( rc == 0 )
        {
            ROS_WARN_STREAM(__func__ << "@" << __LINE__ << ":::EOF read during message from python script " << program_name);
            return false;
        }
        else if ( rc < 0 )
        {
            ROS_ERROR_STREAM(__func__ << "@" << __LINE__ << ":::Read ERROR during message from python script " << program_name);
            return false;
        }
        else
        {
            iBuf += rc;
        }
    }
    msgBuf.shrink_to_fit();
    std::string s(msgBuf.begin(), msgBuf.end());
    output = s;
    return true;
    // return std::string( &(msgBuf[0]) );
}


bool PipeCommsManager::shutdown() {
    ::close(pipe_py_to_cpp[1]);
    ros::Duration(2).sleep();
    ROS_INFO_STREAM("Killing child process " << pid_result);

    kill(pid_result, SIGKILL);
}

