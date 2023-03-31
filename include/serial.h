#pragma once

#include<stddef.h>
#include "buffer.h"


class Serial{
public:
    Serial(char * arg);
    Serial();
    
    ~Serial();
    bool open_port(char* port);
    bool is_open();
    void close_port();
    int write_to_port(const char* message, size_t length);

    void set_delimeter(char sc){buffer_.setSpecialCharacter(sc);};
    
    int read_from_port(char* message);
    void read_until_flag();
    
    int serial_port_;

private:
    Buffer buffer_;
    bool port_opened;
};