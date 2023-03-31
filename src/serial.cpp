#include "serial.h"

#include <iostream>


#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


Serial::Serial(): buffer_('+'){
    
}

Serial::Serial(char* arg): buffer_('+'), port_opened(false){
    open_port(arg);
}

bool Serial::is_open(){
    return port_opened;
}

bool Serial::open_port(char* port){
        // Open the serial port
    serial_port_ = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port_ == -1){
        port_opened = false;
        return false;
    }

    // Configure the serial port
    struct termios options;
    tcgetattr(serial_port_, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcsetattr(serial_port_, TCSANOW, &options);
    port_opened = true;
    return true;
}

void Serial::close_port(){
    close(serial_port_);
}
Serial::~Serial(){

    close_port();
}

int Serial::write_to_port(const char* message, size_t length){
    return write(serial_port_, message, length);
}

int Serial::read_from_port(char* message){
    //char read_buffer[32];
    int bytes_read = read(serial_port_, &buffer_.temp_buffer, sizeof(buffer_.temp_buffer));
    bytes_read = buffer_.process(bytes_read); 
    if (bytes_read > 0){
        buffer_.read_data(message);
    }
    return bytes_read;
}

void Serial::read_until_flag(){
    return;
}
