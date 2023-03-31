#pragma once

#include <stddef.h>
#define MAX_BUFFER_SIZE 1024

class Buffer{
public:
    Buffer();
    Buffer(char special_character);
    ~Buffer();
    void setSpecialCharacter(char special_character);
    int process(size_t length);
    void read_data(char* outter_buff);
    //this buffer is to be passed to serial
    char* temp_buffer;
private:
    void strcpy_custom(char* dest, const char* src, size_t n, bool null_term);
    size_t strchr_custom(const char* str, char ch, size_t n); 

    char* buffer0_;
    char* buffer1_;

    size_t length0_;
    size_t length1_;
    char special_character_;
    bool active_buffer; 
    
};