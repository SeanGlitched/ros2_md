#include "buffer.h"

#include <cstring>
#include <stdexcept> // for std::runtime_error

Buffer::Buffer(): Buffer('\n'){}

Buffer::Buffer(char special_character): 
        length0_(0), 
        length1_(0),
        special_character_(special_character), 
        active_buffer(0)
{
    buffer0_ = new char[MAX_BUFFER_SIZE];
    buffer1_ = new char[MAX_BUFFER_SIZE];
    temp_buffer = new char[MAX_BUFFER_SIZE];
}

Buffer::~Buffer(){
    delete [] buffer0_;
    delete [] buffer1_;
    delete [] temp_buffer;
}

void Buffer::setSpecialCharacter(char special_character){
    special_character_ = special_character;
}
//length is the length of characters add to the temp buffer
int Buffer::process(size_t length){
    if(length == 0) return 0;
    char* forward_buffer = (active_buffer? buffer1_: buffer0_);
    size_t* fb_length_ = (active_buffer ? &length1_ : &length0_);

    size_t pos = strchr_custom(temp_buffer, special_character_, length) + 1; 
    if(1+pos + *fb_length_ > MAX_BUFFER_SIZE) 
        throw std::runtime_error("Buffer Overflow");
    if(pos!=0){
        strcpy_custom(forward_buffer+*fb_length_, temp_buffer, pos + 1, true);
        
        fb_length_ += pos; 
        active_buffer = !active_buffer;
        

        forward_buffer =    (active_buffer? buffer1_: buffer0_);
        fb_length_ =        (active_buffer? &length1_: &length0_);
    
        length -= pos;
    }
    if(*fb_length_ + length > MAX_BUFFER_SIZE )
        throw std::runtime_error("Buffer Overflow");

    strcpy_custom(forward_buffer+*fb_length_, temp_buffer + pos, length, false);
    *fb_length_ += length;
    
    std::memset(temp_buffer, 0, MAX_BUFFER_SIZE);
    return pos;

}

void Buffer::read_data(char* outer_buff){
    char*   back_buffer = (!active_buffer? buffer1_: buffer0_);
    size_t*  b_length_ = (!active_buffer? &length1_: &length0_);
    
    std::strcpy(outer_buff,back_buffer);  
    std::memset(back_buffer, 0, *b_length_);
    *b_length_ = 0;  
}


//null term tells it to add the null term character
void Buffer::strcpy_custom(char* dest, const char* src, size_t n, bool null_term) {
    for (size_t i = 0; i < n; i++) {
        dest[i] = src[i];
    }
    if (null_term)
        dest[n] = '\0';
    
}

size_t Buffer::strchr_custom(const char* str, char ch, size_t n) {
    for (size_t i = 0; i < n && str[i] != '\0'; i++) {
            if (str[i] == ch) {
                return i;
            }
    }
    return -1;
}