//
// Created by sub on 18/12/17.
//

#ifndef RIC_INTERFACE_SERIALCOM_H
#define RIC_INTERFACE_SERIALCOM_H

#include "ric_exception.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

typedef unsigned char byte;

#include <string>


class SerialCom
{

private:
    int file_handle_,
            baudrate_;
    void setAttributes();


public:
    ~SerialCom(){ ::close(file_handle_); }
    void connect(std::string port, int baudrate);
    bool send(const byte buff[], size_t size);
    int read(byte buff[], size_t size);
    int read();

};



#endif //RIC_INTERFACE_SERIALCOM_H
