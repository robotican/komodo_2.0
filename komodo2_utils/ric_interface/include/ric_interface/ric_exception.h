//
// Created by Eli Eli on 18/11/2017.
//

#ifndef RIC_INTERFACE_RIC_EXCEPTION_H
#define RIC_INTERFACE_RIC_EXCEPTION_H

#include <iostream>
//#include <exception>
//#include <stdexcept>
//#include <sstream>

namespace ric_interface
{
    struct RicException : public std::runtime_error
    {
    public:
        RicException(std::string msg) : std::runtime_error(msg) {}
    };

    struct ConnectionExeption : public RicException
    {
    public:
        ConnectionExeption(std::string msg) : RicException(msg) {}
    };
}


#endif //RIC_INTERFACE_RIC_EXCEPTION_H
