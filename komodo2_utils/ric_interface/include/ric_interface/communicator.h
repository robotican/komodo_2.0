//
// Created by sub on 18/12/17.
//

#ifndef RIC_INTERFACE_COMMUNICATOR_H
#define RIC_INTERFACE_COMMUNICATOR_H

#include <ric_interface/protocol.h>
#include <ric_interface/serial_com.h>
#include <string.h>
#include <iostream>
#include "crc8.h"



namespace ric_interface
{
    typedef uint8_t byte;
    class Communicator
    {

    public:

        enum State
        {
            HEADER_PART_A,
            HEADER_PART_B,
            PACKAGE,
            CHECKSUM
        };

        void connect(std::string port, int baudrate)
        {
            serial_.connect(port, baudrate);
        }

        /* return -1 for reading in process, or type of the incoming pkg */
        int read(byte buff[])
        {
            switch (state_)
            {
                case HEADER_PART_A: //read header
                {
                    if (tryReadHeader())
                        state_ = HEADER_PART_B;
                    break;
                }
                case HEADER_PART_B: //read pkg size
                {
                    pkg_size_ = tryReadPkgSize();
                    if (pkg_size_ != -1)
                        state_ = PACKAGE;
                    break;
                }
                case PACKAGE:
                {
                    int incoming = serial_.read();
                    if (incoming != -1)
                        buff[pkg_indx_++] = (byte)incoming;

                    if (pkg_indx_ >= pkg_size_) //done reading pkg content
                        state_ = CHECKSUM;
                    break;
                }
                case CHECKSUM:
                {

                    int incoming = serial_.read();
                    if (incoming != -1)
                    {
                        byte incoming_checksum = (byte)incoming;
                        byte computed_checksum = crc_.get_crc(buff, pkg_size_);

                        //fprintf(stderr, "got chksum: %d, comp chksum: %d\n", incoming_checksum, computed_checksum);

                        if (incoming_checksum == computed_checksum)
                        {
                            protocol::package pkg;
                            fromBytes(buff, sizeof(protocol::package), pkg);
                            reset();
                            return (uint8_t)pkg.type;
                        }
                        else
                            return -2; //wrong checksum
                    }
                    break;
                }
            }
            return -1;
        }

        static void fromBytes(byte buff[], size_t pkg_size, protocol::package &pkg)
        {
            memcpy(&pkg, buff, pkg_size);
        }

        static void toBytes(const protocol::package &pkg, size_t pkg_size, byte buff[])
        {
            memcpy(buff, &pkg, pkg_size);
        }

        bool write(const protocol::package &pkg, size_t pkg_size)
        {
            /* send pkg header */
            byte header_buff[2];
            header_buff[protocol::HEADER_INDX] = protocol::HEADER_CODE;
            header_buff[protocol::PKG_SIZE_INDX] = pkg_size;
            if (!serial_.send(header_buff, 2))
                return false;
            byte pkg_buff[pkg_size];
            toBytes(pkg, pkg_size, pkg_buff);
            byte checksum[1];
            checksum[0] = crc_.get_crc(pkg_buff, pkg_size);
            /* send pkg content and the checksum */
            if (!serial_.send(pkg_buff, pkg_size))
                return false;
            if (!serial_.send(checksum, 1))
                return false;
            return true;
        }


    private:
        SerialCom serial_;
        State state_ = HEADER_PART_A;
        int pkg_indx_ = 0;
        int pkg_size_ = 0;
        Crc8 crc_;

        void reset()
        {
            state_ = HEADER_PART_A;
            pkg_indx_ = 0;
            pkg_size_ = 0;
        }

        /* try to read valid header start */
        bool tryReadHeader()
        {
            if (serial_.read() == protocol::HEADER_CODE)
                return true;
            return false;
        }

        /* return -1 for failure and pkg size as success */
        int tryReadPkgSize()
        {
            return serial_.read();
        }
    };
}

#endif //RIC_INTERFACE_COMMUNICATOR_H
