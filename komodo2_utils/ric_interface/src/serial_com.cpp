
#include <ric_interface/serial_com.h>

void SerialCom::connect(std::string port, int baudrate) {
    baudrate_ = baudrate;
    file_handle_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (file_handle_ < 0)
        throw ric_interface::ConnectionExeption("[ric_interface]: Failed to open RIC board port");

    /* sleep for 10ms and then clear serial buffer */
    usleep(10000);
    tcflush(file_handle_,TCIOFLUSH);

    /* Check if the file descriptor is pointing to a TTY device or not */
    if (!isatty(file_handle_))
        throw ric_interface::ConnectionExeption("[ric_interface]: RIC Board port is not a tty device");

    setAttributes();
}

void SerialCom::setAttributes() {
    struct termios tty;

    if (tcgetattr(file_handle_, &tty) < 0)
        throw ric_interface::ConnectionExeption("[ric_interface]: Failed to get RIC board port attributes");

    cfsetospeed(&tty, (speed_t) baudrate_);
    cfsetispeed(&tty, (speed_t) baudrate_);
    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    if (tcsetattr(file_handle_, TCSANOW, &tty) != 0)
        throw ric_interface::ConnectionExeption("[ric_interface]: Failed to set RIC board port attributes");

    tcflush(file_handle_, TCIOFLUSH);
}

int SerialCom::read(byte buff[], size_t size) {

    const int max_bad_reads = 200;
    int bad_reads = 0;
    int indx;
    for (indx=0; indx<size && bad_reads<max_bad_reads;)
    {
        byte incoming_byte;
        int i = ::read(file_handle_, &incoming_byte, 1);
        if (i == 1)
        {
            //printf("new: %i\n", incoming_byte);
            buff[indx++] = incoming_byte;
        }
        else
        {
            bad_reads++;
        }
    }
    if (bad_reads == max_bad_reads)
        return -1;
    return indx;
}

int SerialCom::read()
{
    byte incoming_byte;
    int i = ::read(file_handle_, &incoming_byte, 1);
    if (i == 1)
        return incoming_byte;
    return -1;
}

bool SerialCom::send(const byte buff[], size_t size) {
    char i = buff[0];
    int write_len = ::write(file_handle_, buff, size);
    if (write_len != size)
        return false;
    return true;
}
