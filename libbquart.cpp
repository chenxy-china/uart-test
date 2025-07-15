#include <unistd.h>
#include <iostream>
#include "libuart.h"

std::string statusReadMoto(int tty_fd)
{
    std::string anyInputs="1101";
    send_str(tty_fd, anyInputs, 1);

    char receive_buff[DATA_LEN] = {0};
    receive_data(tty_fd,receive_buff);
    std::string ret = receive_buff;
    return ret;
}

std::string statusReadSys(int tty_fd)
{
    std::string anyInputs="1102";
    send_str(tty_fd, anyInputs, 1);

    char receive_buff[DATA_LEN] = {0};
    receive_data(tty_fd,receive_buff);
    std::string ret = receive_buff;
    return ret;
}

std::string fixAngleRead(int tty_fd)
{
    std::string anyInputs="1201";
    send_str(tty_fd, anyInputs, 1);

    char receive_buff[DATA_LEN] = {0};
    receive_data(tty_fd,receive_buff);
    std::string ret = receive_buff;
    return ret;
}

void fixAngleSet(int tty_fd,std::string angle)
{
    std::string anyInputs="1201" + angle;
    send_str(tty_fd, anyInputs, 0);
}

std::string imuDataRead(int tty_fd)
{
    std::string anyInputs="1202";
    send_str(tty_fd, anyInputs, 1);

    char receive_buff[DATA_LEN] = {0};
    receive_data(tty_fd,receive_buff);
    std::string ret = receive_buff;
    return ret;
}

std::string accDataRead(int tty_fd)
{
    std::string anyInputs="1203";
    send_str(tty_fd, anyInputs, 1);

    char receive_buff[DATA_LEN] = {0};
    receive_data(tty_fd,receive_buff);
    std::string ret = receive_buff;
    return ret;
}

std::string accRangeRead(int tty_fd)
{
    std::string anyInputs="1204";
    send_str(tty_fd, anyInputs, 1);

    char receive_buff[DATA_LEN] = {0};
    receive_data(tty_fd,receive_buff);
    std::string ret = receive_buff;
    return ret;
}

void accRangeSet(int tty_fd,std::string range)
{
    std::string anyInputs="1204" + range;
    send_str(tty_fd, anyInputs, 0);
}

std::string imuPrecisionaRead(int tty_fd)
{
    std::string anyInputs="1205";
    send_str(tty_fd, anyInputs, 1);

    char receive_buff[DATA_LEN] = {0};
    receive_data(tty_fd,receive_buff);
    std::string ret = receive_buff;
    return ret;
}

void imuPrecisionaSet(int tty_fd, std::string precision)
{
    std::string anyInputs="1205" + precision;
    send_str(tty_fd, anyInputs, 0);
}

void errorClearMoto(int tty_fd)
{
    std::string anyInputs="1301";
    send_str(tty_fd, anyInputs, 0);
}

void errorClearSys(int tty_fd)
{
    std::string anyInputs="1302";
    send_str(tty_fd, anyInputs, 0);
}

std::string brightnessReadData0(int tty_fd)
{
    std::string anyInputs="1401";
    send_str(tty_fd, anyInputs, 1);

    char receive_buff[DATA_LEN] = {0};
    receive_data(tty_fd,receive_buff);
    std::string ret = receive_buff;
    return ret;
}

std::string brightnessReadData2(int tty_fd)
{
    std::string anyInputs="1402";
    send_str(tty_fd, anyInputs, 1);

    char receive_buff[DATA_LEN] = {0};
    receive_data(tty_fd,receive_buff);
    std::string ret = receive_buff;
    return ret;
}