/*
 *  SIM_test.c  --  sim test application
 *
 *  Copyright (c) 2017 Rockchip Electronics Co. Ltd.
 *  Author: linqihao <kevein.lin@rock-chips.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * 	 http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdlib.h>

#include <errno.h>
#include <string.h>

using namespace std;

#define DATA_LEN 256

int uart_receive(int fd,char *rbuf,int rbuf_len,int timeout_ms)
{
    int             rv=-1;
    int             len=-1;
    fd_set          rd_set;
    struct timeval  time_out;

    if(!rbuf||rbuf_len<=0)
    {
        printf("receive func get invalid parameter\n");
        return -1;
    }

    if(timeout_ms)
    {
        time_out.tv_sec=timeout_ms/1000;
        time_out.tv_usec=(timeout_ms%1000)*1000; //微秒

        FD_ZERO(&rd_set);
        FD_SET(fd,&rd_set);

        rv=select(fd+1,&rd_set,NULL,NULL,&time_out);
        if(rv<0)
        {
            printf("select failure:%s\n",strerror(errno));
            return -1;
        }
        else if(rv==0)
        {
            printf("select timeout\n");
            return -1;
        }

    }

    if(FD_ISSET(fd,&rd_set))
    {
        memset(rbuf,0,rbuf_len);
        if((len=read(fd,rbuf,rbuf_len))<0)
        {
            printf("read from serial port failure\n");
            return -1;
        }
        printf("read %d bytes data from serial port fd=%d\n",len,fd);
        return len;
    }

    return 0;
}

int uart_send(int fd,char *send,int data_len)
{
    int len=-1;

    len=write(fd,send,data_len);
    if(len<0)
    {
        printf("write failure:%s\n",strerror(errno));
        return -1;
    }
    printf("write to serial port successfully!\n");

    return 0;
}

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if(tcgetattr(fd, &oldtio)  !=  0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch(nBits)
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    switch(nEvent)
    {
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
    }

    switch(nSpeed)
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }
    if(nStop == 1)
    {
        newtio.c_cflag &=  ~CSTOPB;
    }
    else if(nStop == 2)
    {
        newtio.c_cflag |=  CSTOPB;
    }
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}


int check_uart(char* uart_name)
{
    int tty_fd;
    int i,ret,index;
    char send_buff[DATA_LEN] = {0};
    char receive_buff[DATA_LEN] = {0};
    fd_set rd;

    tty_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if(tty_fd<0)
    {
        printf("open %s failed!\n", uart_name);
        return -1;
    }

    if((ret = set_opt(tty_fd, 115200, 8, 'N', 1)) < 0)
    {
        perror("set lte port set_opt error");
        close(tty_fd);
        return -1;
    }

    //send
    for(i=0;i<DATA_LEN;i++)
        send_buff[i]=i;

    ret = uart_send(tty_fd,send_buff,DATA_LEN);
    if(ret)
    {
        perror("uart_send failed!");
        close(tty_fd);
        return -1;
    }
    
    //receive
    index = 0;
    for(i=0;i<100;i++){
        ret = uart_receive(tty_fd,&receive_buff[index],DATA_LEN-index,10);
        if(ret>0){
            index+=ret;
            if(index>=DATA_LEN)
                break;
        }
    }
    if(index<=0)
    {
        perror("uart_receive failed!");
        close(tty_fd);
        return -1;
    }

    printf("receive: %s\n",receive_buff);

    //check
    /*for(i=0;i<DATA_LEN;i++){
        if(receive_buff[i]!=i){
            FormatPrint("receive error: buff[%x]=%x\n",i,receive_buff[i]);
            close(tty_fd);
            return -1;
        }
            
    }*/


    //
    close(tty_fd);
    return 0;
}

#if 0
int check_sim(char* uart_name)
{
    int tty_fd;
    int i,ret,index;
    char send_buff[DATA_LEN] = {0};
    char receive_buff[DATA_LEN] = {0};
    fd_set rd;

    tty_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if(tty_fd<0)
    {
        FormatPrint("open %s failed!\n", uart_name);
        return -1;
    }

    if((ret = set_opt(tty_fd, 115200, 8, 'N', 1)) < 0)
    {
        perror("set lte port set_opt error");
        close(tty_fd);
        return -1;
    }

    //send
    for(i=0;i<DATA_LEN;i++)
        send_buff[i]=i;
    strcpy(send_buff,"AT\r\n");
    FormatPrint("send len = %d\n", strlen(send_buff));

    ret = uart_send(tty_fd,send_buff,strlen(send_buff));
    if(ret)
    {
        perror("uart_send failed!");
        close(tty_fd);
        return -1;
    }
    
    //receive
    index = 0;
    for(i=0;i<100;i++){
        ret = uart_receive(tty_fd,&receive_buff[index],DATA_LEN-index,10);
        if(ret>0){
            index+=ret;
            if(index>=DATA_LEN)
                break;
        }
    }

    
    if(index<=0)
    {
        perror("uart_receive failed!");
        close(tty_fd);
        return -1;
    }

    FormatPrint("receive: %s\n",receive_buff);


    //
    close(tty_fd);
    return 0;
}
#endif

int main(int argc, char *argv[])
{
    if(argc != 2)
    {
        printf("The input parameter is incorrect! \n");
        return -1;
    }

    int ret = 0;
    ret = check_uart(argv[1]);
    if(ret == 0)
    {
        printf("UART_test=[OK]\n");
    }
    else
    {
        printf("UART_test=[NG]\n");
    }
    
    return ret;
}

/**
 * @brief CRC16校验
 * @details 用于Modbus通信的CRC16校验
 * @param cpucData 数据缓冲区
 * @param usLength 数据长度
 * @return uint16_t 返回值
 * @since V1.0.0
 */
static uint16_t CrcValueCalc(const uint8_t *cpucData, uint16_t usLength)
{
    uint16_t usCrcValue = 0xFFFF;
    int iI = 0;
    while (usLength--)
    {
        usCrcValue ^= *cpucData++;
        for (iI = 8 - 1; iI >= 0; iI--)
        {
            if (usCrcValue & 0x0001)
            {
                usCrcValue = (usCrcValue >> 1) ^ 0xA001;
            }
            else
            {
                usCrcValue = usCrcValue >> 1;
            }
        }
    }
    return (usCrcValue);
}"
