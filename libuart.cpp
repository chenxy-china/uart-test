#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include "libuart.h"
#include "utils.h"

using namespace std;

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

    return 0;
}

int uart_send(int fd,char *send,int data_len)
{
    int len=-1;

    len=write(fd,send,data_len);
#ifdef DEBUG
    for(int i = 0; i < data_len; i++) {
        printf("%02x", (unsigned int)*(send+i));
    }
    printf("\n");
#endif
    if(len<0)
    {
        printf("write failure:%s\n",strerror(errno));
        return -1;
    }
    printf("write to serial port successfully!\n");

    return 0;
}

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
#ifdef DEBUG
            printf("select timeout\n");
#endif
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

int check_uart(char* uart_name)
{
    int tty_fd;

    tty_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if(tty_fd<0)
    {
        printf("open %s failed!\n", uart_name);
        return -1;
    }

    //串口波特率：115200，8N1
    if((set_opt(tty_fd, 115200, 8, 'N', 1)) < 0)
    {
        perror("set lte port set_opt error");
        close(tty_fd);
        return -1;
    }

    printf("open %s and set opt [115200,8,N,1] done!\n",uart_name);

    return tty_fd;
}

int send_data(int tty_fd, char* send_buff, int data_len)
{
    int i,ret,index;

    //send
    ret = uart_send(tty_fd,send_buff,data_len);
    if(ret)
    {
        perror("uart_send failed!");
        return -1;
    }

    return 0;
}

int receive_data(int tty_fd, char* receive_buff)
{
    //receive
    int i, ret = 0, index = 0;
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

#ifdef DEBUG
    printf("receive_buff : %d , ", index);  
    for(i = 0 ;i < index; i++){
        printf("%02x",*(receive_buff+i));
    }
    printf("\n");
#endif

    return index;
}

int send_str(int tty_fd, std::string anyInputs)
{
    int len = anyInputs.length();
    char *dataInputs , *data, *dataSends;
    dataInputs = (char *)malloc((len+1)*sizeof(char));
    strcpy(dataInputs,anyInputs.c_str());
    data = (char *)malloc((len+1)*sizeof(char));

    int i = 0, j = 0;
    j = str2dex(dataInputs,data,len);

    dataSends = (char *)malloc((j+5)*sizeof(char));
    memset(dataSends,0,(j+5));

    memset(dataSends,DATAHEAD,1);
    int readflag = 0;
    if(j > 2){
        memset(dataSends+1,WRITEFLAG,1);
    } else{
        memset(dataSends+1,READFLAG,1);
        readflag = 1;
    }
    memset(dataSends+2,j+3,1);
    memcpy(dataSends+3,data,j);

    uint16_t crcdata =  CrcValueCalc((uint8_t *)dataSends,j+3);
    memcpy(dataSends+j+3,(unsigned char *)&crcdata,2);

    send_data(tty_fd, dataSends,(j+5));

    free(dataInputs);
    free(data);
    free(dataSends);

    return readflag;
}

int send_str(int tty_fd, std::string anyInputs, int readflag)
{
    int len = anyInputs.length();
    char *dataInputs , *data, *dataSends;
    dataInputs = (char *)malloc((len+1)*sizeof(char));
    strcpy(dataInputs,anyInputs.c_str());
    data = (char *)malloc((len+1)*sizeof(char));

    int i = 0, j = 0;
    j = str2dex(dataInputs,data,len);

    dataSends = (char *)malloc((j+5)*sizeof(char));
    memset(dataSends,0,(j+5));

    memset(dataSends,DATAHEAD,1);
    if(readflag == 0){
        memset(dataSends+1,WRITEFLAG,1);
    } else{
        memset(dataSends+1,READFLAG,1);
    }
    memset(dataSends+2,j+3,1);
    memcpy(dataSends+3,data,j);

    uint16_t crcdata =  CrcValueCalc((uint8_t *)data,j+3);
    // memcpy(dataSends+j+3,(unsigned char *)&crcdata,2);
    char crclowbyte = crcdata & 0xFF;
    char crchighbyte = (crcdata & 0xFF00) >> 8 ;
    memcpy(dataSends+j+3,&crclowbyte ,1);
    memcpy(dataSends+j+4,&crchighbyte ,1);

    send_data(tty_fd, dataSends,(j+5));

    free(dataInputs);
    free(data);
    free(dataSends);

    return 0;
}
