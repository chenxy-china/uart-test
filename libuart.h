#include <string>

#define DATA_LEN 256
#define DATAHEAD 0x40
#define READFLAG 0X03
#define WRITEFLAG 0x06

int check_uart(char* uart_name);
int send_data(int tty_fd, char* send_buff, int data_len);
int receive_data(int tty_fd, char* receive_buff);
int send_str(int tty_fd, std::string anyInputs);