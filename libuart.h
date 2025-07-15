#include <string>

#define DATA_LEN 256
#define DATAHEAD 0x40
#define READFLAG 0X03
#define WRITEFLAG 0x06

/**
 * @brief 检查串口
 * @details 用于打开串口，并设置
 * @param uart_name 串口号 /dev/ttySn
 * @return int 打开的串口文件描述符
 */
int check_uart(char* uart_name);

/**
 * @brief 发生数据
 * @details 用于通过串口发送数据
 * @param tty_fd 串口文件描述符
 * @param send_buff 发送数据缓冲区指针
 * @param data_len 发送数据长度，以字节为单位
 * @return int 发送成功返回0，失败返回-1
 */
int send_data(int tty_fd, char* send_buff, int data_len);

/**
 * @brief 接收数据
 * @details 用于通过串口接收数据
 * @param tty_fd 串口文件描述符
 * @param receive_buff 接收数据缓冲区指针
 * @return int 成功返回接收数据长度，失败返回-1
 */
int receive_data(int tty_fd, char* receive_buff);


/**
 * @brief 发生字符串
 * @details 用于通过串口发送字符串
 * @param tty_fd 串口文件描述符
 * @param anyInputs 发送字符串 由指令，二级指令，其他数据(可选) 组成
* @param readflag 读写标志位
 * @return int 发送成功返回0，失败返回-1
 */
 int send_str(int tty_fd, std::string anyInputs);
 int send_str(int tty_fd, std::string anyInputs, int readflag);
 