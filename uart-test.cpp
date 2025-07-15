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
// #include <stdio.h>
// #include <termios.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <sys/ioctl.h>
#include <unistd.h>
// #include <stdlib.h>

// #include <errno.h>
#include <string.h>
// #include <string>
#include <iostream>
#include "libuart.h"

// using namespace std;

int main(int argc, char *argv[])
{
    if(argc != 2)
    {
        printf("The input parameter is incorrect! \n");
        return -1;
    }

    int tty_fd = 0;
    tty_fd = check_uart(argv[1]);
    if(tty_fd < 0)
    {
        printf("check_uart NG\n");
        // return -1;
    }

    while(1){
        std::string anyInputs;
        std::cout << "输入用户指令：";
        std::cin  >> anyInputs;

        if(anyInputs.compare("quit") == 0 || anyInputs.compare("QUIT") == 0 
        || anyInputs.compare("exit") == 0 || anyInputs.compare("EXIT") == 0)
            break;

        try {
            int num = std::stoi(anyInputs);
        } catch (std::invalid_argument const& e) {
            break;
        } catch (std::out_of_range const& e) {
            break;
        }

        int readflag = send_str(tty_fd, anyInputs);

        if(readflag != 0){
            char receive_buff[DATA_LEN] = {0};
            receive_data(tty_fd,receive_buff);
        }
    }

    close(tty_fd);
    
    return 0;
}
