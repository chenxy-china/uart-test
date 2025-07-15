#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

int str2dex(char *datain, char *dataout, int len)
{
    int i=0,j=0;
    char str[2] = {0,};
    char *endptr;
    int num;

    for(i = 0; i < len; i=i+2) {
        strncpy(str,datain+i,2);
        num = strtol(str, &endptr, 16); // 16 表示十进制

        if (*endptr == '\0') { // 确保转换结束于字符串末尾
#ifdef DEBUG
            printf("转换后的整数: 0x%02x\n", num);
#endif
            memcpy(dataout+j,&num,1);
            j++;
        } else {
            printf("转换失败，未转换的部分: %s\n", endptr);
            return -1;
        }
    }

#ifdef DEBUG
    printf("转换后:");  
    for(i = 0; i < j; i++) {
        printf("%02x", (unsigned int)*(dataout+i));
    }
    printf("\n");
#endif

    return j;
}


/**
 * @brief CRC16校验
 * @details 用于Modbus通信的CRC16校验
 * @param cpucData 数据缓冲区
 * @param usLength 数据长度
 * @return uint16_t 返回值
 * @since V1.0.0
 */
 uint16_t CrcValueCalc(const uint8_t *cpucData, uint16_t usLength)
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
 }