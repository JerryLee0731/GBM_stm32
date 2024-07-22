#include "mbed.h"
#include "uart_pc.h"

void uart_pc(BufferedSerial& serial_port, char buffer[32]) {
    if (serial_port.readable()) {
        int index = 0;
        char c;
        bool start_detected = false;

        // 读取完整的数据包，假设以'\n'为结束符
        while (1) {
            serial_port.read(&c, 1);

            if (c == '$') {
                // 检测到开头符号
                start_detected = true;
                index = 0;  // 重置缓冲区索引
                continue;
            }
            
            if (start_detected) {
                if (c == '\n') {
                    buffer[index] = '\0';  // 确保字符串以空字符结束
                    break;
                } else {
                    buffer[index++] = c;
                }
            }
        }
        printf("recieved from serial port\n");

        // 使用方式: 解析数据包
        // sscanf(buffer, "%lf %lf %lf %lf", &thetaf, &thetar, &omegaf, &omegar);
    }
}