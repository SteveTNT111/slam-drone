/*
============================================
uart_param_config();       函数配置时钟源和uart控制器
uart_set_pin()             配置引脚输入输出        
uart_driver_install()      配置发送/接受缓冲区
uart_write_bytes()         发送缓冲区写入数据
uart_read_bytes()          接受缓冲区中读取数据

GPIO17 Tx  GPIO18 Rx  U1
============================================
*/
#include "driver/gpio.h"
#include "driver/uart.h"

void Uart_Init(void)
{
    QueueHandle_t uart_queue;
    esp_err_t err0;
    uart_config_t uart_config_structure;
    uart_config_structure.baud_rate = 9600;
    uart_config_structure.data_bits = UART_DATA_8_BITS;
    uart_config_structure.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;//硬件流控
    uart_config_structure.parity = UART_PARITY_DISABLE;//无校验位
    uart_config_structure.source_clk = UART_SCLK_DEFAULT;
    uart_config_structure.stop_bits = UART_STOP_BITS_1;
    uart_config_structure.rx_flow_ctrl_thresh = 0;//硬件流控已禁用，设为 0 
    uart_config_structure.flags.allow_pd = 0;//不懂
    err0 = uart_param_config(UART_NUM_1,&uart_config_structure);
    if(err0 != 0)
    {
        printf("Uart_Init uart_param_config error!\r\n");
    }

    uart_set_pin(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, -1, -1, -1, -1);
    esp_err_t err1;
    err1 = uart_driver_install(UART_NUM_1, 1024, 1024, 1, &uart_queue, 0);
    if(err1 != 0)
    {
        printf("Uart_Init uart_driver_install error! %d (%s)\n",err1, esp_err_to_name(err1));
        return;
    }

}
