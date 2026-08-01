/*
======================================
ESP32S3 有2个IIC控制器  IIC0 IIC1
有主机模式 和 从机模式
低速100KHz 高速400KHz 
Init 旧版
// i2c_param_config();         函数配置时钟源，GPIO交换矩引脚。IIC参数
// i2c_driver_install();       向CPU注册IIC

// i2c_cmd_link_create();      函数创建命令寄存器中的命令链（存放在数组中）
// i2c_master_start();         向命令链中存入IIC_START命令
// i2c_master_write_byte()     向命令链中存入IIC_WRITE命令
// i2c_master_read_byte()      向命令链中存入IIC_READ命令
// i2c_master_stop()           向命令链中存入IIC_STOP命令
// i2c_master_cmd_begin()      函数开始命令控制器，开始IIC通信
// i2c_cmd_link_delete()       函数删除命令链，释放内存
新版
i2c_new_master_bus
i2c_master_bus_add_device

i2c_master_transmit
======================================*/
#include "IIC.h"
#include "driver/i2c_master.h"
#include "driver/dedic_gpio.h"
#include "driver/gpio.h"


//主机 写   
    i2c_master_bus_handle_t i2c_master_bus_handle_W;
    i2c_master_dev_handle_t i2c_master_dev_handle_W;
void IIC_Init_W(void)
{
    

    esp_err_t err0;
	i2c_master_bus_config_t i2c_master_bus_config_structure;
	i2c_master_bus_config_structure.clk_source = I2C_CLK_SRC_DEFAULT;
	i2c_master_bus_config_structure.flags.enable_internal_pullup = true;
	i2c_master_bus_config_structure.glitch_ignore_cnt = 7;
	i2c_master_bus_config_structure.i2c_port = I2C_NUM_0;
	i2c_master_bus_config_structure.intr_priority = 1;//中断优先级
	i2c_master_bus_config_structure.scl_io_num = GPIO_NUM_4;
	i2c_master_bus_config_structure.sda_io_num = GPIO_NUM_5;
	//i2c_master_bus_config_structure.trans_queue_depth =20;
	err0 = i2c_new_master_bus(&i2c_master_bus_config_structure, &i2c_master_bus_handle_W);
	if(err0 != 0)
	{
		printf("OLED_Send_Cmd i2c_new_master_bus error!\r\n");
	}
	esp_err_t err1;
	i2c_device_config_t i2c_device_config_structure;
	i2c_device_config_structure.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	i2c_device_config_structure.device_address = 0x3C;
	i2c_device_config_structure.flags.disable_ack_check = false;
	i2c_device_config_structure.scl_speed_hz = 400000;
	i2c_device_config_structure.scl_wait_us = 1000;
	
 	err1 = i2c_master_bus_add_device(i2c_master_bus_handle_W, &i2c_device_config_structure, &i2c_master_dev_handle_W);
	if(err1 != 0)
	{
		printf("OLED_IIC_Init i2c_master_bus_add_device error1\r\n");
	}
}
//=======================没写完=========================================
//     i2c_master_bus_handle_t i2c_master_bus_handle_R;
//     i2c_master_dev_handle_t i2c_master_dev_handle_R;
// void IIC_Init_R(void)
// {
//     esp_err_t err0;
// 	i2c_master_bus_config_t i2c_master_bus_config_structure;
// 	i2c_master_bus_config_structure.clk_source = I2C_CLK_SRC_DEFAULT;
// 	i2c_master_bus_config_structure.flags.enable_internal_pullup = true;
// 	i2c_master_bus_config_structure.glitch_ignore_cnt = 7;
// 	i2c_master_bus_config_structure.i2c_port = I2C_NUM_0;
// 	i2c_master_bus_config_structure.intr_priority = 1;//中断优先级
// 	i2c_master_bus_config_structure.scl_io_num = GPIO_NUM_4;
// 	i2c_master_bus_config_structure.sda_io_num = GPIO_NUM_5;
// 	//i2c_master_bus_config_structure.trans_queue_depth =20;
// 	err0 = i2c_new_master_bus(&i2c_master_bus_config_structure, &i2c_master_bus_handle_R);
// 	if(err0 != 0)
// 	{
// 		printf("OLED_Send_Cmd i2c_new_master_bus error!\r\n");
// 	}
// 	esp_err_t err1;
// 	i2c_device_config_t i2c_device_config_structure;
// 	i2c_device_config_structure.dev_addr_length = I2C_ADDR_BIT_LEN_7;
// 	i2c_device_config_structure.device_address = 0x3C;
// 	i2c_device_config_structure.flags.disable_ack_check = false;
// 	i2c_device_config_structure.scl_speed_hz = 400000;
// 	i2c_device_config_structure.scl_wait_us = 1000;
	
//  	err1 = i2c_master_bus_add_device(i2c_master_bus_handle_R, &i2c_device_config_structure, &i2c_master_dev_handle_R);
// 	if(err1 != 0)
// 	{
// 		printf("OLED_IIC_Init i2c_master_bus_add_device error1\r\n");
// 	}
//     esp_err_t err2;
//     uint8_t read_buffer
//     err2 = i2c_master_receive(i2c_master_dev_handle_R, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms)//

// }

// void IIC_Init_W_R(void)
// {
//     i2c_device_config_t dev_cfg = {
//     .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//     .device_address = 0x58,
//     .scl_speed_hz = 100000,
// };

// i2c_master_dev_handle_t dev_handle;
// ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
// uint8_t buf[20] = {0x20};
// uint8_t buffer[2];
// ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, sizeof(buf), buffer, 2, -1));
// }
