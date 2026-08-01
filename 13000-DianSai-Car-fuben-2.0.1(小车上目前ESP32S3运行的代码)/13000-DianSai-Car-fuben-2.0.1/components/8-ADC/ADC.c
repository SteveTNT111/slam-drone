/*
=============================================
ESP32S3 ADC 资源 SARADC1 SARADC2 共20路输入通道
12位逐次逼近型ADC
注意：手册中提示 ADC2控制器无法使用 

输入电压 = 数值量/4095 * 3.3
模式控制器 分为单次 多次 转换

ADC转换完成后，形成一个转换帧
一个转换帧内部包含多个转换结果
一个转换结果由4个字节（32bit)组成

模拟GPIO口是固定的
如ADC1CH0 IO1，ADC1CH1 IO2 

adc_continuous_new_handle();················函数配置转换结果存储单元大小，转换帧存储单元大小
adc_digi_pattern_config_t···················结构体配置转换表
adc_continuous_config();····················函数配置ADC总控制器需要下发和执行的参数
adc_continuous_register_event_callbacks()···函数注册回调函数，用于数据读取
adc_continuous_start()······················函数开启连续转换
=============================================
*/
#include "ADC.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_continuous.h"

#include "esp_adc/adc_oneshot.h"//单次转换头文件


//本代码是ADC多次转换 
uint8_t *Data_Value;//存放转换帧地址
uint16_t adc_ch0_datavalue;
uint16_t adc_ch1_datavalue;


bool Adc_Continuous_CallBack_t(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    Data_Value = edata->conv_frame_buffer;
    if(edata->size == 8)
    {
        adc_ch0_datavalue = ((Data_Value[1] & 0x0F) << 8)| Data_Value[0];
        adc_ch1_datavalue = ((Data_Value[5] & 0x0F) << 8)| Data_Value[4];
        return true;
    }
    return false;
}
void ADC_Init_Multiple_Conversions(void)
{
    //本代码是ADC多次转换初始化
    adc_continuous_handle_t ADC_Continuous_Handle;
    esp_err_t err0;
    adc_continuous_handle_cfg_t adc_continuous_handle_cfg_structure;
    adc_continuous_handle_cfg_structure.conv_frame_size = 8;//转换帧的大小
    adc_continuous_handle_cfg_structure.flags.flush_pool = 0;
    adc_continuous_handle_cfg_structure.max_store_buf_size = 1024;
    err0 = adc_continuous_new_handle(&adc_continuous_handle_cfg_structure, &ADC_Continuous_Handle);
    if(err0 != 0)
    {
        printf("ADC_Init adc_continuous_new_handle error!\r\n");
    }

    adc_digi_pattern_config_t adc_digi_pattern_config_structure[2];//转换表
    adc_digi_pattern_config_structure[0].atten = ADC_ATTEN_DB_12;//衰减系数
    adc_digi_pattern_config_structure[0].bit_width = ADC_BITWIDTH_12;//分辨率4095
    adc_digi_pattern_config_structure[0].channel = ADC_CHANNEL_0;//CH0
    adc_digi_pattern_config_structure[0].unit = ADC_UNIT_1; //ADC1
    adc_digi_pattern_config_structure[1].atten = ADC_ATTEN_DB_12;
    adc_digi_pattern_config_structure[1].bit_width = ADC_BITWIDTH_12;
    adc_digi_pattern_config_structure[1].channel = ADC_CHANNEL_1;
    adc_digi_pattern_config_structure[1].unit = ADC_UNIT_1;

    esp_err_t err1;
    adc_continuous_config_t adc_continuous_config_structure;
    adc_continuous_config_structure.adc_pattern = adc_digi_pattern_config_structure;//转换表名称
    adc_continuous_config_structure.conv_mode =ADC_CONV_SINGLE_UNIT_1;
    //adc_continuous_config_structure.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;//Only use ADC1 for conversion 的数据格式是 ADC_DIGI_OUTPUT_FORMAT_TYPE1  绑定的
    adc_continuous_config_structure.pattern_num = 2;//有多小个通道被使用
    adc_continuous_config_structure.sample_freq_hz = 20000;//采样频率2KHz
    err1 = adc_continuous_config(ADC_Continuous_Handle,&adc_continuous_config_structure);
    if(err1 != 0)
    {
        printf("ADC_Init adc_continuous_config error!\r\n");
    }

    esp_err_t err2;
    adc_continuous_evt_cbs_t adc_continuous_evt_cbs_structure;
    adc_continuous_evt_cbs_structure.on_conv_done = Adc_Continuous_CallBack_t;//ADC每次完成一帧DMA数据采集、缓冲区填充完毕 就触发
    //adc_continuous_evt_cbs_structure.on_pool_ovf = ;
    err2 = adc_continuous_register_event_callbacks(ADC_Continuous_Handle , &adc_continuous_evt_cbs_structure, NULL);
    if(err2 != 0)
    {
        printf("ADC_Init adc_continuous_register_event_callbacks error!\r\n");
    }

    esp_err_t err3;
    err3 = adc_continuous_start(ADC_Continuous_Handle);
    if(err3 != 0)
    {
        printf("ADC_Init adc_continuous_start error!\r\n");
    }

}


/*
==================================================
adc_oneshot_new_unit()      函数配置时钟源，ADC参数
adc_oneshot_config_channel()函数配置总线控制器要下发和执行的参数
adc_oneshot_read()           函数开始一次转换并获取转换数据

ADC1CH3 ADC1CH4

==================================================*/
adc_oneshot_unit_handle_t Adc_Oneshot_Unit_Handle;
void ADC_Init_Standalone_Conversion(void)
{
    
    esp_err_t err4;
    adc_oneshot_unit_init_cfg_t adc_oneshot_unit_init_cfg_structure;
    adc_oneshot_unit_init_cfg_structure.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    adc_oneshot_unit_init_cfg_structure.ulp_mode = ADC_ULP_MODE_DISABLE;
    adc_oneshot_unit_init_cfg_structure.unit_id = ADC_UNIT_1;
    err4 = adc_oneshot_new_unit(&adc_oneshot_unit_init_cfg_structure, &Adc_Oneshot_Unit_Handle);
    if(err4 != 0)
    {
        printf("ADC_Init_Standalone_Conversoin adc_oneshot_new_unit error!\r\n");
    }
    esp_err_t err5;
    adc_oneshot_chan_cfg_t adc_oneshot_chan_cfg_structure;
    adc_oneshot_chan_cfg_structure.atten = ADC_ATTEN_DB_12;//衰减系数
    adc_oneshot_chan_cfg_structure.bitwidth = ADC_BITWIDTH_12;
    err5 = adc_oneshot_config_channel(Adc_Oneshot_Unit_Handle, ADC_CHANNEL_3, &adc_oneshot_chan_cfg_structure);
    err5 = adc_oneshot_config_channel(Adc_Oneshot_Unit_Handle, ADC_CHANNEL_4, &adc_oneshot_chan_cfg_structure);
    if(err5 != 0)
    {
        printf("ADC_Init_Standalone_Conversoin adc_oneshot_config_channel error!\r\n");
    }
    
}