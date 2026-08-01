#include "WIFI.h"
#include "OLED.h"
#include "esp_wifi.h"
#include <string.h>

#include "esp_sntp.h"//sntp协议
#include "time.h"
/*
===================================================
所有消费级wifi设备都必须遵循802.11标准才能实现互联
wifi AP(接入点模式) STA(站点模式) 混合模式
LWIP协议栈
WiFi 仅支持2.4GHz
联网 通信 


===================================================*/

#define DEFAULT_STA_SSID        "REDMI Turbo 4 Pro"//esp站点要连接的名称
#define DEFAULT_STA_PASSWORD    "123456789990"

#define DEFAULT_AP_SSID        "ESP32_S3_AP"//esp热点名称
#define DEFAULT_AP_PASSWORD    "123456789990"

 void wifista_event_handler(void* event_handler_arg,esp_event_base_t event_base,int32_t event_id,void* event_data)
 {
    if(event_base == WIFI_EVENT)
    {
        if(event_id == WIFI_EVENT_STA_START)
        {
            esp_wifi_connect();//wifi连接
        }
        else if(event_id == WIFI_EVENT_STA_CONNECTED)
        {
            OLED_ShowString(1,1,"wifi_connected   ",BLACK,WHITE);//wifi已连接事件
        }
        else if(event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            OLED_ShowString(1,1,"wifi_disconnected",RED,WHITE);//WiFi断开连接事件
            esp_wifi_stop();//关闭WiFi模块
        }
    }
    else if(event_base == IP_EVENT)
    {
        if(event_id == IP_EVENT_STA_GOT_IP)
        {
            esp_netif_ip_info_t *event = (esp_netif_ip_info_t*)event_data;
            
            OLED_ShowNum(   2,1,esp_ip4_addr1_16(&event->ip),     3,BLACK,WHITE);
            OLED_ShowString(2,4,".",BLACK,WHITE);
            OLED_ShowNum(   2,5,esp_ip4_addr2_16(&event->ip),     3,BLACK,WHITE);
            OLED_ShowString(2,8,".",BLACK,WHITE);
            OLED_ShowNum(   2,9,esp_ip4_addr3_16(&event->ip),     3,BLACK,WHITE);
            OLED_ShowString(2,12,".",BLACK,WHITE);
            OLED_ShowNum(   2,13,esp_ip4_addr4_16(&event->ip),    3,BLACK,WHITE);
            OLED_ShowString(2,18,"IP",BLACK,WHITE); //IPV4

            OLED_ShowNum(   3,1,esp_ip4_addr1_16(&event->netmask),3,BLACK,WHITE);
            OLED_ShowString(3,4,".",BLACK,WHITE);
            OLED_ShowNum(   3,5,esp_ip4_addr2_16(&event->netmask),3,BLACK,WHITE);
            OLED_ShowString(3,8,".",BLACK,WHITE);
            OLED_ShowNum(   3,9,esp_ip4_addr3_16(&event->netmask),3,BLACK,WHITE);
            OLED_ShowString(3,12,".",BLACK,WHITE);
            OLED_ShowNum(   3,13,esp_ip4_addr4_16(&event->netmask),3,BLACK,WHITE);
            OLED_ShowString(3,18,"Netmask",BLACK,WHITE);//子网掩码

            OLED_ShowNum(   4,1,esp_ip4_addr1_16(&event->gw),      3,BLACK,WHITE);
            OLED_ShowString(4,4,".",BLACK,WHITE);
            OLED_ShowNum(   4,5,esp_ip4_addr2_16(&event->gw),      3,BLACK,WHITE);
            OLED_ShowString(4,8,".",BLACK,WHITE);
            OLED_ShowNum(   4,9,esp_ip4_addr3_16(&event->gw),      3,BLACK,WHITE);
            OLED_ShowString(4,12,".",BLACK,WHITE);
            OLED_ShowNum(   4,13,esp_ip4_addr4_16(&event->gw),     3,BLACK,WHITE);
            OLED_ShowString(4,18,"Gateway",BLACK,WHITE);//网关地址

        }
    }
 }

void WIFI_Init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,&wifista_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,&wifista_event_handler, NULL);
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_config_structure = WIFI_INIT_CONFIG_DEFAULT();//WiFi初始化设置默认
    esp_wifi_init(&wifi_init_config_structure);

    esp_wifi_set_mode(WIFI_MODE_STA);

    wifi_config_t wifi_config_structure = {0};
    strcpy((char*)wifi_config_structure.sta.ssid, DEFAULT_STA_SSID);
    strcpy((char*)wifi_config_structure.sta.password, DEFAULT_STA_PASSWORD);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config_structure);

    esp_wifi_start();

}

 void wifiap_event_handler(void* event_handler_arg,esp_event_base_t event_base,int32_t event_id,void* event_data)
 {
    if(event_base == WIFI_EVENT)
    {
        if(event_id == WIFI_EVENT_AP_STACONNECTED)
        {
            OLED_ShowString(4,8,"Connectes  ",BLACK,WHITE);
        }
        else if(event_id == WIFI_EVENT_AP_STADISCONNECTED)
        {
            OLED_ShowString(4,8,"Disonnectes",BLACK,WHITE);
        }
    }
 }

void WIFI_Init_ap(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,&wifiap_event_handler, NULL);
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_init_config_structure = WIFI_INIT_CONFIG_DEFAULT();//WiFi初始化设置默认
    esp_wifi_init(&wifi_init_config_structure);

    esp_wifi_set_mode(WIFI_MODE_AP);

    wifi_config_t wifi_config_structure = {0};
    strcpy((char*)wifi_config_structure.ap.ssid, DEFAULT_AP_SSID);
    strcpy((char*)wifi_config_structure.ap.password, DEFAULT_AP_PASSWORD);
    wifi_config_structure.ap.ssid_len = strlen(DEFAULT_AP_SSID);            //账户名长度
    wifi_config_structure.ap.max_connection = 2;                            //最大连接数
    wifi_config_structure.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;             //加密方式
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config_structure);

    esp_wifi_start();

    OLED_ShowString(1,1,"SSID:",BLACK,WHITE);
    OLED_ShowString(1,6,DEFAULT_AP_SSID,BLACK,WHITE);
    OLED_ShowString(2,1,"PASSWORD:",BLACK,WHITE);
    OLED_ShowString(3,1,DEFAULT_AP_PASSWORD,BLACK,WHITE);

    OLED_ShowString(4,1,"Stable:",BLACK,WHITE);

}


 void wifi_GetTime_event_handler(void* event_handler_arg,esp_event_base_t event_base,int32_t event_id,void* event_data)
 {
    if(event_base == WIFI_EVENT)
    {
        if(event_id == WIFI_EVENT_STA_START)
        {
            esp_wifi_connect();//wifi连接
        }
        else if(event_id == WIFI_EVENT_STA_CONNECTED)
        {
            OLED_ShowString(1,1,"wifi_connected   ",BLACK,WHITE);//wifi已连接事件
        }
        else if(event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            OLED_ShowString(1,1,"wifi_disconnected",RED,WHITE);//WiFi断开连接事件
            esp_wifi_connect();
            //esp_wifi_stop();//关闭WiFi模块
        }
    }
    
 }

void WIFI_Init_Get_Time(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,&wifi_GetTime_event_handler, NULL);
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_config_structure = WIFI_INIT_CONFIG_DEFAULT();//WiFi初始化设置默认
    esp_wifi_init(&wifi_init_config_structure);

    esp_wifi_set_mode(WIFI_MODE_STA);

    wifi_config_t wifi_config_structure = {0};
    strcpy((char*)wifi_config_structure.sta.ssid, DEFAULT_STA_SSID);
    strcpy((char*)wifi_config_structure.sta.password, DEFAULT_STA_PASSWORD);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config_structure);

    esp_wifi_start();

    //时间同步设置
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);//sntp 请求模式

    esp_sntp_setservername(0, "ntp1.aliyun.com");//设置要访问的NPT服务器
    esp_sntp_setservername(1, "cn.pool.ntp.org");//设置要访问的NPT服务器
    esp_sntp_setservername(2, "pool.npt.org");//设置要访问的NPT服务器

    esp_sntp_init();//sntp初始化

    setenv ("TZ", "CST-8",1);//TZ Time Zone 缩写 表示要设置的时区参数  CST-8 我们要设置的时区 东八区

    tzset();//写设置的时区立即生效
}