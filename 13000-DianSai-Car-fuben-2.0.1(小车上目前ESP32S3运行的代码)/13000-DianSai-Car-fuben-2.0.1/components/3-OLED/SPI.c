#include "SPI.h"
#include "OLED.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

spi_device_handle_t Spi2_Handle;

/* SPI 访问互斥锁 —— 防止多任务并发操作同一 SPI 设备导致崩溃 */
static SemaphoreHandle_t spi_mutex = NULL;

void Spi2_Init(void)
{
    esp_err_t err0;
    spi_bus_config_t spi_bus_config_structure = 
    {
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = 0,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .max_transfer_sz = 340*240*2,
        .miso_io_num = -1,
        .mosi_io_num = SDA_GPIO,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .sclk_io_num = SCL_GPIO,
    };
    err0 = spi_bus_initialize(SPI2_HOST, &spi_bus_config_structure,SPI_DMA_CH_AUTO);
    if(err0 != 0)
    {
        printf("Spi2_Init_spi_bus_config error!\r\n");
    }

    esp_err_t err1;
    spi_device_interface_config_t spi_device_interface_config_structure = 
    {
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .clock_speed_hz = 60000000,
        .mode = 0,
        .queue_size = 7,
        .spics_io_num = CS_GPIO,
    };
    err1 = spi_bus_add_device(SPI2_HOST, &spi_device_interface_config_structure, &Spi2_Handle);
    if(err1 != 0)
    {
        printf("Spi2_Init_spi_bus_add_device error!\r\n");
    }

    /* 创建互斥锁（带优先级继承，防止优先级反转） */
    spi_mutex = xSemaphoreCreateMutex();
    if(spi_mutex == NULL)
    {
        printf("Spi2_Init: xSemaphoreCreateMutex failed!\r\n");
    }
}

uint8_t Spi2_Transfer_Byte(uint8_t Data)
{
    if(spi_mutex == NULL) return 0;

    xSemaphoreTake(spi_mutex, portMAX_DELAY);

    esp_err_t err2;
    spi_transaction_t spi_transaction_structure = {0};
    spi_transaction_structure.length = 8;
    spi_transaction_structure.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    spi_transaction_structure.tx_data[0] = Data;
    err2 = spi_device_polling_transmit(Spi2_Handle, &spi_transaction_structure);
    if(err2 != ESP_OK)
    {
        printf("Spi2_Transfer_Byte error! err=0x%x\r\n", err2);
    }

    xSemaphoreGive(spi_mutex);
    return spi_transaction_structure.rx_data[0];
}

void Spi2_Write_Data(uint8_t *Data, uint32_t Length)
{
    if(spi_mutex == NULL) return;

    xSemaphoreTake(spi_mutex, portMAX_DELAY);

    spi_transaction_t spi_transaction_structure = {0};
    spi_transaction_structure.length = Length * 8;
    spi_transaction_structure.tx_buffer = Data;
    spi_device_polling_transmit(Spi2_Handle, &spi_transaction_structure);

    xSemaphoreGive(spi_mutex);
}
