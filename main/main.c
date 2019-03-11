/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include <string.h>
#include "driver/spi_master.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/event_groups.h"
#include "mdns.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/netdb.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "http_server.h"
#include "wifi_manager.h"

#define SDA_PIN GPIO_NUM_18
#define SCL_PIN GPIO_NUM_23

#define MS5803 0x76

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

#define CMD_RESET 0x1E    // ADC reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV 0x40 // ADC conversion command
#define CMD_ADC_D1 0x00   // ADC D1 conversion
#define CMD_ADC_D2 0x10   // ADC D2 conversion
#define CMD_ADC_256 0x00  // ADC OSR=256
#define CMD_ADC_512 0x02  // ADC OSR=512
#define CMD_ADC_1024 0x04 // ADC OSR=1024
#define CMD_ADC_2048 0x06 // ADC OSR=2048
#define CMD_ADC_4096 0x08 // ADC OSR=4096
#define CMD_PROM_RD 0xA0  // Prom read command


/**
 * @brief RTOS task that periodically prints the heap memory available.
 * @note Pure debug information, should not be ever started on production code!
 */
void monitoring_task(void *pvParameter)
{
	for(;;){
		printf("free heap: %d\n",esp_get_free_heap_size());
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}

void i2c_master_init()
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000};
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

uint16_t cmd_prom(uint8_t coef_num)
{
    i2c_cmd_handle_t cmd;
    uint8_t byte1;
    uint8_t byte2;
    uint16_t coef;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                              // I2C Start.
    i2c_master_write_byte(cmd, (MS5803 << 1) | I2C_MASTER_WRITE, true); // Select MS5803
    i2c_master_write_byte(cmd, CMD_PROM_RD + coef_num * 2, true);       // Cmd byte
    //i2c_master_stop(cmd); // I2C Stop.

    i2c_master_start(cmd);                                             // I2C Start.
    i2c_master_write_byte(cmd, (MS5803 << 1) | I2C_MASTER_READ, true); // Select MS5803
    i2c_master_read_byte(cmd, &byte1, I2C_MASTER_ACK);                 // Read 8 bit.
    i2c_master_read_byte(cmd, &byte2, I2C_MASTER_NACK);                // Read 8 bit.
    i2c_master_stop(cmd);                                              // I2C Stop.

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    coef = (byte1 << 8) | (byte2);
    return coef;
}

uint32_t cmd_adc(uint8_t cmd_conv)
{
    i2c_cmd_handle_t cmd;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint32_t value;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                              // I2C Start.
    i2c_master_write_byte(cmd, (MS5803 << 1) | I2C_MASTER_WRITE, true); // Select MS5803
    i2c_master_write_byte(cmd, CMD_ADC_CONV + cmd_conv, true);          // Cmd byte

    i2c_master_stop(cmd); // I2C Stop.
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);                                              // I2C Start.
    i2c_master_write_byte(cmd, (MS5803 << 1) | I2C_MASTER_WRITE, true); // Select MS5803
    i2c_master_write_byte(cmd, CMD_ADC_READ, true);                     // Cmd byte

    i2c_master_start(cmd);                                             // I2C Start.
    i2c_master_write_byte(cmd, (MS5803 << 1) | I2C_MASTER_READ, true); // Select MS5803
    i2c_master_read_byte(cmd, &byte1, I2C_MASTER_ACK);                 // Read 8 bit.
    i2c_master_read_byte(cmd, &byte2, I2C_MASTER_ACK);                 // Read 8 bit.
    i2c_master_read_byte(cmd, &byte3, I2C_MASTER_NACK);                // Read 8 bit.
    i2c_master_stop(cmd);                                              // I2C Stop.

    vTaskDelay(100 / portTICK_PERIOD_MS);

    i2c_master_cmd_begin(I2C_NUM_0, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    value = (byte1 << 16) | (byte2 << 8) | (byte3);
    return value;
}

void sensorTask(void *parameter) {
    uint16_t C[8]; // calibration coefficients
    uint32_t D1;   // ADC value of the pressure conversion
    uint32_t D2;   // ADC value of the temperature conversion
    double P;      // compensated pressure value
    double T;      // compensated temperature value
    double T2;     // compensated temperature value
    double dT;     // difference between actual and measured temperature
    double OFF;    // offset at actual temperature
    double SENS;   // sensitivity at actual temperature

    for (size_t i = 0; i < 8; i++)
    {
        C[i] = cmd_prom(i);
        printf("%x\n", C[i]);
    }

    while (true)
    {
        D2 = cmd_adc(CMD_ADC_D2 + CMD_ADC_4096); // read D2
        D1 = cmd_adc(CMD_ADC_D1 + CMD_ADC_4096); // read D1

        dT = D2 - (C[5] * pow(2, 8));
        OFF = (C[2] * pow(2, 16)) + (C[4] * dT) / pow(2, 7);
        SENS = (C[1] * pow(2, 15)) + (C[3] * dT) / pow(2, 8);

        T = (2000 + ((C[6] * dT) / pow(2, 23))) / 100;
        P = ((D1 * (SENS / pow(2, 21)) - OFF) / pow(2, 15)) / 10;

        if (T < 20)
        {
            T2 = 3 * pow(dT, 2) / (pow(2, 33));
        }
        else
        {
            T2 = 7 * pow(dT, 2) / (pow(2, 37));
        }

        T = T - T2;

        printf("TEMP     : %f\n", T);
        printf("PRESSURE : %f\n", P);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    /* disable the default wifi logging */
	esp_log_level_set("wifi", ESP_LOG_NONE);

    /* initialize i2c */
    i2c_master_init();

	/* initialize flash memory */
	nvs_flash_init();

	/* start the HTTP Server task */
	xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL);

	/* start the wifi manager task */
	xTaskCreate(&wifi_manager, "wifi_manager", 4096, NULL, 4, NULL);

	/* start the sensor task */
    xTaskCreatePinnedToCore(
            sensorTask,   /* Function to implement the task */
            "sensorTask", /* Name of the task */
            10000,      /* Stack size in words */
            NULL,       /* Task input parameter */
            6 | portPRIVILEGE_BIT, /* Priority of the task */
            NULL,       /* Task handle. */
            0); /* Core where the task should run */

	/* your code should go here. In debug mode we create a simple task on core 2 that monitors free heap memory */
#if WIFI_MANAGER_DEBUG
	xTaskCreatePinnedToCore(&monitoring_task, "monitoring_task", 2048, NULL, 1, NULL, 1);
#endif

char rx_buffer[128];
char addr_str[128];
int addr_family;
int ip_protocol;

struct sockaddr_in destAddr;
destAddr.sin_addr.s_addr = inet_addr("10.10.10.255");
destAddr.sin_family = AF_INET;
destAddr.sin_port = htons(5566);
addr_family = AF_INET;
ip_protocol = IPPROTO_UDP;

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

        while (1) {

            int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
            } else {
                ESP_LOGI(TAG, "Message sent");
            }

            shutdown(sock, 0);
            close(sock);    

            vTaskDelay(1000 / portTICK_PERIOD_MS);    
        }

}
