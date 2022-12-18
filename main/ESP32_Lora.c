#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const int RX_BUF_SIZE= 2024;
#define TXD_PIN2 (GPIO_NUM_17)
#define RXD_PIN2 (GPIO_NUM_16)
//for UART2
void init2(void){
    const uart_config_t uart_config_2={
        .baud_rate = 9600,
        .data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE,
        .stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,
        .source_clk=UART_SCLK_APB
    };
    uart_driver_install(UART_NUM_2,RX_BUF_SIZE*2,0,0,NULL,0);
    uart_param_config(UART_NUM_2,&uart_config_2);
    uart_set_pin(UART_NUM_2,TXD_PIN2,RXD_PIN2,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
}


void task_tx(void *ptr)
{
    const char str[5]="GPGGA";
    char* p;
    uint8_t* data= (uint8_t*)malloc(RX_BUF_SIZE+1);
    uint8_t* str2=(uint8_t*)malloc(RX_BUF_SIZE+1);
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int i=0;
        const int rxbytes=uart_read_bytes(UART_NUM_2,data,RX_BUF_SIZE,500 / portTICK_RATE_MS);
            if(rxbytes>0)
                {
                p=strstr((const char*)data,str);
                while(*p!='\n'){
                    *(str2+i)=(uint8_t)(*p);
                    printf("%c",*(str2+i));
                    i++;
                    p++;
                }
                printf("\n");
                data[rxbytes]=0;
            }
    // lora_send_packet((uint8_t*)str2,i);
        //printf("Packet sent ...\n");
    }
}

void app_main()
{
   init2();
//    lora_init();
//    lora_set_frequency(434000000);
   // lora_set_spreading_factor(12);
   //lora_set_bandwidth(7);
   // lora_set_coding_rate(8);
   // lora_set_tx_power(17);
//    lora_enable_crc();
   xTaskCreate(&task_tx, "task_tx", 2048*5, NULL, 5, NULL);
}