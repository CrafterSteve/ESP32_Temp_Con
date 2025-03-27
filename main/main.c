#include "./device_init.h"
#include "./printer.h"
#include <stdlib.h>


esp_err_t ret;
uint8_t read_data[20];
uint8_t RTC_time[7] = {0};
uint8_t ADC_value;
uint8_t read_int;
uint8_t hour, minute, second, year, month, day, second_old;
uint16_t epr_addr = 0;

char str[20];
char adc_str[16];
uint8_t adc_buf[240];
char time_str[25];
char write_str[16];
uint8_t write_data[16];

uint8_t count = 0;
int loop_count = 0;
uint8_t time_count = 0;
uint8_t time_count_old = 0;


void app_main(void)
{
    // 设备初始化
    init_gpio();
    //lpt_init(); 
    spiffs_init();
    i2c_master_init();
    i2c_scan();             // i2c 总线地址扫描
    OLEDDisplay_t *oled = OLEDDisplay_init(I2C_MASTER_NUM,0x78,SDA,SCL);
    RTC_set();              // RTC 初始化、时间设置
    
    // Intro Screen-----------------------------------------start
    OLEDDisplay_drawRect(oled,6,32,20,20);
	OLEDDisplay_display(oled);
	vTaskDelay(500 / portTICK_PERIOD_MS);

    OLEDDisplay_setTextAlignment(oled,TEXT_ALIGN_CENTER);
    OLEDDisplay_setFont(oled,ArialMT_Plain_24);
    OLEDDisplay_drawString(oled,64, 00, "Temp_CON");
	OLEDDisplay_display(oled);
	vTaskDelay(500 / portTICK_PERIOD_MS);

    OLEDDisplay_setFont(oled,ArialMT_Plain_10);
    OLEDDisplay_drawString(oled,64, 40, "----------");
	OLEDDisplay_display(oled);
    // Intro Screen------------------------------------------end

    // erase eeprom data
    for(int i = 0; i < 240; i++){
        sprintf(write_str, "%d", 14);
        for(int i = 0; i < 16; i++){
            write_data[i] = (uint8_t)write_str[i];
        }
        ret = write_to_eeprom((uint16_t)(i*16), write_data, 16);
        if (ret != ESP_OK) {ESP_LOGE("I2C", "eeprom read faild: %s", esp_err_to_name(ret));}
        else{ESP_LOGI("I2C_storage", "write data: %s", write_data);}
    }

    while (1) {
        spiffs_info();

        vTaskDelay(pdMS_TO_TICKS(50));

        chip_select(1);
        set_dataBus(255);                                   // set voltage -9 to 9v (0-255)

        chip_select(3);
        ADC_value = read_dataBus();                         // read voltage
        sprintf(adc_str, "ADC: %d", ADC_value);

        rtc_read(RTC_time);
        second = RTC_time[0];
        sprintf(time_str, "%02X:%02X:%02X 20%02X.%02X.%02X", 
            RTC_time[2], RTC_time[1], RTC_time[0], RTC_time[6], RTC_time[5], RTC_time[3]);
        
        // 
        if(epr_addr > 240*16){
            epr_addr = 0;
        }
        
        // gloable timer counter
        if(second != second_old){
            sprintf(write_str, "%d", ADC_value);
            for(int i = 0; i < 16; i++){
                write_data[i] = (uint8_t)write_str[i];
            }
            ret = write_to_eeprom(epr_addr, write_data, 16);
            if (ret != ESP_OK) {ESP_LOGE("I2C", "eeprom read faild: %s", esp_err_to_name(ret));}
            else{ESP_LOGI("I2C_storage", "write data: %s", write_data);}

            // display refresh every second.
            OLEDDisplay_clear(oled);
            OLEDDisplay_setFont(oled,ArialMT_Plain_16);
            OLEDDisplay_drawString(oled,64, 25, adc_str);       // ADC value display

            OLEDDisplay_setFont(oled,ArialMT_Plain_10);
            OLEDDisplay_drawString(oled,64, 00, time_str);      // RTC time display

            OLEDDisplay_display(oled);
 
            second_old = second;
            time_count++;
        }
        epr_addr = epr_addr + 16;

        // print temp curve every 30s (actually about 46s, printing takes time.)
        if(time_count > time_count_old + 30){
            for(int i = 0; i < 240; i++){
                ret = read_from_eeprom((uint16_t)(i*16), read_data, 16);
                if (ret != ESP_OK) {ESP_LOGE("I2C", "eeprom read faild: %s", esp_err_to_name(ret));}
                else{ESP_LOGI("I2C_storage", "read data: %s", read_data);}
                adc_buf[i] = atoi((char*)read_data);
                ESP_LOGI("I2C_storage", "adc_data_buffer: %d", adc_buf[i]);
            }

            OLEDDisplay_clear(oled);
            OLEDDisplay_setFont(oled,ArialMT_Plain_16);
            OLEDDisplay_drawString(oled,64, 25, "Printing...");
            OLEDDisplay_display(oled);

            lpt_send_string(time_str);
            print_image("/storage/bmp.txt", adc_buf);
            time_count_old = time_count;
        }

        

        //char lpt_str[25];
        //sprintf(lpt_str, "hello printer");
        //lpt_send_string(lpt_str);
        

    }
}
