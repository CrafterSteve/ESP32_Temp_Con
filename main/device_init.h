#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "OLEDDisplay.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"


//Data Bus
#define D0  32
#define D1  33
#define D2  25
#define D3  26
#define D4  27
#define D5  14
#define D6  12
#define D7  13

#define WR  19
#define RD  18

//Chip Select
#define A0  4
#define A1  0
#define A2  2

//Printer
#define PRN_STROBE  D7
#define PRN_ERR     D0
#define PRN_STAT    D1
#define PRN_PAPER   D2
#define PRN_BUSY    D3
#define PRN_ACK     D4

//I2C Bus
#define SCL 22
#define SDA 21
#define I2C_MASTER_FREQ_HZ   100000 
#define I2C_MASTER_NUM       I2C_NUM_0

#define EEPROM_ADDR          0x50
#define PCF8523_I2C_ADDR     0x68

uint8_t reg_addr = 0x00;                    // PCF8523寄存器地址

SemaphoreHandle_t print_mux = NULL;


const gpio_num_t gpio_pins[] = {
    D0,D1,D2,D3,D4,D5,D6,D7,
    A0,A1,A2,
    WR,RD
};

const gpio_num_t data_pins[] = {
    D0,D1,D2,D3,D4,D5,D6,D7
};

const gpio_num_t printer_stat_pins[] = {
    PRN_ERR,PRN_STAT,PRN_PAPER,PRN_BUSY,PRN_ACK
};

const gpio_num_t printer_control_pins[] = {
    D0,D1,D2,D3,D4,D5,D6,D7
};

// I2C init---------------------------------------------
esp_err_t i2c_master_init(void)
{
    esp_err_t ret;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {ESP_LOGE("I2C", "init FAILED: %s", esp_err_to_name(ret));}
    else{ESP_LOGI("I2C", "init SUCCESSED");}
    return  ret;
}

void i2c_scan() {
    uint8_t address;
    for (address = 0; address < 128; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI("I2C", "I2C 地址 0x%02X 被找到", address);
        }
    }
}

// 写数据到 AT24C64
esp_err_t write_to_eeprom(uint16_t addr, uint8_t *data, size_t size)
{
    esp_err_t ret;
    uint8_t write_buf[64];

    write_buf[0] = (uint8_t)(addr >> 8);    // 高字节
    write_buf[1] = (uint8_t)(addr & 0xFF);  // 低字节
    for (size_t i = 0; i < size; i++) {
        write_buf[2 + i] = data[i];
    }

    // 启动 I2C 写入操作
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                  // 启动信号
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_WRITE, true);  // 发送 I2C 地址
    ESP_LOGI("I2C Address", "0x%02X", (EEPROM_ADDR << 1 | I2C_MASTER_WRITE));
    i2c_master_write(cmd, write_buf, size + 2, true);                       // 写入数据
    i2c_master_stop(cmd);                                                   // 停止信号

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));   // 执行命令
    i2c_cmd_link_delete(cmd);                                               // 删除命令

    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "I2C write failed with error: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t write_byte_to_eeprom(uint16_t addr, uint8_t data)
{
    esp_err_t ret;
    uint8_t write_buf[3];

    write_buf[0] = (uint8_t)(addr >> 8);    // 高字节
    write_buf[1] = (uint8_t)(addr & 0xFF);  // 低字节
    write_buf[2] = data;

    // 启动 I2C 写入操作
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                  // 启动信号
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_WRITE, true);  // 发送 I2C 地址
    ESP_LOGI("I2C Address", "0x%02X", (EEPROM_ADDR << 1 | I2C_MASTER_WRITE));
    i2c_master_write(cmd, write_buf, 3, true);                       // 写入数据
    i2c_master_stop(cmd);                                                   // 停止信号

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));   // 执行命令
    i2c_cmd_link_delete(cmd);                                               // 删除命令

    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "I2C write failed with error: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    return ESP_OK;
}

// 从 AT24C64 读取数据
esp_err_t read_from_eeprom(uint16_t addr, uint8_t *data, size_t size)
{
    esp_err_t ret;
    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)(addr >> 8);     // 高字节
    addr_buf[1] = (uint8_t)(addr & 0xFF);   // 低字节

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                  // 启动信号
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_WRITE, true);  // 发送 I2C 写入命令
    i2c_master_write(cmd, addr_buf, 2, true);                               // 写入地址
    i2c_master_start(cmd);                                                  // 启动信号
    i2c_master_write_byte(cmd, EEPROM_ADDR << 1 | I2C_MASTER_READ, true);   // 发送 I2C 读取命令
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);                 // 读取数据
    i2c_master_stop(cmd);                                                   // 停止信号

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));   // 执行命令
    i2c_cmd_link_delete(cmd);                                               // 删除命令

    return ret;
}


// Data Bus (D0-7) and Chip Select---------------------------------------------
void init_gpio()
{
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE 
    };
    for (int i = 0; i < sizeof(gpio_pins) / sizeof(gpio_pins[0]); i++) {
        io_conf.pin_bit_mask |= (1ULL << gpio_pins[i]);
    }
    gpio_config(&io_conf);
}

void config_gpio_write()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE 
    };
    for (int i = 0; i < sizeof(data_pins) / sizeof(data_pins[0]); i++) {
        io_conf.pin_bit_mask |= (1ULL << data_pins[i]);
    }
    gpio_config(&io_conf);
}

void config_printer_write()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE 
    };
    for (int i = 0; i < sizeof(printer_control_pins) / sizeof(printer_control_pins[0]); i++) {
        io_conf.pin_bit_mask |= (1ULL << printer_control_pins[i]);
    }
    gpio_config(&io_conf);
}

void config_gpio_read()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE 
    };
    for (int i = 0; i < sizeof(data_pins) / sizeof(data_pins[0]); i++) {
        io_conf.pin_bit_mask |= (1ULL << data_pins[i]);
    }
    gpio_config(&io_conf);
}

void config_printer_read()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE 
    };
    for (int i = 0; i < sizeof(printer_stat_pins) / sizeof(printer_stat_pins[0]); i++) {
        io_conf.pin_bit_mask |= (1ULL << printer_stat_pins[i]);
    }
    gpio_config(&io_conf);
}

void chip_select(int chip_select)
{
    gpio_set_level(A2, (chip_select >> 2) & 0x01);
    gpio_set_level(A1, (chip_select >> 1) & 0x01);
    gpio_set_level(A0, chip_select & 0x01);
    //0x01 即用 001 做mask，位移后仅抽取最低位值
}

// Map a number from -12 to 12 to an 8-bit binary number (0-255)
uint8_t map_to_8bit(int value) {
    if (value < 0) value = 0;
    if (value > 255) value = 255;
    return (uint8_t)(value);
}

void set_dataBus(uint8_t value) {
    // Write mode
    gpio_set_level(WR, 0);  // Set WR to low (write)
    gpio_set_level(RD, 1);  // Set RD to high (read disabled)
    config_gpio_write();

    ESP_LOGI("DATA_BUS", "Data数值: %d", value);
    gpio_set_level(D0, (value & (1 << 0)) ? 1 : 0);  // Check if bit 0 is 1
    gpio_set_level(D1, (value & (1 << 1)) ? 1 : 0);  // Check if bit 1 is 1
    gpio_set_level(D2, (value & (1 << 2)) ? 1 : 0);  // Check if bit 2 is 1
    gpio_set_level(D3, (value & (1 << 3)) ? 1 : 0);  // Check if bit 3 is 1
    gpio_set_level(D4, (value & (1 << 4)) ? 1 : 0);  // Check if bit 4 is 1
    gpio_set_level(D5, (value & (1 << 5)) ? 1 : 0);  // Check if bit 5 is 1
    gpio_set_level(D6, (value & (1 << 6)) ? 1 : 0);  // Check if bit 6 is 1
    gpio_set_level(D7, (value & (1 << 7)) ? 1 : 0);  // Check if bit 7 is 1

    gpio_set_level(WR, 1);
}


int read_dataBus() {
    int value = 0;

    gpio_set_level(WR, 0);
    config_gpio_read();
    gpio_set_level(WR, 1);  //Trigger ADC convert
    gpio_set_level(RD, 0);

    value |= gpio_get_level(D0) << 0;  // Read D0 and set bit 0
    value |= gpio_get_level(D1) << 1;  // Read D1 and set bit 1
    value |= gpio_get_level(D2) << 2;  // Read D2 and set bit 2
    value |= gpio_get_level(D3) << 3;  // Read D3 and set bit 3
    value |= gpio_get_level(D4) << 4;  // Read D4 and set bit 4
    value |= gpio_get_level(D5) << 5;  // Read D5 and set bit 5
    value |= gpio_get_level(D6) << 6;  // Read D6 and set bit 6
    value |= gpio_get_level(D7) << 7;  // Read D7 and set bit 7
    return value; 
}

// RTC---------------------------------------------------------------------------
esp_err_t RTC_set()
{
    esp_err_t ret;
    uint8_t set_time[11];

    set_time[0] = reg_addr;                     // 寄存器地址
    set_time[1] = 0x00;
    set_time[2] = 0x00;
    set_time[3] = 0xF0;
    set_time[4] = 0x56 & 0xFF;                  // 秒
    set_time[5] = 0x34 & 0x7F;                  // 分钟
    set_time[6] = 0x12 & 0x3F;                  // 小时
    set_time[7] = 0x15 & 0x3F;                  // 日期
    set_time[8] = 0x01;                         // 星期（可以随便设定，例如 01，表示星期一）
    set_time[9] = 0x05 & 0x1F;                  // 月份
    set_time[10] = 0x24;                        // 年份：23 表示 2023 年
    
    // 设置 RTC 初始时间
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, PCF8523_I2C_ADDR, set_time, sizeof(set_time), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {ESP_LOGE("I2C", "时间设置失败: %s", esp_err_to_name(ret));}
    else{
        ESP_LOGI("I2C", "时间设置成功 %02X:%02X:%02X 20%02X.%02X.%02X", 
            set_time[6], set_time[5], set_time[4], set_time[10], set_time[9], set_time[7]);
        }
    return ret;
}

esp_err_t read_pcf8523_time(uint8_t *data) {
    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM,             // I2C 总线端口
        PCF8523_I2C_ADDR,           // I2C 地址
        &reg_addr,                  // 写入寄存器地址
        1,                          // 写入 1 字节（寄存器地址）
        data,                       // 读取数据
        10,                         // 读取 7 字节数据（时间等）
        1000 / portTICK_PERIOD_MS   // 超时
    );
    return ret;
}

void rtc_read(uint8_t time_data[])
{
    esp_err_t ret;
    uint8_t data[10] = {0};

    ret = read_pcf8523_time(data);
    if (ret == ESP_OK) {
        // 解析并显示时间
        uint8_t second = data[3] & 0x7F;  // 忽略无效位
        uint8_t minute = data[4] & 0x7F;
        uint8_t hour = data[5] & 0x1F;
        uint8_t day = data[6] & 0x3F;
        uint8_t weekdays = data[7];
        uint8_t month = data[8] & 0x1F;
        uint8_t year = data[9];

        time_data[0] = second;
        time_data[1] = minute;
        time_data[2] = hour;
        time_data[3] = day;
        time_data[4] = weekdays;
        time_data[5] = month;
        time_data[6] = year;

    } else {
        ESP_LOGE("PCF8523", "读取 RTC 时间失败: %s", esp_err_to_name(ret));
    }
}
        