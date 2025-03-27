// 走纸与换行
#define LF  0x0A    // line feed
#define CR  0x0D    // carriage return
#define FF  0x0C    // form feed

// 字体与格式
#define ESC_M   0x1B4D  // 10cpi 字体
#define ESC_P   0x1B50  // 12cpi 字体
#define ESC_g   0x1B67  // 双倍宽度字体
#define ESC_i   0x1B21  // 选择字体样式掩码控制
#define ESC_    0x1B2D  // 设定下划线模式，跟1开0关

#define ESC_BMP 0x1B2A  // 位图模式

#define SINGLE_DENSITY_8POINT   0
#define SINGLE_DENSITY_24POINT  0x20
#define DOUBLE_DENSITY_8POINT   1
#define DOUBLE_DENSITY_24POINT  0x21

#define THRESHOLD 128  // 黑白转换阈值

#define HIGH    1
#define LOW     0

#define WIDTH 256
#define HEIGHT 256

#pragma pack(1)  // 结构体按1字节对齐，保证BMP格式正确

// BMP 文件头（14字节）
typedef struct {
    uint16_t fileType;   // "BM" (0x4D42)
    uint32_t fileSize;   // 文件大小
    uint16_t reserved1;
    uint16_t reserved2;
    uint32_t dataOffset; // 像素数据偏移
} BMPFileHeader;

// BMP 信息头（40字节）
typedef struct {
    uint32_t headerSize;  
    int32_t width;       
    int32_t height;      
    uint16_t planes;     
    uint16_t bitCount;   // 24位 BMP
    uint32_t compression;
    uint32_t imageSize;  
    int32_t xPixelsPerMeter;
    int32_t yPixelsPerMeter;
    uint32_t colorsUsed;
    uint32_t colorsImportant;
} BMPInfoHeader;

#pragma pack() // 取消字节对齐限制


void lpt_init(void);
void lpt_strobe(uint8_t level);
uint8_t lpt_ready(void);
void lpt_tx(uint8_t data);
void lpt_send_string(char* string);
void spiffs_init();
void spiffs_info();
void print_image(const char *path, uint8_t* adc_buf);


// Initialize the printer (set the STROBE signal high)
void lpt_init(void) {
    config_gpio_write();
    
    // Send ESC/P init command
    lpt_tx(0x1B);  // ESC
    lpt_tx('@');   // 初始化
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI("LPT", "Printer initialized");
}

void lpt_strobe(uint8_t level) {
    gpio_set_level(D7, level);
}

// Check if the printer is ready (if the BUSY signal is low)
uint8_t lpt_ready(void) {
    uint8_t busy, ack, error, paper_status;

    chip_select(2);
    config_printer_read();

    busy = gpio_get_level(PRN_BUSY);
    ack = gpio_get_level(PRN_ACK);
    error = gpio_get_level(PRN_ERR);
    paper_status = gpio_get_level(PRN_PAPER);
    //ESP_LOGI("Printer Status", "Busy: %d, Ack: %d, Error: %d, Paper: %d", busy, ack, error, paper_status);

    config_gpio_write();

    if (busy == 0) {  // Printer is ready (BUSY is low)
        return 1;
    } else {
        return 0;
    }
}

// Send a character to the printer
void lpt_tx(uint8_t data) {
    int i = 0;

    // Wait for the printer to be ready
    
    while (!lpt_ready()) {
        if(i>5){
            ESP_LOGI("LPT", "Time out");
            return;
        }
        ESP_LOGI("LPT", "Waiting for printer to be ready...");
        vTaskDelay(pdMS_TO_TICKS(10));  // Delay for 100ms
        i++;
    }
    
    
    config_gpio_write();
    chip_select(0);                 // Select 374
    // Send data to the printer (simulate by writing to the printer data register)
    gpio_set_level(D0, (data & (1 << 0)) ? 1 : 0);
    gpio_set_level(D1, (data & (1 << 1)) ? 1 : 0);
    gpio_set_level(D2, (data & (1 << 2)) ? 1 : 0);
    gpio_set_level(D3, (data & (1 << 3)) ? 1 : 0);
    gpio_set_level(D4, (data & (1 << 4)) ? 1 : 0);
    gpio_set_level(D5, (data & (1 << 5)) ? 1 : 0);
    gpio_set_level(D6, (data & (1 << 6)) ? 1 : 0);
    gpio_set_level(D7, (data & (1 << 7)) ? 1 : 0);
    /*
    ESP_LOGI("LPT", "D0: %d", (data & (1 << 0)) ? 1 : 0);
    ESP_LOGI("LPT", "D1: %d", (data & (1 << 1)) ? 1 : 0);
    ESP_LOGI("LPT", "D2: %d", (data & (1 << 2)) ? 1 : 0);
    ESP_LOGI("LPT", "D3: %d", (data & (1 << 3)) ? 1 : 0);
    ESP_LOGI("LPT", "D4: %d", (data & (1 << 4)) ? 1 : 0);
    ESP_LOGI("LPT", "D5: %d", (data & (1 << 5)) ? 1 : 0);
    ESP_LOGI("LPT", "D6: %d", (data & (1 << 6)) ? 1 : 0);
    ESP_LOGI("LPT", "D7: %d", (data & (1 << 7)) ? 1 : 0);
    */
    vTaskDelay(pdMS_TO_TICKS(1));

    chip_select(4);                 // Raising Edge for 374
    lpt_strobe(LOW);              // Set STROBE low
    vTaskDelay(pdMS_TO_TICKS(1));
    lpt_strobe(HIGH);             // Set STROBE high
    chip_select(7);                 // CS7 wont interfere databus.
    /*
    switch (data) {
        case LF:    ESP_LOGI("LPT", "Line feed");break;
        case CR:    ESP_LOGI("LPT", "Carriage return");break;
        default:    ESP_LOGI("LPT", "Sent character: 0x%2x", data);break;
    }
    */
}
   

void lpt_send_string(char* string)
{
    int i = 0;
    uint8_t data;

    while (string[i])
    {
        if(i>25){break;}
        data = string[i];
        lpt_tx(data);
        i++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    lpt_tx(LF);
}

// SPIFFS 配置
esp_vfs_spiffs_conf_t spiffs_conf = {
    .base_path = "/storage",
    .partition_label = "storage",
    .max_files = 5,
    .format_if_mount_failed = true
};

// SPIFFS 初始化
void spiffs_init() {
    esp_err_t ret;
    
    ret = esp_vfs_spiffs_register(&spiffs_conf);    // 挂载 SPIFFS
    if (ret != ESP_OK) {
        ESP_LOGE("SPIFFS", "Mount FAILED: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI("SPIFFS", "Mount SUCCESSED");
}

// 获取 SPIFFS 信息
void spiffs_info(){
    esp_err_t ret;
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(spiffs_conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE("SPIFFS", "Failed to get SPIFFS partition info (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI("SPIFFS", "Partition Size: Total=%d, Used=%d", total, used);
    }
}

void print_image(const char *path, uint8_t* adc_buf) {

    FILE *file = fopen(path, "r");
    if (!file) {
        ESP_LOGE("BMP", "Failed to open file");
        return;
    }

    // 发送 GS v 0 指令
    lpt_tx(0x1D);  // GS
    lpt_tx(0x76);  // 'v'
    lpt_tx(0x30);  // '0'
    lpt_tx(0x00);  // 密度

    lpt_tx(WIDTH/8);     // 宽度
    lpt_tx(0x00);
    lpt_tx(HEIGHT-1);    // 高度
    lpt_tx(0x00);

    // 初始化 img 数组
                             
    uint8_t bmp_data[WIDTH/8];  // 每行的压缩数据

    // 逐行读取并处理数据
    for (int y = 0; y < HEIGHT; y++) {
        uint8_t img[WIDTH];
        for (int x = 0; x < WIDTH; x++) {
            char ch='1';
            fscanf(file, " %c", &ch);
            if (ch == '0') {
                img[x] = 0x00;
            } else if (ch == '1') {
                img[x] = 0x01;
            } else {
                ESP_LOGI("DEBUG", "error");
            }
            //ESP_LOGI("DEBUG", "%d", img[x]);
        }

        // 叠加曲线
        for(int i = 0; i < 240; i++){
            if(adc_buf[i] == (256 - y - 14)){img[i+14] = 0;}     // 14 pixel offset (curve starts for 0 on axis.)
            if(adc_buf[i] == (256 - y + 1 - 14)){img[i+14] = 0;}
        }

        int j = 0;
        int x = 0;
        uint8_t _data = 0;
        
        // 逐像素处理一行数据
        for (int i = 0; i < WIDTH/8; i++) {
            _data |= ((img[i*8+0]==1) ? 0 : 1) << 7;
            _data |= ((img[i*8+1]==1) ? 0 : 1) << 6; 
            _data |= ((img[i*8+2]==1) ? 0 : 1) << 5; 
            _data |= ((img[i*8+3]==1) ? 0 : 1) << 4; 
            _data |= ((img[i*8+4]==1) ? 0 : 1) << 3; 
            _data |= ((img[i*8+5]==1) ? 0 : 1) << 2; 
            _data |= ((img[i*8+6]==1) ? 0 : 1) << 1; 
            _data |= ((img[i*8+7]==1) ? 0 : 1) << 0;
            
            //ESP_LOGI("DEBUG", "0x%2x", _data);
            bmp_data[i] = _data;
            _data = 0;
        }

        // 逐字节输出图像数据
        for (int i = 0; i < WIDTH/8; i++) {
            lpt_tx(bmp_data[i]);
            //lpt_tx(0xff);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    lpt_tx(0x1B); lpt_tx(0x64); lpt_tx(1);
    fclose(file);
    vTaskDelay(pdMS_TO_TICKS(10));
}
