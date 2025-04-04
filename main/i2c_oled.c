/* SSD1306 Example

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "OLEDDisplay.h"

static const char *TAG = "oled";

#define I2C_MASTER_SCL_IO 	22
#define I2C_MASTER_SDA_IO 	21
#define I2C_MASTER_NUM 		I2C_NUM_0
SemaphoreHandle_t print_mux = NULL;

static void i2c_test_task()
{
	OLEDDisplay_t *oled = OLEDDisplay_init(I2C_MASTER_NUM,0x78,I2C_MASTER_SDA_IO,I2C_MASTER_SCL_IO);

	OLEDDisplay_fillRect(oled,7,7,18,18);
	OLEDDisplay_drawRect(oled,6,32,20,20);
	OLEDDisplay_display(oled);
	vTaskDelay(500 / portTICK_PERIOD_MS);

    OLEDDisplay_setTextAlignment(oled,TEXT_ALIGN_CENTER);
    OLEDDisplay_setFont(oled,ArialMT_Plain_24);
    OLEDDisplay_drawString(oled,64, 00, "Test");
	OLEDDisplay_display(oled);
	vTaskDelay(500 / portTICK_PERIOD_MS);

    OLEDDisplay_setFont(oled,ArialMT_Plain_16);
    OLEDDisplay_drawString(oled,64, 25, "Testing");
	OLEDDisplay_display(oled);
	vTaskDelay(500 / portTICK_PERIOD_MS);

    OLEDDisplay_setFont(oled,ArialMT_Plain_10);
    OLEDDisplay_drawString(oled,64, 40, "This is a test");
	OLEDDisplay_display(oled);

	while(1) {
		OLEDDisplay_setColor(oled,INVERSE);
		OLEDDisplay_fillRect(oled,6,6,20,20);
		OLEDDisplay_fillRect(oled,6,32,20,20);
		OLEDDisplay_display(oled);
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}
}
