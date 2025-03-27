# ESP32_Temp_Con
An temperture control device, based on ESP32. With custom PCB board, it can fit an ESP32-DevKit as controller, DAC and ADC is done on board, as well as, a 5 to +-12v power supply.

## Notes:
项目中 ./main/my_spiffs/ 目录下的文件会通过命令，全部烧写的分区 label 的 flash 中。

创建自定义空间前，先做配置：
used "idf.py menuconfig" to change partition table to csv.

created "partitions.csv" with 256KB of spiffs as "/storage": （以下是"partitions.csv"文件应该包含的内容）
nvs, data, nvs, 0x9000, 0x6000
phy_init, data, phy, 0xf000, 0x1000
factory, app, factory, 0x10000, 0x100000
storage, data, spiffs, 0x110000, 0x40000

## SPIFFS指令：
使用时注意 python 工具的路径。“./” 代表本项目的根目录

python D:\esp\v5.2.1\esp-idf\components\partition_table\gen_esp32part.py .\partitions.csv .\partitions.bin                                      // 从csv生成分区文件

idf.py partition-table-flash                                                                                                                    // 烧录分区文件

python D:\esp\v5.2.1\esp-idf\components\spiffs\spiffsgen.py 0x40000 D:\esp_projects\dactest_2.1\main\my_spiffs my_spiffs_img                    // 创建SPIFFS镜像文件

python D:\esp\v5.2.1\esp-idf\components\partition_table\parttool.py -p COM3 write_partition --partition-name storage --input .\my_spiffs_img    // 写入esp32

## 程序调用
in code: base_path = "/storage"     // esp32内文件目录
"/storage/bmp_test.bmp" 为存入的文件
FILE *file = fopen("/storage/bmp_test.bmp", "rb");  // 如此打开flash上的文件
可通过 esp_spiffs_info(spiffs_conf.partition_label, &total, &used); 查看占用，检查是否写入。
