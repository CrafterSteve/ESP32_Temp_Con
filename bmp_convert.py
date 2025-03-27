from PIL import Image
import numpy as np

def bmp_to_binary_array(image_path, threshold=128):
    # 打开图像并转换为灰度图像
    img = Image.open(image_path).convert('L')  # 'L' 表示灰度模式
    
    # 将图像转换为 numpy 数组
    img_array = np.array(img)
    
    # 将灰度图像的每个像素值与阈值进行比较，生成二进制数组
    binary_array = np.where(img_array > threshold, 1, 0)
    
    return binary_array

def print_binary_array(binary_array):
    # 打印二进制数组
    for row in binary_array:
        print(f"{{{', '.join(map(str, row))}}},")

def save_binary_array_to_txt(binary_array, output_path):
    # 将二进制数组写入文本文件
    with open(output_path, 'w') as f:
        for row in binary_array:
            f.write(''.join(map(str, row)) + '\n')

def main():
    binary_array = bmp_to_binary_array("D:/esp_projects/dactest_2.1/main/my_spiffs/bmp_3.bmp")
    save_binary_array_to_txt(binary_array, "D:/esp_projects/dactest_2.1/main/my_spiffs/bmp.txt")

if __name__ == "__main__":
    main()

