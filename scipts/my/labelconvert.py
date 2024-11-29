import struct
import sys
from collections import defaultdict


def count_numbers_in_file(filename):
    counts = defaultdict(int)

    try:
        with open(filename, 'r') as file:
            for line in file:
                line = line.strip()  # 去掉前后空白字符
                if line:  # 确保行不为空
                    number = int(line)  # 转换为整数
                    counts[number] += 1  # 统计出现次数
    except Exception as e:
        print(f"Error occurred while reading the file: {e}")
        return

    # 输出结果到终端
    for number, count in sorted(counts.items()):
        print(f"{number} : {count}")

def read_semantic_kitti_label_file(filename):
    labels = []
    try:
        with open(filename, 'rb') as file:
            while True:
                # data = file.read(4)  # 读取 4 字节（uint32_t）
                data = file.read(2)  # 读取 4 字节（uint32_t）
                if not data:
                    break
                # label = struct.unpack('I', data)[0]  # 解包为 uint32_t
                label = struct.unpack('H', data)[0]  # 解包为 uint32_t
                labels.append(label)
    except Exception as e:
        print(f"Error occurred while reading the file: {e}")
    return labels

def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py <input_filename>")
        return

    input_filename = sys.argv[1]
    labels = read_semantic_kitti_label_file(input_filename)

    # 打印读取到的标签数量
    print(f"Number of labels: {len(labels)//2}")

    # 写入每个标签到指定的 txt 文件
    filename = input_filename[-12:-6]
    output_filename = "/home/ifly/MyWork/mytools_ws/src/scipts/testdata/" + filename +".txt"
    try:
        with open(output_filename, 'w') as outfile:
            for index, label in enumerate(labels):
                if index % 2 == 0:  # 只写入奇数行（0, 2, 4...）
                    outfile.write(f"{label}\n")
        print(f"Labels saved to {output_filename}")
    except Exception as e:
        print(f"Unable to open output file {output_filename}: {e}")
    # chakangbiaoqian
    count_numbers_in_file(output_filename)

if __name__ == "__main__":
    main()

