from collections import defaultdict
import sys

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

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <input_filename>")

    input_filename = sys.argv[1]

    count_numbers_in_file(input_filename)

