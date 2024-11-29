#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <filesystem> // C++17

typedef uint16_t LabelType;


// std::vector<LabelType> readSemanticKittiLabelFile(const std::string& filename) {
//     std::vector<LabelType> labels;
//     std::ifstream file(filename, std::ios::binary);
//     if (file.is_open()) {
//         LabelType label;
//         while (file.read(reinterpret_cast<char*>(&label), sizeof(LabelType))) {
//             labels.push_back(label);
//             // file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
//         }
//         if (file.bad()) {
//             std::cerr << "Error occurred while reading the file." << std::endl;
//         }
//         file.close();
//     } else {
//         std::cerr << "Unable to open file " << filename << std::endl;
//     }
//     return labels;
// }

std::vector<uint16_t> readSemanticKittiLabelFile(const std::string& filename) {
    std::vector<uint16_t> labels;
    try {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open file");
        }

        uint16_t label;
        while (file.read(reinterpret_cast<char*>(&label), sizeof(2*label))) {
            labels.push_back(label);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error occurred while reading the file: " << e.what() << std::endl;
    }
    return labels;
}

int main(int argc, char** argv) {
    const std::string filename = argv[1];
    std::vector<LabelType> labels = readSemanticKittiLabelFile(filename);

    // 打印读取到的标签数量
    std::cout << "Number of labels: " << labels.size() << std::endl;

    // 写入每个标签到当前工作目录的 txt 文件
    std::string output_filename = "/home/ifly/MyWork/mytools_ws/src/data/output.txt";
    std::ofstream outfile(output_filename);
    if (outfile.is_open()) {
        for (const auto& label : labels) {
            outfile << label << std::endl;
        }
        outfile.close();
        std::cout << "Labels saved to " << output_filename << std::endl;
    } else {
        std::cerr << "Unable to open output file " << output_filename << std::endl;
    }

    return 0;
}
