#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "nlohmann/json.hpp"

class CSVReader
{
public:
    CSVReader();
    ~CSVReader();

    // 读取指定行数的CSV文件并返回JSON字符串
    std::string readCSVToJSON(const std::string& filename, size_t start_row, size_t end_row);

private:
    // 解析CSV行
    std::vector<std::string> parseLine(const std::string& line);

    // 读取CSV文件的标题栏
    bool readHeader(const std::string& filename);

    nlohmann::json json_data;
    std::vector<std::string> header;
};
