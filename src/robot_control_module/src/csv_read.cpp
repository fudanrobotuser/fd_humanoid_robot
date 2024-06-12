#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

class CSVReader
{
public:
    CSVReader();
    ~CSVReader();

    std::string readCSVToJSON(const std::string& filename, size_t start_row, size_t end_row);

private:
    std::vector<std::string> parseLine(const std::string& line);
    bool readHeader(const std::string& filename);

    nlohmann::json json_data;
    std::vector<std::string> header;
};

CSVReader::CSVReader() {}
CSVReader::~CSVReader() {}

std::string CSVReader::readCSVToJSON(const std::string& filename, size_t start_row, size_t end_row)
{
  
    if (!readHeader(filename))
    {
        return "";
    }

    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return "";
    }

    std::string line;
    size_t row = 0;
    while (std::getline(file, line))
    {
        if (row >= start_row && row <= end_row)
        {
            std::vector<std::string> values = parseLine(line);

            if (values.size() == header.size())
            {
                nlohmann::json joint;
                
                joint["positions"] = nlohmann::json::array();
                joint["velocities"] = nlohmann::json::array();
                joint["accelerations"] = nlohmann::json::array();
                for (size_t i = 1; i < header.size(); ++i)
                {
                    joint["positions"].push_back(std::stod(values[i]));
                    joint["velocities"].push_back(0.0); // 假设速度为0
                    joint["accelerations"].push_back(0.0); // 假设加速度为0
                }
                joint["time_from_start"]["sec"] =  0 ;
                if (std::stod(values[0]) == 0.0) {
                    joint["time_from_start"]["sec"] =0.0;
                    joint["time_from_start"]["nanosec"] = 1000000;
                } else {
                  double  t =  std::stod(values[0]);
                  // 获取秒部分
                  int seconds = static_cast<int>(t);
                  

                  joint["time_from_start"]["sec"] =seconds;
                  // 获取毫秒部分
                  long long nanoseconds = static_cast<long long>((t - seconds) * 1000000000LL);
                  joint["time_from_start"]["nanosec"] = nanoseconds; 

                  std::cout << "seconds "<<values[0]<<" "<<t << " "<<seconds << " "<< nanoseconds << std::endl;
                }
                //if (row % 2 == 0) {
                    json_data["points"].push_back(joint);
                //}
            }
        }
        row++;
    }

    file.close();

    return json_data.dump(4); // 使用格式化的输出
    //   return json_data; // 使用格式化的输出
}

std::vector<std::string> CSVReader::parseLine(const std::string& line)
{
    std::vector<std::string> values;
    std::stringstream ss(line);
    std::string value;

    while (std::getline(ss, value, ','))
    {
        values.push_back(value);
    }

    return values;
}

bool CSVReader::readHeader(const std::string& filename)
{
    json_data = nullptr;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    std::string header_line;
    if (!std::getline(file, header_line))
    {
        std::cerr << "读取文件失败" << std::endl;
        return false;
    }

    header = parseLine(header_line);
    json_data["header"]["stamp"]["sec"] = 0;
    json_data["header"]["stamp"]["nanosec"] = 0;
    json_data["header"]["frame_id"] = "";
    json_data["joint_names"] = nlohmann::json::array();
    for (size_t i = 1; i < header.size(); ++i)
    {
        // 清除回车符
        header[i].erase(std::remove(header[i].begin(), header[i].end(), '\r'), header[i].end());
        json_data["joint_names"].push_back(header[i]);
    }
    std::cout << "" << std::endl;
    file.close();
    return true;
}
