#pragma once

#include <vector>
#include <string>

#include <map>
#include <fstream>
#include <iostream>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

class CSVReader {
public:
    template<typename T>
    static std::vector<T> Read(const std::string& filename)
    {
        std::vector<T> data;
        std::string line;
        std::ifstream fs(filename);

        // 读取表头
        std::getline(fs, line);
        boost::tokenizer<boost::escaped_list_separator<char>> tokHeader(line);
        std::vector<std::string> headers(tokHeader.begin(), tokHeader.end());

        // 读取数据行
        while (std::getline(fs, line)) {
            boost::tokenizer<boost::escaped_list_separator<char>> tok(line);
            std::vector<std::string> tokens(tok.begin(), tok.end());

            if (tokens.size() < headers.size()) {
                std::cerr << "Skipping line due to mismatched token count." << std::endl;
                continue;
            }

            std::map<std::string, std::string> values;
            for (size_t i = 0; i < headers.size(); ++i) {
                values[headers[i]] = tokens[i];
            }

            T item;
            item.fromkv(values);
            data.push_back(item);
        }

        return data;
    }
};
