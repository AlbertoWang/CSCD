#pragma once
#include <string>
#include <vector>
#include <cstdlib>
#include <cstring>

namespace zjl {
    namespace Util {
        inline bool string_end_with(const std::string&str, const std::string& suffix) {
            return strcmp(str.data() + str.size() - suffix.size(), suffix.data()) == 0; 
        }

        inline std::vector<std::string> string_split_by_ws(const std::string&str) {
            using std::vector;
            using std::string;
            vector<string> sub_strings;
            string string_copy = str;
            size_t pos = 0;
            while (
                ((pos = string_copy.find(" ")) != std::string::npos) ||
                ((pos = string_copy.find("\t")) != std::string::npos) ||
                ((pos = string_copy.find("\v")) != std::string::npos) ||
                ((pos = string_copy.find("\f")) != std::string::npos) 
                ) {
                const auto sub_str = string_copy.substr(0, pos);
                if (!sub_str.empty()) {
                    sub_strings.push_back(sub_str);
                }
                string_copy.erase(0, pos + 1); // if delimiter is a string, 1 should be delimiter.length()
            }
            if (!string_copy.empty()) sub_strings.push_back(string_copy);
            return sub_strings;
        }

        inline std::vector<std::string> string_split_by(const std::string&str, const std::string& delimiter) {
            using std::vector;
            using std::string;
            vector<string> subStrings;
            string stringCopy = str;
            size_t pos = 0;
            while ((pos = stringCopy.find(delimiter)) != std::string::npos) {
                const auto subStr = stringCopy.substr(0, pos);
                if (!subStr.empty()) {
                    subStrings.push_back(subStr);
                }
                stringCopy.erase(0, pos + delimiter.size()); // if delimiter is a string, 1 should be delimiter.length()
            }
            if (!stringCopy.empty()) subStrings.push_back(stringCopy);
            return subStrings;
        }
    }
};