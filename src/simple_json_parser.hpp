#pragma once

#include <string>
#include <map>
#include <sstream>
#include <iomanip>

// 简单的 JSON 解析器（仅支持基本类型）
class SimpleJsonParser {
public:
    static std::map<std::string, std::string> parse(const std::string& json_str) {
        std::map<std::string, std::string> result;
        std::string key, value;
        bool in_key = false, in_value = false, in_string = false;
        bool escape_next = false;
        bool key_started = false, value_started = false;
        
        for (size_t i = 0; i < json_str.length(); ++i) {
            char c = json_str[i];
            
            if (escape_next) {
                if (in_string) {
                    value += c;
                }
                escape_next = false;
                continue;
            }
            
            if (c == '\\') {
                escape_next = true;
                if (in_string) {
                    value += c;
                }
                continue;
            }
            
            if (c == '"') {
                in_string = !in_string;
                if (in_string && !key_started && !value_started) {
                    key_started = true;
                } else if (in_string && key_started && !value_started) {
                    value_started = true;
                }
                continue;
            }
            
            if (!in_string) {
                // 跳过空白字符
                if (c == ' ' || c == '\n' || c == '\r' || c == '\t') {
                    continue;
                }
                
                if (c == '{') {
                    continue;
                }
                
                if (c == ':') {
                    in_key = false;
                    in_value = true;
                    key_started = false;
                    value_started = false;
                    continue;
                }
                
                if (c == ',' || c == '}') {
                    if (!key.empty() && !value.empty()) {
                        // 去除引号
                        trim_quotes(key);
                        trim_quotes(value);
                        result[key] = value;
                    }
                    key.clear();
                    value.clear();
                    in_key = true;
                    in_value = false;
                    key_started = false;
                    value_started = false;
                    if (c == '}') {
                        break;
                    }
                    continue;
                }
            }
            
            if (in_key || (!in_key && !in_value)) {
                key += c;
                in_key = true;
            } else if (in_value) {
                value += c;
            }
        }
        
        // 处理最后一个键值对
        if (!key.empty() && !value.empty()) {
            trim_quotes(key);
            trim_quotes(value);
            result[key] = value;
        }
        
        return result;
    }
    
    static bool get_bool(const std::string& str) {
        return str == "true" || str == "1";
    }
    
    static double get_double(const std::string& str) {
        try {
            return std::stod(str);
        } catch (...) {
            return 0.0;
        }
    }
    
    static int get_int(const std::string& str) {
        try {
            return std::stoi(str);
        } catch (...) {
            return 0;
        }
    }
    
private:
    static void trim_quotes(std::string& str) {
        if (str.length() >= 2 && str[0] == '"' && str[str.length()-1] == '"') {
            str = str.substr(1, str.length() - 2);
        }
    }
};
