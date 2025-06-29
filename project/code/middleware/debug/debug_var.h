#ifndef DEBUG_VAR_H
#define DEBUG_VAR_H

#include <string>
#include <functional>

using namespace std;

// 调试变量结构
struct DebugVar
{
    string name;                                  // 变量名
    function<string()> getter;                    // 获取值
    function<void(const string&)> setter;        // 设置值 - 返回void
    bool read_only;                               // 是否只读

    DebugVar() = default;

    DebugVar(string var_name,
        function<string()> get_func,
        function<void(const string&)> set_func = nullptr,
        bool readonly = false)
        : name(var_name), getter(get_func), setter(set_func), read_only(readonly)
    {
    }

    string get() const
    {
        return getter ? getter() : "";
    }

    void set(const string& value)
    {
        if (setter && !read_only)
        {
            setter(value);
        }
    }
};

// 便捷的变量创建函数
template<typename T>
DebugVar make_debug_var(string name, T* ptr)
{
    return DebugVar(
        name,
        [ptr]() -> string {
            if constexpr (is_same<T, bool>::value)
            {
                return *ptr ? "true" : "false";
            }
            else
            {
                return to_string(*ptr);
            }
        },
        [ptr](const string& value) -> void {
            if constexpr (is_same<T, bool>::value)
            {
                if (value == "true" || value == "1") 
                    *ptr = true;
                else if (value == "false" || value == "0") 
                    *ptr = false;
            }
            else if constexpr (is_same<T, float>::value)
            {
                *ptr = stof(value);
            }
            else if constexpr (is_integral<T>::value)
            {
                *ptr = static_cast<T>(stoi(value));
            }
        }
    );
}

template<typename T>
DebugVar make_readonly_var(string name, T* ptr)
{
    return DebugVar(
        name,
        [ptr]() -> string {
            if constexpr (is_same<T, bool>::value)
            {
                return *ptr ? "true" : "false";
            }
            else
            {
                return to_string(*ptr);
            }
        },
        nullptr,
        true
    );
}

// DebugVar make_function_var(string name, function<void(const string&)> func)
// {
//     return DebugVar(
//         name,
//         []() -> string { return ""; },
//         func,
//         false
//     );
// }

#endif // DEBUG_VAR_H