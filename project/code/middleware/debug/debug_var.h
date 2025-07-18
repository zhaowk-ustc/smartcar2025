#ifndef DEBUG_VAR_H
#define DEBUG_VAR_H

#include <string>
#include <functional>
#include <iomanip>
#include <sstream>
#include <type_traits> // 添加类型特征支持

using namespace std;

// 调试变量结构
struct DebugVar
{
    string name;                           // 变量名
    function<string()> getter;             // 获取值
    function<void(const string &)> setter; // 设置值 - 返回void
    bool read_only;                        // 是否只读
    bool is_function = false;              // 是否为function类型
    function<void()> decrement = nullptr;  // 添加递减函数
    function<void()> increment = nullptr;  // 添加递增函数
    DebugVar() = default;

    DebugVar(string var_name,
             function<string()> get_func,
             function<void(const string &)> set_func = nullptr,
             bool readonly = false,
             bool is_func = false,
             function<void()> dec_func = nullptr,
             function<void()> inc_func = nullptr)
        : name(var_name), getter(get_func), setter(set_func), read_only(readonly), is_function(is_func), decrement(dec_func), increment(inc_func)
    {
    }

    string get() const
    {
        return getter ? getter() : "";
    }

    void set(const string &value)
    {
        if (setter && !read_only)
        {
            setter(value);
        }
    }

    // 更改变量名
    void set_name(const string &new_name)
    {
        name = new_name;
    }
};

// 便捷的变量创建函数
template <typename T>
DebugVar make_debug_var(string name, T *ptr)
{
    function<void()> inc_func = nullptr;
    function<void()> dec_func = nullptr;
    // 根据类型创建递增和递减函数
    if constexpr (is_same<T, bool>::value)
    {
        inc_func = [ptr]()
        { *ptr = true; }; // 设置为true
        dec_func = [ptr]()
        { *ptr = false; }; // 设置为false
    }
    else if constexpr (is_integral<T>::value)
    {
        inc_func = [ptr]()
        { *ptr += 1; }; // 整型加1
        dec_func = [ptr]()
        { *ptr -= 1; }; // 整型减1
    }
    else if constexpr (is_floating_point<T>::value)
    {
        inc_func = [ptr]()
        { *ptr += 0.1f; }; // 浮点加0.1
        dec_func = [ptr]()
        { *ptr -= 0.1f; }; // 浮点减0.1
    }

    return DebugVar(
        name,
        [ptr]() -> string
        {
            if constexpr (is_same<T, bool>::value)
            {
                return *ptr ? "true" : "false";
            }
            else if constexpr (is_same<T, float>::value)
            {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << *ptr;
                return oss.str();
            }
            else
            {
                return to_string(*ptr);
            }
        },
        [ptr](const string &value) -> void
        {
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
        },
        false,
        false,
        dec_func,
        inc_func);
}

template <typename T>
DebugVar make_readonly_var(string name, T *ptr)
{
    return DebugVar(
        name,
        [ptr]() -> string
        {
            if constexpr (is_same<T, bool>::value)
            {
                return *ptr ? "true" : "false";
            }
            else if constexpr (is_same<T, float>::value)
            {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << *ptr;
                return oss.str();
            }
            else
            {
                return to_string(*ptr);
            }
        },
        nullptr,
        true);
}

inline DebugVar make_function_var(string name, function<void(void)> func)
{
    return DebugVar(
        name,
        nullptr,
        [func](const string &)
        { func(); },
        false,
        true);
}

#endif // DEBUG_VAR_H