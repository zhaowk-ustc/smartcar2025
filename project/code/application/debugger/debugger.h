#ifndef DEBUGGER_H
#define DEBUGGER_H

#include "zf_common_headfile.h"
#include "middleware/debug/debuggable.h"
#include "driver/serial.h"
#include <set>
#include <vector>
using namespace std;

class Debugger : public IDebuggable
{
public:
    Debugger() = default;

    void init();

    void update();

    // 命令处理接口
    void execute_command(const string& command_line);

    // 帮助信息
    void print_help() const;

    // 注册输出函数
    void register_debug_output_function(void (*func)(const string&))
    {
        debug_output_ = func;
    }

    void send_watch_vars() const;

    string get_debug_var(const string& var_name) const;
    void set_debug_var(const string& var_name, const string& value);

    vector<const DebugVar*> list_var_ptrs() const;

private:
    struct Command
    {
        string action;
        string var_name;
        string value;
    };

    Command parse_command(const string& command_line) const;

    bool has_debug_var(const string& var_name) const;

    set<string> watch_vars_; // 监视变量集合
    void add_watch_var(const string& var_name);
    void remove_watch_var(const string& var_name);

    // 输出函数指针
    void (*debug_output_)(const string&) = serial_write_line;
};

#endif // DEBUGGER_H