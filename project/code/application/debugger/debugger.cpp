#include "debugger.h"
#include "driver/serial.h"
#include <sstream>

void Debugger::init()
{
    init_serial(); // 初始化串口
}

void Debugger::update()
{
    // 处理串口输入
    string command_line = serial_read_line();
    if (!command_line.empty())
    {
        execute_command(command_line);
    }

    // 发送监视变量的当前值
    send_watch_vars();
}

void Debugger::execute_command(const string& command_line)
{
    auto cmd = parse_command(command_line);

    if (cmd.action == "set")
    {
        if (!has_debug_var(cmd.var_name))
        {
            debug_output_("Variable not found: " + cmd.var_name);
            return;
        }
        const auto& var = vars_.at(cmd.var_name);
        if (var.read_only)
        {
            debug_output_("Variable is read-only: " + cmd.var_name);
            return;
        }
        if (var.is_function)
        {
            debug_output_("Cannot set function variable, use call instead: " + cmd.var_name);
            return;
        }
        set_debug_var(cmd.var_name, cmd.value);
        debug_output_("Set " + cmd.var_name + " = " + get_debug_var(cmd.var_name));
    }
    else if (cmd.action == "get")
    {
        if (!has_debug_var(cmd.var_name))
        {
            debug_output_("Variable not found: " + cmd.var_name);
            return;
        }
        const auto& var = vars_.at(cmd.var_name);
        if (var.is_function)
        {
            debug_output_("Cannot get value of function variable: " + cmd.var_name);
            return;
        }
        string value = get_debug_var(cmd.var_name);
        debug_output_(cmd.var_name + " = " + value);
    }
    else if (cmd.action == "call")
    {
        if (!has_debug_var(cmd.var_name))
        {
            debug_output_("Variable not found: " + cmd.var_name);
            return;
        }
        auto& var = vars_.at(cmd.var_name);
        if (!var.is_function)
        {
            debug_output_("Variable is not a function: " + cmd.var_name);
            return;
        }
        // 调用 function 变量
        var.set("");
        debug_output_("Function called: " + cmd.var_name);
    }
    else if (cmd.action == "help")
    {
        print_help();
    }
    else if (cmd.action == "list")
    {
        debug_output_("=== List of Debug Variables ===");
        for (const auto& [name, var] : vars_)
        {
            if (var.is_function)
            {
                debug_output_(name + " = [function]");
            }
            else
            {
                std::string line = name + " = " + var.get();
                if (var.read_only) line += " [readonly]";
                debug_output_(line);
            }
        }
        debug_output_("===============================");
    }
    else if (cmd.action == "watch")
    {
        if (!cmd.var_name.empty())
        {
            if (!has_debug_var(cmd.var_name))
            {
                debug_output_("Cannot watch: Variable not found: " + cmd.var_name);
            }
            else if (vars_.at(cmd.var_name).is_function)
            {
                debug_output_("Cannot watch function variable: " + cmd.var_name);
            }
            else
            {
                add_watch_var(cmd.var_name);
                debug_output_("Added watch for: " + cmd.var_name);
            }
        }
        else
        {
            debug_output_("Usage: watch <var_name>");
        }
    }
    else if (cmd.action == "unwatch")
    {
        if (!cmd.var_name.empty())
        {
            remove_watch_var(cmd.var_name);
            debug_output_("Removed watch for: " + cmd.var_name);
        }
        else
        {
            debug_output_("Usage: unwatch <var_name>");
        }
    }
    else if (cmd.action == "unwatch")
    {
        if (!cmd.var_name.empty())
        {
            remove_watch_var(cmd.var_name);
            debug_output_("Removed watch for: " + cmd.var_name);
        }
        else
        {
            debug_output_("Usage: unwatch <var_name>");
        }
    }
    else
    {
        debug_output_("Unknown command: " + command_line);
    }
}

void Debugger::print_help() const
{
    debug_output_("=== Debug Commands ===");
    debug_output_("set <var_name> <value>  - Set variable value");
    debug_output_("get <var_name>          - Get variable value");
    debug_output_("help                    - Show this help");
    debug_output_("call <var_name>         - Call function variable");
    debug_output_("watch <var_name>        - Watch variable");
    debug_output_("unwatch <var_name>      - Unwatch variable");
    debug_output_("list                    - List all debug variables");
    debug_output_("\n");
    debug_output_("Examples:");
    debug_output_("  set spid.kp 50");
    debug_output_("  get target_speed");
    debug_output_("  watch current_speed");
    debug_output_("  unwatch current_speed");
    debug_output_("  list");
    debug_output_("======================");
}

Debugger::Command Debugger::parse_command(const string& command_line) const
{
    Command cmd;
    istringstream iss(command_line);
    iss >> cmd.action >> cmd.var_name >> cmd.value;
    return cmd;
}

void Debugger::set_debug_var(const string& var_name, const string& value)
{
    if (has_debug_var(var_name))
    {
        vars_.at(var_name).set(value);
    }
    else
    {
        debug_output_("Variable not found: " + var_name);
    }
}

string Debugger::get_debug_var(const string& var_name) const
{
    if (has_debug_var(var_name))
    {
        return vars_.at(var_name).get();
    }
    else
    {
        debug_output_("Variable not found: " + var_name);
        return "";
    }
}

bool Debugger::has_debug_var(const string& var_name) const
{
    return vars_.find(var_name) != vars_.end();
}

// 添加监视变量
void Debugger::add_watch_var(const std::string& var_name)
{
    if (has_debug_var(var_name))
        watch_vars_.insert(var_name);
    else
        debug_output_("Cannot watch: Variable not found: " + var_name);
}

// 移除监视变量
void Debugger::remove_watch_var(const std::string& var_name)
{
    watch_vars_.erase(var_name);
}

// 发送所有监视变量的当前值
void Debugger::send_watch_vars() const
{
    if (watch_vars_.empty()) return;
    std::string line = "[WATCH]";
    for (const auto& name : watch_vars_)
    {
        line += name + "=" + vars_.at(name).get() + "\t";
    }
    line.pop_back();  // 移除最后一个制表符
    debug_output_(line);
}

vector<const DebugVar*> Debugger::list_var_ptrs() const
{
    vector<const DebugVar*> ptrs;
    for (const auto& [name, var] : vars_)
    {
        ptrs.push_back(&var);
    }
    return ptrs;
}