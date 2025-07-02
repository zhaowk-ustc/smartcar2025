#ifndef DEBUGGABLE_H
#define DEBUGGABLE_H

#include "debug_var.h"
#include "../module_base.h"
#include <map>

// 可调试接口
class IDebuggable
{
public:
    IDebuggable() = default;
    virtual ~IDebuggable() = default;

    void add_debug_var(const string& name, const DebugVar& var)
    {
        vars_[name] = var;
    }

    // string get_debug_var(const string& name) const
    // {
    //     auto it = vars_.find(name);
    //     return (it != vars_.end() && it->second.getter) ? it->second.getter() : "";
    // }

    // void set_debug_var(const string& name, const string& value)
    // {
    //     auto it = vars_.find(name);
    //     if (it != vars_.end() && it->second.setter && !it->second.read_only)
    //     {
    //         it->second.setter(value);
    //     }
    // }

    // bool has_debug_var(const string& name) const
    // {
    //     return vars_.find(name) != vars_.end();
    // }

    // 导出功能
    void export_debug_vars(IDebuggable* target, const string& prefix)
    {
        for (auto& [name, var] : vars_)
        {
            const string exported_name = prefix + name;
            var.set_name(exported_name);
            target->add_debug_var(exported_name, var);
        }
        vars_.clear();
    }

    


protected:
    map<string, DebugVar> vars_;
    
    virtual void setup_debug_vars() {}
};

// 调试模块基类
class DebuggableModule : public Module, public IDebuggable
{
};

#endif // DEBUGGABLE_H