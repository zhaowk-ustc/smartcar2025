#include "serial.h"
#include "zf_common_headfile.h"

std::string serial_read_line()
{
    constexpr uint32 MAX_LINE_LEN = 128;
    static std::string buffer; // 静态缓冲区

    uint8 buff[MAX_LINE_LEN] = { 0 };
    uint32 len = wireless_uart_read_buffer(buff, MAX_LINE_LEN);

    if (len > 0)
    {
        buffer.append(reinterpret_cast<char*>(buff), len);
    }

    // 查找换行符
    size_t pos = buffer.find_first_of("\r\n");
    if (pos != std::string::npos)
    {
        std::string line = buffer.substr(0, pos);
        // 移除已读内容和后续的所有换行符
        size_t next_pos = buffer.find_first_not_of("\r\n", pos);
        if (next_pos == std::string::npos)
            buffer.clear();
        else
            buffer = buffer.substr(next_pos);
        return line;
    }

    return ""; // 没有完整一行
}

void serial_write_line(const std::string& data)
{
    // 发送一行数据（可自动补\n）
    wireless_uart_send_string((data + "\n").c_str());
}

void init_serial()
{
    // 初始化串口
    wireless_uart_init();
    
}
