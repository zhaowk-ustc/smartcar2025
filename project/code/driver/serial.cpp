#include "serial.h"
#include "zf_common_headfile.h"

std::string serial_read_line()
{
    constexpr uint32 MAX_LINE_LEN = 128;
    static std::string wireless_buffer;
    static std::string debug_buffer;

    uint8 wireless_buff[MAX_LINE_LEN] = { 0 };
    uint8 debug_buff[MAX_LINE_LEN] = { 0 };

    // 读取无线串口
    uint32 len = wireless_uart_read_buffer(wireless_buff, MAX_LINE_LEN);
    if (len > 0)
        wireless_buffer.append(reinterpret_cast<char*>(wireless_buff), len);

    // 读取debug串口
    uint32 debug_len = debug_read_ring_buffer(debug_buff, MAX_LINE_LEN);
    if (debug_len > 0)
        debug_buffer.append(reinterpret_cast<char*>(debug_buff), debug_len);

    // 查找无线串口缓冲区的换行
    size_t pos = wireless_buffer.find_first_of("\r\n");
    if (pos != std::string::npos)
    {
        std::string line = wireless_buffer.substr(0, pos);
        size_t next_pos = wireless_buffer.find_first_not_of("\r\n", pos);
        if (next_pos == std::string::npos)
            wireless_buffer.clear();
        else
            wireless_buffer = wireless_buffer.substr(next_pos);
        return line;
    }

    // 查找debug串口缓冲区的换行
    pos = debug_buffer.find_first_of("\r\n");
    if (pos != std::string::npos)
    {
        std::string line = debug_buffer.substr(1, pos);
        size_t next_pos = debug_buffer.find_first_not_of("\r\n", pos);
        if (next_pos == std::string::npos)
            debug_buffer.clear();
        else
            debug_buffer = debug_buffer.substr(next_pos);
        return line;
    }

    return "";
}

void serial_write_line(const std::string& data)
{
    // 发送一行数据到无线串口
    wireless_uart_send_string((data + "\n").c_str());

    // 发送一行数据到debug串口
    debug_send_buffer(reinterpret_cast<const uint8*>((data + "\n").c_str()), data.size() + 1);

}

void init_serial()
{
    wireless_uart_init();
}
