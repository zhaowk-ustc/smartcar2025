#ifndef SERIAL_H
#define SERIAL_H

#include <string>
using namespace std;

string serial_read_line();
void serial_write_line(const string& data);
void init_serial();

#endif // SERIAL_H