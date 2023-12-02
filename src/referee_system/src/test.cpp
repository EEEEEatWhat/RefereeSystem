#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include "DataType.h"
#include "MappingTables.h"




int main(int argc, char const *argv[])
{
    boost::asio::io_service ioService;
    boost::asio::serial_port serialPort(ioService, "/dev/ttyUSB0");
    std::vector<uint8_t> buffer(4096);  // 适当调整缓冲区大小
    RM_referee::TypeMethodsTables Factory_;
    Factory_.AddTypeMethod<RM_referee::GameStatusPacket>(0x0001);
    Factory_.SerialReadAsync(serialPort,buffer);
    return 0;
}






