// #include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstddef>
#include <iostream>
namespace RM_referee{
    // Packet header structure
    #pragma pack(1)
    struct PacketHeader {
        uint8_t SOF;
        uint16_t DataLength;
        uint8_t SequenceNumber;
        uint8_t CRC8;
    } ;
    #pragma pack()
    static_assert(sizeof(PacketHeader) == 5, "PacketHeader must be 5 bytes long with packing");
    
    static constexpr uint8_t StartOfFrame = 0xa5;

    // Base packet
    class RefereePacket {
    public:
        virtual void SolvePacket(uint8_t*,uint8_t*){
        std::cout<<"[Warning : This should not appear !]\n";
        };
    };

    class GameStatusPacket : public RefereePacket {
    public:
        virtual void SolvePacket(uint8_t*,uint8_t*) override ;
    };
}