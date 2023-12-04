/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/

#pragma once 
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
        RefereePacket(){};
        ~RefereePacket(){};
        virtual void SolvePacket(uint8_t*,uint8_t*){
        std::cout<<"[Warning : This should not appear !]\n";
        };
    };
    
    #define GENERATEPACK(packname) 

    /** How To Use
     * 
     *  GENERATEPACK[GameStatusPacket];
     * 
     *      is the same with 
     * 
     *  class GameResultPacket : public RefereePacket {
     *  public:
     *  virtual void SolvePacket(uint8_t*,uint8_t*) override {};
     * 
    };
    */


    //0x0001 GameStatusPacket
    class GameStatusPacket : public RefereePacket { 
    public:
        virtual void SolvePacket(uint8_t*,uint8_t*) override {}; 
    };

    //0x0002 GameResultPacket
    class GameResultPacket : public RefereePacket { 
    public:
        virtual void SolvePacket(uint8_t*,uint8_t*) override {}; 
    };

    //0x0003 GameRobotHPPacket
    class GameRobotHPPacket : public RefereePacket { 
    public:
        virtual void SolvePacket(uint8_t*,uint8_t*) override {}; 
    };
    //0x0101
    //0x0102
    class ExtSupplyProjectileAction : public RefereePacket { 
    public:
        virtual void SolvePacket(uint8_t*,uint8_t*) override {}; 
    };
    //0x0104
    //0x0105
    //0x0201
    //0x0202 PowerHeatDataPacket
    class PowerHeatDataPacket : public RefereePacket { 
    public:
        virtual void SolvePacket(uint8_t*,uint8_t*) override {}; 
    };
    //0x0203
    //0x0204
    //0x0205
    //0x0206
    //0x0207
    //0x0208
    //0x0209
    //0x020A
    //0x020B
    //0x020C
    //0x020D
    //0x020E
    //0x0301
    //0x0302 CustomRobotDataPacket
    class CustomRobotDataPacket : public RefereePacket { 
    public:
        virtual void SolvePacket(uint8_t*,uint8_t*) override {}; 
    };
    //0x0303
    //0x0304
    //0x0305
    //0x0306
    //0x0307
    //0x0308

}