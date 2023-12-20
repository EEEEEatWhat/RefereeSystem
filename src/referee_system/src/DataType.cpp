#include "DataType.h"
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace RM_referee{
//TODO 添加提升复用性的宏
    uint16_t PowerHeatDataPacket::SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size)  {
        if(cmd_id != GetID())
            std::cout<<"SolveMethod does not match ID !\n";
        std::memcpy(&m_value,data,data_size);
        std::cout<<std::hex<<"0x"<<cmd_id<<"\n"<<std::dec;
        std::cout
                <<"chassis电压(mV):"<<m_value.chassis_voltage<<"\n"
                <<"chassis电流(mA):"<<m_value.chassis_current<<"\n"
                <<"底盘功率(W):"<<m_value.chassis_power<<"\n"
                <<"缓冲能量(J):"<<m_value.buffer_energy<<"\n"
                <<"第1个17mm枪口热量:"<<m_value.shooter_17mm_1_barrel_heat<<"\n"
                <<"第2个17mm枪口热量:"<<m_value.shooter_17mm_2_barrel_heat<<"\n"
                <<"42mm枪口热量:"<<m_value.shooter_42mm_barrel_heat<<"\n";
        return DateLength();
    };

    uint16_t CustomRobotDataPacket::SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size)  {
        if(cmd_id != GetID())
            std::cout<<"SolveMethod does not match ID !\n";
        std::memcpy(&m_value,data,data_size);
    return DateLength();
    };

    uint16_t ExtSupplyProjectile::SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size)  {
        if(cmd_id != GetID())
            std::cout<<"SolveMethod does not match ID !\n";
        std::memcpy(&m_value,data,data_size);//BUG：data_size准确性？CRC8 may be useful。
        std::cout<<std::hex<<"0x"<<cmd_id<<"\n"<<std::dec;
        /*  采样数据版本不对!
        */
        // std::cout
        //         <<""<<m_value.reserved <<"\n"
        //         <<""<<m_value.supply_projectile_num <<"\n"
        //         <<""<<m_value.supply_projectile_step <<"\n"
        //         <<""<<m_value.supply_robot_id <<"\n";
    return DateLength();
    };
}