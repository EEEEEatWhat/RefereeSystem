#include <unordered_map>
#include <functional>
#include <memory>
#include "DataType.h"
    /**
        @brief 通过映射表来解析数据
        @param 映射表接受cmd_id，对应DJI裁判系统附录
        @param 解析包函数，通过继承数据包基类并重写Solve函数提供解包方法
        @date  2023/11/27 基于DJI裁判系统协议V1.5'
        @return NULL
    */
namespace RM_referee {
    using std::hex;

    class TypeMethodsTables {
    private:
        std::unordered_map< uint16_t, std::shared_ptr<GameStatusPacket>> m_map;
    public:
        TypeMethodsTables();
        ~TypeMethodsTables();

        template <typename Type>
        void AddTypeMethod(uint16_t);
        void MapSolve(const uint16_t, uint8_t *, uint8_t *);
    };

    TypeMethodsTables::TypeMethodsTables(){}
    TypeMethodsTables::~TypeMethodsTables() {}
    /**
     * @brief  向工厂添加ID和绑定的类
     * @param  Type 类型名
     * @param  cmd_id 
     * @return NONE
    */
    template <typename Type>
    void TypeMethodsTables::AddTypeMethod(uint16_t cmd_id) {
        m_map.emplace(cmd_id, std::make_shared<Type>());
    }

    /// @brief 通过map键值对解包
    /// @brief 查表效率可能没有switch高，后续看情况选择
    /// @param cmd_id 键
    /// @param data 待解包数据
    /// @param buf 解包后存放位置
    /// @warning 注意内存大小不要越界访问
    void TypeMethodsTables::MapSolve(const uint16_t cmd_id , uint8_t* data ,uint8_t* buf){
        auto it = m_map.find(cmd_id);
        it->second->SolvePacket(data ,buf );
    }

}  // namespace RM_referee


