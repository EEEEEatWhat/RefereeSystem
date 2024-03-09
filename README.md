# Referee_System 
## 简介
 该仓库为Robomaster（又称RM）的比赛裁判系统上位机适用的数据收发节点，基于裁判系统协议v1.61编写；
 ```bash
 提供服务端接口
 service：/RequestSerialize

 消息类型
 	uint16 cmd_id
	---
	uint16 cmd_id
	uint16 data_length
	uint8[] data_stream 
 ```
 >基于ROS2-humble环境，使用boost::asio对串口数据进行读取，通过内部解析线程解析保存数据，并提供service接口，获取字节流，该字节流严格符合结构体内部内存对齐特性，可以通过简单memcpy完成数据解析；
## 如何使用
### 服务端

---

### 客户端

---
## 继续开发



# TODO
- [ ] 固定端口号
- [ ] 初始化数据包
- [ ] 尝试使用模板重写
- [ ] 导出为插件或者库
- [ ] 清除所有todo
