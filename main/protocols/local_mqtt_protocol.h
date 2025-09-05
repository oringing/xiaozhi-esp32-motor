#ifndef LOCAL_MQTT_PROTOCOL_H
#define LOCAL_MQTT_PROTOCOL_H

#include "mqtt_protocol.h"
#include <string>
#include <vector>
#include <esp_system.h>

// 聊天记录结构体
struct ChatRecord {
    std::string device_id;
    uint64_t timestamp;
    std::string user_query;
    std::string device_response;
    int status; // 1-成功, 0-失败
};

class LocalMqttProtocol : public MqttProtocol {
public:
    LocalMqttProtocol();
    ~LocalMqttProtocol() override;

    // 初始化本地MQTT配置
    bool Initialize();
    
    // 启动本地MQTT客户端连接（异步）
    bool Start() override;
    
    // 检查是否已连接
    bool IsConnected() const;
    
    // 发送Hello测试消息
    bool SendHelloTest();
    
    // 发送Goodbye测试消息
    bool SendGoodbyeTest();
    
    // 上传聊天记录
    bool UploadChatRecord(const ChatRecord& record);
    
    // 从本地缓存上传所有未发送的记录
    bool UploadCachedRecords();
    
    // 获取客户端ID
    const std::string& GetClientId() const { return client_id_; }

    bool SendText(const std::string& text);  // 添加声明

private:
    // 默认EMQX服务器配置
    std::string default_endpoint_ = "192.168.3.11:1883";
    std::string default_username_ = "xiaozhi";
    std::string default_password_ = "123456";
    std::string client_id_;
    
    // 聊天记录发布主题
    std::string chat_topic_;
    
    // 连接状态
    bool connected_ = false;
    std::string endpoint_;
    std::string username_;
    std::string password_;
    int keepalive_interval_ = 240;
    
    // 生成默认客户端ID (基于设备唯一标识)
    std::string GenerateDefaultClientId();
    
    // 异步连接任务
    void ConnectTask();
    
    // 保存记录到本地缓存
    bool SaveToCache(const std::string& json_data);
    
    // 从缓存加载记录
    std::vector<std::string> LoadFromCache();
    
    // 清除已上传的缓存记录
    bool ClearCachedRecords(const std::vector<int>& indices);
};

#endif // LOCAL_MQTT_PROTOCOL_H