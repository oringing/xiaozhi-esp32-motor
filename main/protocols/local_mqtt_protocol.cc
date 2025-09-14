#include "board.h"
#include "settings.h"
#include "esp_log.h"
#include "cJSON.h"
#include "local_mqtt_protocol.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "application.h"

#include <sstream>
#include <chrono>

static const char* TAG = "LocalMqtt";
static const char* CACHE_NAMESPACE = "chat_cache";
static const int MAX_CACHE_SIZE = 100;

LocalMqttProtocol::LocalMqttProtocol() {
    client_id_ = GenerateDefaultClientId();
}

LocalMqttProtocol::~LocalMqttProtocol() {
    // 析构时尝试上传剩余缓存
    UploadCachedRecords();
}

bool LocalMqttProtocol::Initialize() {
    // 从设置中加载配置，若无则使用默认值
    Settings settings("local_mqtt", true);
    // 临时添加这一行来清除旧配置
    settings.EraseKey("endpoint");
    
    endpoint_ = settings.GetString("endpoint", default_endpoint_);
    username_ = settings.GetString("username", default_username_);
    password_ = settings.GetString("password", default_password_);
    client_id_ = settings.GetString("client_id", client_id_);
    keepalive_interval_ = settings.GetInt("keepalive", 240);
    
    // 保存配置到非易失性存储
    settings.SetString("endpoint", endpoint_);
    settings.SetString("username", username_);
    settings.SetString("password", password_);
    settings.SetString("client_id", client_id_);
    settings.SetInt("keepalive", keepalive_interval_);
    settings.SetString("initialized", "true");
    
    // 构建聊天主题
    chat_topic_ = "xiaozhi/chat/" + client_id_;
    
    ESP_LOGI(TAG, "Local MQTT initialized with endpoint: %s, client_id: %s", 
             endpoint_.c_str(), client_id_.c_str());
    
    return true;
}

bool LocalMqttProtocol::Start() {
    ESP_LOGI(TAG, "Scheduling local MQTT client connection");
    
    // 使用Application的调度机制在正确的线程中执行连接
    Application::GetInstance().Schedule([this]() {
        ConnectTask();
    });
    
    return true;
}

void LocalMqttProtocol::ConnectTask() {
    ESP_LOGI(TAG, "Starting local MQTT client connection task");
    
    if (endpoint_.empty()) {
        ESP_LOGW(TAG, "Local MQTT endpoint is not specified");
        return;
    }

    auto network = Board::GetInstance().GetNetwork();
    // 创建MQTT客户端，使用不同的ID避免冲突
    mqtt_ = network->CreateMqtt(1);
    mqtt_->SetKeepAlive(keepalive_interval_);

    mqtt_->OnDisconnected([this]() {
        ESP_LOGI(TAG, "Disconnected from local MQTT endpoint");
        connected_ = false;
    });

    mqtt_->OnMessage([this](const std::string& topic, const std::string& payload) {
        ESP_LOGI(TAG, "Received message on topic %s: %s", topic.c_str(), payload.c_str());
        // 本地MQTT客户端主要用于发送，不需要处理消息
    });

    ESP_LOGI(TAG, "Connecting to local MQTT endpoint %s", endpoint_.c_str());
    std::string broker_address;
    int broker_port = 1883;
    size_t pos = endpoint_.find(':');
    if (pos != std::string::npos) {
        broker_address = endpoint_.substr(0, pos);
        broker_port = std::stoi(endpoint_.substr(pos + 1));
    } else {
        broker_address = endpoint_;
    }
    
    if (!mqtt_->Connect(broker_address, broker_port, client_id_, username_, password_)) {
        ESP_LOGE(TAG, "Failed to connect to local MQTT endpoint");
        connected_ = false;
        return;
    }

    ESP_LOGI(TAG, "Connected to local MQTT endpoint");
    connected_ = true;
    
    // 发送测试消息
    SendHelloTest();
    
    // 上传缓存的记录
    UploadCachedRecords();
}

bool LocalMqttProtocol::IsConnected() const {
    return connected_ && mqtt_ && mqtt_->IsConnected();
}

std::string LocalMqttProtocol::GenerateDefaultClientId() {
    uint8_t mac[6];
    // 使用正确的宏定义
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    char client_id[20];
    snprintf(client_id, sizeof(client_id), "esp32_%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(client_id);
}

bool LocalMqttProtocol::SendText(const std::string& text) {
    if (!IsConnected() || chat_topic_.empty()) {
        ESP_LOGW(TAG, "Not connected or topic not set, cannot send message");
        return false;
    }
    
    if (!mqtt_->Publish(chat_topic_, text)) {
        ESP_LOGE(TAG, "Failed to publish message: %s", text.c_str());
        connected_ = false; // 标记为断开连接
        return false;
    }
    
    return true;
}

bool LocalMqttProtocol::SendHelloTest() {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "hello_test");
    cJSON_AddStringToObject(root, "device_id", client_id_.c_str());
    cJSON_AddNumberToObject(root, "timestamp", 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    
    char* json_str = cJSON_PrintUnformatted(root);
    std::string message(json_str);
    cJSON_free(json_str);
    cJSON_Delete(root);
    
    ESP_LOGI(TAG, "Sending hello test: %s", message.c_str());
    return SendText(message);
}

bool LocalMqttProtocol::SendGoodbyeTest() {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "goodbye_test");
    cJSON_AddStringToObject(root, "device_id", client_id_.c_str());
    cJSON_AddNumberToObject(root, "timestamp", 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    
    char* json_str = cJSON_PrintUnformatted(root);
    std::string message(json_str);
    cJSON_free(json_str);
    cJSON_Delete(root);
    
    ESP_LOGI(TAG, "Sending goodbye test: %s", message.c_str());
    return SendText(message);
}

bool LocalMqttProtocol::UploadChatRecord(const ChatRecord& record) {
    // 创建聊天记录JSON
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "chat_record");
    cJSON_AddStringToObject(root, "device_id", record.device_id.c_str());
    cJSON_AddNumberToObject(root, "timestamp", record.timestamp);
    cJSON_AddStringToObject(root, "user_query", record.user_query.c_str());
    cJSON_AddStringToObject(root, "device_response", record.device_response.c_str());
    cJSON_AddNumberToObject(root, "status", record.status);
    
    char* json_str = cJSON_PrintUnformatted(root);
    std::string message(json_str);
    cJSON_free(json_str);
    cJSON_Delete(root);
    
    // 尝试直接发送
    bool sent = SendText(message);
    
    // 如果发送失败，保存到缓存
    if (!sent) {
        ESP_LOGW(TAG, "Failed to send chat record, saving to cache");
        return SaveToCache(message);
    }
    
    ESP_LOGI(TAG, "Successfully uploaded chat record");
    return true;
}

bool LocalMqttProtocol::SaveToCache(const std::string& json_data) {
    Settings cache(CACHE_NAMESPACE, true);
    
    // 获取当前缓存数量
    int count = cache.GetInt("count", 0);
    
    // 如果达到最大缓存，循环覆盖最旧的
    if (count >= MAX_CACHE_SIZE) {
        count = MAX_CACHE_SIZE - 1;
    }
    
    // 保存新记录
    std::string key = "record_" + std::to_string(count);
    cache.SetString(key.c_str(), json_data);
    
    // 只有在设置字符串成功的情况下才增加计数
    // 避免计数错误导致后续读取问题
    cache.SetInt("count", count + 1);
    
    ESP_LOGI(TAG, "Saved record to cache, key: %s", key.c_str());
    return true;
}

std::vector<std::string> LocalMqttProtocol::LoadFromCache() {
    Settings cache(CACHE_NAMESPACE, false);
    std::vector<std::string> records;
    
    int count = cache.GetInt("count", 0);
    for (int i = 0; i < count; i++) {
        std::string key = "record_" + std::to_string(i);
        std::string record = cache.GetString(key.c_str(), "");
        if (!record.empty()) {
            records.push_back(record);
        }
    }
    
    ESP_LOGI(TAG, "Loaded %d records from cache", (int)records.size());
    return records;
}

bool LocalMqttProtocol::ClearAllCachedRecords() {
    Settings cache(CACHE_NAMESPACE, true);
    
    int count = cache.GetInt("count", 0);
    
    // 清除所有记录
    for (int i = 0; i < count; i++) {
        std::string key = "record_" + std::to_string(i);
        cache.EraseKey(key.c_str());
    }
    
    cache.SetInt("count", 0);
    ESP_LOGI(TAG, "Cleared all cached records");
    return true;
}
bool LocalMqttProtocol::ClearCachedRecords(const std::vector<int>& indices) {
    Settings cache(CACHE_NAMESPACE, true);
    int count = cache.GetInt("count", 0);
    
    // 标记要删除的记录
    std::vector<bool> to_delete(count, false);
    for (int idx : indices) {
        if (idx >= 0 && idx < count) {
            to_delete[idx] = true;
        }
    }
    
    // 重新保存剩余记录
    int new_count = 0;
    for (int i = 0; i < count; i++) {
        if (!to_delete[i]) {
            std::string old_key = "record_" + std::to_string(i);
            std::string new_key = "record_" + std::to_string(new_count);
            std::string value = cache.GetString(old_key.c_str(), "");
            
            if (!value.empty()) {
                cache.SetString(new_key.c_str(), value);
                new_count++;
            }
            
            // 清除旧记录
            cache.EraseKey(old_key.c_str());
        }
    }
    
    cache.SetInt("count", new_count);
    ESP_LOGI(TAG, "Cleared %d cached records, %d remaining", (int)indices.size(), new_count);
    return true;
}

bool LocalMqttProtocol::UploadCachedRecords() {
    if (!IsConnected()) {
        ESP_LOGW(TAG, "Not connected, skipping cache upload");
        return false;
    }
    
    std::vector<std::string> records = LoadFromCache();
    if (records.empty()) {
        ESP_LOGI(TAG, "No cached records to upload");
        return true;
    }
    
    ESP_LOGI(TAG, "Uploading %d cached records", (int)records.size());
    
    std::vector<int> successful_indices;
    
    for (int i = 0; i < records.size(); i++) {
        if (SendText(records[i])) {
            successful_indices.push_back(i);
            ESP_LOGI(TAG, "Successfully uploaded cached record %d", i);
        } else {
            ESP_LOGE(TAG, "Failed to upload cached record %d", i);
            break; // 如果发送失败，停止上传
        }
    }
    
    // 清除成功上传的记录
    if (!successful_indices.empty()) {
        ClearCachedRecords(successful_indices);
    }
    
    return true;
}