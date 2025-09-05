#ifndef MQTT_PROTOCOL_H
#define MQTT_PROTOCOL_H


#include "protocol.h"
#include <mqtt.h>
#include <udp.h>
#include <cJSON.h>
#include <mbedtls/aes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <functional>
#include <string>
#include <map>
#include <mutex>

#define MQTT_PING_INTERVAL_SECONDS 90
#define MQTT_RECONNECT_INTERVAL_MS 10000

#define MQTT_PROTOCOL_SERVER_HELLO_EVENT (1 << 0)

class MqttProtocol : public Protocol {
    public:

        MqttProtocol();
        ~MqttProtocol();

        bool Start() override;
        bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
        bool OpenAudioChannel() override;
        void CloseAudioChannel() override;
        bool IsAudioChannelOpened() const override;

    protected:  // 改为 protected 而不是 private，以便protocols包内的LocalMqttProtocol 类可以访问
        EventGroupHandle_t event_group_handle_;

        std::string publish_topic_;
        std::unique_ptr<Mqtt> mqtt_;
        std::string endpoint_;
        std::mutex channel_mutex_;
        
        std::unique_ptr<Udp> udp_;
        mbedtls_aes_context aes_ctx_;
        std::string aes_nonce_;
        std::string udp_server_;
        int udp_port_;
        uint32_t local_sequence_;
        uint32_t remote_sequence_;

        bool StartMqttClient(bool report_error=false);
        bool SendText(const std::string& text) override; 
        void ParseServerHello(const cJSON* root);
        std::string DecodeHexString(const std::string& hex_string);
    
        std::string GetHelloMessage();
};
#endif // MQTT_PROTOCOL_H