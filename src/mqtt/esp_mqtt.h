#pragma once
#include <Arduino.h>
#include <mqtt_client.h>

class MQTTClientCallbacks
{
public:
    virtual ~MQTTClientCallbacks() {};
    virtual void onConnect();
    virtual void onDisconnect();
    virtual void onEvent(const char *topic, int topicLen, const char *data, int dataLen);
};


// typedef void (*event_callback)(const char *topic, int topicLen, const char *data, int dataLen);

class ESP_MQTT_Class
{
public:
    ESP_MQTT_Class();
    ~ESP_MQTT_Class();
    int init(const char *host, uint32_t port, const char *username = NULL, const char *password = NULL, const char *clientId = NULL);
    int start();
    void stop();
    int setKeepAlive(int seconds);
    int setClientKepPem(const char *keyPem);
    int setClientCerPem(const char *certPem);
    int setCertPem(const char *certPem);
    int autoReconnect(bool en);
    int setBufferSize(int size);
    int setUrl(const char *url);
    int setHost(const char *host);
    int setPort(int port);
    int setUserName(const char *username);
    int setPassword(const char *password);

    void deinit();

    int subcribe(const char *topic, int qos = 0);
    int unsubcribe(const char *topic);
    int piblish(const char *topic, const char *data, int len, int qos = 0, int retain = 0);

    // void registerForMqttEvent(event_callback cb);
    void setMqttClientCallbacks(MQTTClientCallbacks *cb);

private:
    int _setconfig();
    esp_mqtt_client_config_t _config;
    esp_mqtt_client_handle_t _handle;
    MQTTClientCallbacks *_clientCallbacks;
protected:
    static esp_err_t _mqtt_event_handler(esp_mqtt_event_handle_t event);

};