#include "esp_mqtt.h"

#define MQTT_TAG "[MQTT]"

ESP_MQTT_Class *mqttClient = nullptr;

ESP_MQTT_Class::ESP_MQTT_Class()
{
    _handle = NULL;
    _clientCallbacks = nullptr;
    memset(&_config, 0, sizeof(_config));
    mqttClient = this;
}

ESP_MQTT_Class::~ESP_MQTT_Class()
{
    if (_handle) {
        esp_mqtt_client_stop(_handle);
        esp_mqtt_client_destroy(_handle);
        _handle = NULL;
    }
}

esp_err_t ESP_MQTT_Class::_mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int32_t msg_id;
    // your_context_t *context = event->context;

    switch (event->event_id) {
        case MQTT_EVENT_BEFORE_CONNECT:
        break;
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
        if (mqttClient->_clientCallbacks != nullptr) {
            mqttClient->_clientCallbacks->onConnect();
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
        if (mqttClient->_clientCallbacks != nullptr) {
            mqttClient->_clientCallbacks->onDisconnect();
        }
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%x\n", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%x\n", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%x\n", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");

        if (mqttClient->_clientCallbacks != nullptr) {
            String topic = "";
            String payload = "";
            for (int i = 0; i < event->topic_len; i++) {
                topic.concat((char)event->topic[i]);
            }
            for (int i = 0; i < event->data_len; i++) {
                payload.concat((char)event->data[i]);
            }
            mqttClient->_clientCallbacks->onEvent(topic.c_str(), event->topic_len, payload.c_str(), event->data_len);
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_ERROR");
        break;
    }
    return 0;
}

int ESP_MQTT_Class::init(const char *host, uint32_t port, const char *username, const char *password, const char *clientId)
{
    _config.host = host;
    _config.port = port;
    _config.client_id = clientId;
    _config.username = username;
    _config.password = password;
    _config.event_handle = &ESP_MQTT_Class::_mqtt_event_handler;
    _handle = esp_mqtt_client_init(&_config);
    return 0;
}

int ESP_MQTT_Class::start()
{
    // _handle = esp_mqtt_client_init(&_config);
    return esp_mqtt_client_start(_handle);
}

void ESP_MQTT_Class::stop()
{
    esp_mqtt_client_stop(_handle);
}

void ESP_MQTT_Class::deinit()
{
    if (_handle) {
        esp_mqtt_client_stop(_handle);
        esp_mqtt_client_destroy(_handle);
        _handle = NULL;
    }
}

int ESP_MQTT_Class::_setconfig()
{
    if (_handle) {
        return esp_mqtt_set_config(_handle, &_config);
    }
    return 0;
}

int ESP_MQTT_Class::setKeepAlive(int seconds)
{
    _config.keepalive = seconds;
    return _setconfig();
}

int ESP_MQTT_Class::setClientKepPem(const char *keyPem)
{
    _config.client_key_pem = keyPem;
    return _setconfig();
}

int ESP_MQTT_Class::setClientCerPem(const char *certPem)
{
    _config.client_cert_pem = certPem;
    return _setconfig();
}

int ESP_MQTT_Class::setCertPem(const char *certPem)
{
    _config.cert_pem = certPem;
    return _setconfig();
}

int ESP_MQTT_Class::autoReconnect(bool en)
{
    _config.disable_auto_reconnect = en;
    return _setconfig();
}

int ESP_MQTT_Class::setBufferSize(int size)
{
    _config.buffer_size = size;
    return _setconfig();
}

int ESP_MQTT_Class::setUrl(const char *url)
{
    _config.uri = url;
    return _setconfig();
}

int ESP_MQTT_Class::setHost(const char *host)
{
    _config.host = host;
    return _setconfig();
}

int ESP_MQTT_Class::setPort(int port)
{
    _config.port = port;
    return _setconfig();
}

int ESP_MQTT_Class::setUserName(const char *username)
{
    _config.username = username;
    return _setconfig();
}

int ESP_MQTT_Class::setPassword(const char *password)
{
    _config.password = password;
    return _setconfig();
}


void ESP_MQTT_Class::setMqttClientCallbacks(MQTTClientCallbacks *cb)
{
    _clientCallbacks = cb;
}

int ESP_MQTT_Class::subcribe(const char *topic, int qos )
{
    return esp_mqtt_client_subscribe(_handle, topic, qos);
}

int ESP_MQTT_Class::unsubcribe(const char *topic)
{
    return esp_mqtt_client_unsubscribe(_handle, topic);
}

int ESP_MQTT_Class::piblish(const char *topic, const char *data, int len, int qos, int retain)
{
    return esp_mqtt_client_publish( _handle, topic, data,  len,  qos,  retain);
}