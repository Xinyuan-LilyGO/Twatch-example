#include "./src/mqtt/esp_mqtt.h"
#include "gui.h"

ESP_MQTT_Class mqtt;

#define MQTT_CLIENT_ID              "office_switch1"
#define MQTT_SERVER_IP              "192.168.36.86"
#define MQTT_SERVER_PORT            1883
#define MQTT_USER                   "MQTT_USER"
#define MQTT_PASSWORD               "MQTT_PASSWORD"

const char *MQTT_LIGHT1_STATE_TOPIC = "office/light1/status";
const char *MQTT_LIGHT2_STATE_TOPIC = "office/light2/status";
const char *MQTT_LIGHT3_STATE_TOPIC = "office/light3/status";
const char *MQTT_LIGHT4_STATE_TOPIC = "office/light4/status";

const  char *MQTT_SWITCH_STATUS_TOPIC = "office/switch1/status";
const  char *MQTT_SWITCH_COMMAND_TOPIC = "office/switch1/set";


static bool onoff[4];
static bool connect = false;



/************************************************************
 *          MQTT Callbacks
 ***********************************************************/
class mycallbacks : public MQTTClientCallbacks
{
    void onConnect()
    {
        connect = true;
        Serial.println("connect mqtt server");
        mqtt.subcribe(MQTT_LIGHT1_STATE_TOPIC);
        mqtt.subcribe(MQTT_LIGHT2_STATE_TOPIC);
        mqtt.subcribe(MQTT_LIGHT3_STATE_TOPIC);
        mqtt.subcribe(MQTT_LIGHT4_STATE_TOPIC);
    }
    void onDisconnect()
    {
        connect = false;
        memset(onoff, 0, sizeof(onoff) / sizeof(onoff[0]));
        Serial.println("disconnect mqtt server");
    }
    void onEvent(const char *topic, int topicLen, const char *data, int dataLen)
    {
        Serial.printf("TOPIC:%s ::: Data:%s\n", topic, data);
        uint8_t index = 0;
        if (!strcmp(topic, MQTT_LIGHT1_STATE_TOPIC)) {
            index = 0;
        } else if (!strcmp(topic, MQTT_LIGHT2_STATE_TOPIC)) {
            index = 1;
        } else if (!strcmp(topic, MQTT_LIGHT3_STATE_TOPIC)) {
            index = 2;
        } else if (!strcmp(topic, MQTT_LIGHT4_STATE_TOPIC)) {
            index = 3;
        }

        bool en = false;
        if (!strcmp(data, "ON")) {
            en = true;
        }
        onoff[index] = en;
    }
};


bool statusMQTT(uint8_t index)
{
    return onoff[index];
}

void controlMQTT(uint8_t index, bool en)
{
    char buf[256];
    if (!connect)return;
    const char *status = en ? "ON" : "OFF";
    snprintf(buf, sizeof(buf), "office/light%d/switch", index);
    mqtt.piblish(buf, status, strlen(status));
}

void setupMQTT()
{
    mqtt.init(MQTT_SERVER_IP, MQTT_SERVER_PORT, MQTT_USER, MQTT_PASSWORD, MQTT_CLIENT_ID);
    mqtt.setMqttClientCallbacks(new mycallbacks());
}

void startMQTT()
{
    mqtt.start();
}

void stopMQTT()
{
    if (!connect)return;
    mqtt.stop();
}