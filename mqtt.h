#ifndef __MQTT_H
#define __MQTT_H


void setupMQTT();
void stopMQTT();
void startMQTT();
void controlMQTT(uint8_t index, bool en);
bool statusMQTT(uint8_t index);
#endif