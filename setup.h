#pragma once

void setupLvgl();
void setupFilesystem();
void setupSensor();
void tft_sleep();
void tft_wakeup();
bool tryToSetupSDCard();
bool isSdMount();
void stopLvglTick();
void startLvglTick();
