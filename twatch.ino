#include "./src/lvgl/lvgl.h"
#include "./src/drive/tft/tft.h"
#include "./src/drive/tft/bl.h"
#include "./src/board_def.h"
#include "./src/drive/i2c/i2c_bus.h"
#include "./src/drive/bma423/bma.h"
#include "./src/drive/fx50xx/FT5206.h"
#include "./src/drive/axp/src/axp20x.h"
#include "./src/drive/rtc/pcf8563.h"
#include "./src/Button2/src/Button2.h"

// #include "./src/mqtt/esp_mqtt.h"
#include "gui.h"
#include "mqtt.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include <soc/rtc.h>
#include "esp_wifi.h"

#include "setup.h"
#include "def.h"
#include <WiFi.h>
#include "bluetooth.h"

#define DEFAULT_SCREEN_TIMEOUT  30*1000

#define WATCH_FLAG_SLEEP_MODE   _BV(1)
#define WATCH_FLAG_SLEEP_EXIT   _BV(2)
#define WATCH_FLAG_BMA_IRQ      _BV(3)
#define WATCH_FLAG_AXP_IRQ      _BV(4)

BackLight bl(TFT_BL);
Motor motor(MOTOR_PIN);
I2CBus i2c(Wire1);
BMA bma(i2c);
// AXP20X_Class axp(i2c);
AXP20X_Class axp;
PCF8563_Class rtc(i2c);
Button2 btn(USER_BUTTON);
// ESP_MQTT_Class mqtt;


static bool esp_sleep = false;

QueueHandle_t g_event_queue_handle = NULL;
EventGroupHandle_t g_event_group = NULL;
EventGroupHandle_t isr_group = NULL;

void per_task(void *);



void setupNetwork()
{
    // WiFi.mode(WIFI_OFF); //! error
    WiFi.mode(WIFI_STA);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        Serial.print("WiFi lost connection. Reason: ");
        Serial.println(info.disconnected.reason);
        xEventGroupClearBits(g_event_group, G_EVENT_WIFI_CONNECTED);
        stopMQTT();
    }, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        Serial.println("Completed scan for access points");
        uint8_t data = Q_EVENT_WIFI_SCAN_DONE;
        xQueueSend(g_event_queue_handle, &data, portMAX_DELAY);
        // int16_t len =  WiFi.scanComplete();
        // Serial.printf("Scan WiFi %d count\n", len);
        // for (int i = 0; i < len; ++i) {
        //     wifi_list_add(WiFi.SSID(i).c_str());
        // }
        // WiFi.scanDelete();
    }, WiFiEvent_t::SYSTEM_EVENT_SCAN_DONE);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        Serial.println("Connected to access point");
        xEventGroupSetBits(g_event_group, G_EVENT_WIFI_CONNECTED);
    }, WiFiEvent_t::SYSTEM_EVENT_STA_CONNECTED);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        Serial.print("Obtained IP address: ");
        Serial.println(WiFi.localIP());
        wifi_connect_status(true);
        startMQTT();
    }, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);
}


uint8_t readBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    i2c.readBytes(addr, reg, data, len);
    return 0;
}

uint8_t writeBytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    i2c.writeBytes(addr, reg, data, len);
    return 0;
}

void setupPeripheral()
{
    i2c.scan();

    axp.begin(readBytes, writeBytes);

    axp.enableIRQ(AXP202_ALL_IRQ, AXP202_OFF);

    axp.setShutdownTime(AXP_POWER_OFF_TIME_4S);

    axp.adc1Enable(0xFF, AXP202_OFF);

    axp.adc2Enable(0xFF, AXP202_OFF);

    axp.adc1Enable(AXP202_BATT_VOL_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1, AXP202_ON);

    axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_CHARGING_FINISHED_IRQ, AXP202_ON);

    axp.clearIRQ();

    axp.setPowerOutPut(AXP202_LDO2, AXP202_ON);

    bl.begin();

    motor.begin();

    bl.adjust(255);

    axp.setLDO3Mode(1);
    axp.setPowerOutPut(AXP202_LDO3, AXP202_ON);

    motor.onec();

    axp.setPowerOutPut(AXP202_EXTEN, AXP202_OFF);
    axp.setPowerOutPut(AXP202_LDO4, AXP202_OFF);
    axp.setPowerOutPut(AXP202_DCDC2, AXP202_OFF);
    axp.setPowerOutPut(AXP202_LDO3, AXP202_OFF);

    bma.begin();

    bma.attachInterrupt();



    pinMode(BMA423_INT1, INPUT);
    attachInterrupt(BMA423_INT1, [] {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        EventBits_t  bits = xEventGroupGetBitsFromISR(isr_group);
        if (bits & WATCH_FLAG_SLEEP_MODE)
        {
            //! Quick wake up
            xEventGroupSetBitsFromISR(isr_group, WATCH_FLAG_SLEEP_EXIT | WATCH_FLAG_BMA_IRQ, &xHigherPriorityTaskWoken);
        } else
        {
            uint8_t data = Q_EVENT_BMA_INT;
            xQueueSendFromISR(g_event_queue_handle, &data, &xHigherPriorityTaskWoken);
        }

        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR ();
        }
    }, RISING);


    pinMode(AXP202_INT, INPUT);
    attachInterrupt(AXP202_INT, [] {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        EventBits_t  bits = xEventGroupGetBitsFromISR(isr_group);
        if (bits & WATCH_FLAG_SLEEP_MODE)
        {
            //! Quick wake up
            xEventGroupSetBitsFromISR(isr_group, WATCH_FLAG_SLEEP_EXIT | WATCH_FLAG_AXP_IRQ, &xHigherPriorityTaskWoken);
        } else
        {
            uint8_t data = Q_EVENT_AXP_INT;
            xQueueSendFromISR(g_event_queue_handle, &data, &xHigherPriorityTaskWoken);
        }
        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR ();
        }
    }, FALLING);

    rtc.check();

    rtc.syncToSystem();

    btn.setClickHandler([](Button2 & b) {
        Serial.println("Button2 Pressed");
        pop_menu();
    });

    btn.setLongClickHandler([](Button2 & b) {
        Serial.println("Pressed Restart Button,Restart now ...");
        delay(1000);
        esp_restart();
    });

}


void setup()
{
    Serial.begin(115200);

    rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);

    g_event_queue_handle = xQueueCreate(20, sizeof(uint8_t));
    g_event_group = xEventGroupCreate();
    isr_group = xEventGroupCreate();

    RTC_Date now = rtc.getDateTime();
    RTC_Date compiled = RTC_Date(__DATE__, __TIME__);
    Serial.printf("compiled: %d:%d:%d - %d:%d:%d\n", compiled.year, compiled.month, compiled.day, compiled.hour, compiled.minute, compiled.second);
    if (now.year < compiled.year ||
            (now.year == compiled.year && now.month < compiled.month ) ||
            (now.year == compiled.year && now.month == compiled.month && now.day < compiled.day)) {
        rtc.setDateTime(compiled);
        Serial.println("reset rtc date time");
    } else {
        const char *time = rtc.formatDateTime(PCF_TIMEFORMAT_YYYY_MM_DD_H_M_S);
        Serial.println(time);
    }

    setupPeripheral();

    setupNetwork();

    setupBuletooth();

    setupMQTT();

    setupLvgl();

    setupFilesystem();

    setupGui();

    lv_disp_trig_activity(NULL);

    lv_task_create([](lv_task_t *args) {
        btn.loop();
    }, 30, 1, nullptr);
}

static bool lenergy = false;

void low_energy()
{
    if (bl.isOn()) {
        xEventGroupSetBits(isr_group, WATCH_FLAG_SLEEP_MODE);
        // esp_sleep = true;
        bl.off();
        stopLvglTick();
        // bma.disalbeIrq();
        bma.enableStepCountInterrupt(false);
        tft_sleep();
        if (!WiFi.isConnected()) {
            lenergy = true;
            // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
            rtc_clk_cpu_freq_set(RTC_CPU_FREQ_2M);
        }
    } else {
        // if (lenergy) {
        //     lenergy = false;
        //     rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);
        // }
        // esp_sleep = false;
        // bma.enableIrq();

        startLvglTick();
        tft_wakeup();
        rtc.syncToSystem();
        updateStepCounter(bma.getCounter());
        updateBatteryLevel();
        updateBatteryIcon(LV_ICON_CALCULATION);
        lv_disp_trig_activity(NULL);
        bl.on();
        bma.enableStepCountInterrupt();
    }
}

void loop()
{
    bool  rlst;
    uint8_t data;
    static uint32_t start = 0;
#if 1
    //! Fast response wake-up interrupt
    EventBits_t  bits = xEventGroupGetBits(isr_group);
    if (bits & WATCH_FLAG_SLEEP_EXIT) {
        // Serial.println("WATCH_FLAG_SLEEP_EXIT");
        // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);
        // bl.on();
        if (lenergy) {
            lenergy = false;
            rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);
        }
        
        low_energy();

        if (bits & WATCH_FLAG_BMA_IRQ) {
            //  Serial.println("WATCH_FLAG_BMA_IRQ");
            do {
                rlst =  bma.readInterrupt();
            } while (!rlst);
            xEventGroupClearBits(isr_group, WATCH_FLAG_BMA_IRQ);
        }
        if (bits & WATCH_FLAG_AXP_IRQ) {
            //  Serial.println("WATCH_FLAG_AXP_IRQ");
            axp.readIRQ();
            axp.clearIRQ();
            //TODO: Only accept axp power pek key short press
            xEventGroupClearBits(isr_group, WATCH_FLAG_AXP_IRQ);
        }
        xEventGroupClearBits(isr_group, WATCH_FLAG_SLEEP_EXIT);
        xEventGroupClearBits(isr_group, WATCH_FLAG_SLEEP_MODE);
    }
    if ((bits & WATCH_FLAG_SLEEP_MODE)) {
        //! No event processing after entering the information screen
        return;
    }

    //! Normal polling
    if (xQueueReceive(g_event_queue_handle, &data, 5 / portTICK_RATE_MS) == pdPASS) {
        switch (data) {
        case Q_EVENT_BMA_INT:
            Serial.println("Q_EVENT_BMA_INT");
            do {
                rlst =  bma.readInterrupt();
            } while (!rlst);

            //! double ciclk
            if (bma.isDoubleClick()) {
                Serial.println("isDoubleClick irq");
            }
            //! setp counter
            if (bma.isStepCounter()) {
                updateStepCounter(bma.getCounter());
            }

            if (bma.isTilt()) {
                Serial.println("isTilt irq");
            }

            if (bma.isActivity()) {
                Serial.print("isActivity irq: ");
                Serial.println(bma.getActivity());
            }

            if (bma.isAnyNoMotion()) {
                Serial.println("isAnyNoMotion irq");
            }
            break;
        case Q_EVENT_AXP_INT:
            axp.readIRQ();
            if (axp.isVbusPlugInIRQ()) {
                updateBatteryIcon(LV_ICON_CHARGE);
                // xEventGroupClearBits(g_event_group, G_EVENT_VBUS_REMOVE);
                // xEventGroupSetBits(g_event_group, G_EVENT_VBUS_PLUGIN);
            }
            if (axp.isVbusRemoveIRQ()) {
                updateBatteryIcon(LV_ICON_CALCULATION);
                // xEventGroupClearBits(g_event_group, G_EVENT_VBUS_PLUGIN);
                // xEventGroupClearBits(g_event_group, G_EVENT_CHARGE_DONE);
                // xEventGroupSetBits(g_event_group, G_EVENT_VBUS_REMOVE);
            }
            if (axp.isChargingDoneIRQ()) {
                // xEventGroupSetBits(g_event_group, G_EVENT_CHARGE_DONE);
                updateBatteryIcon(LV_ICON_CALCULATION);
            }
            if (axp.isPEKShortPressIRQ()) {
                axp.clearIRQ();
                low_energy();
                return;
            }
            axp.clearIRQ();
            break;
        case Q_EVENT_WIFI_SCAN_DONE: {
            int16_t len =  WiFi.scanComplete();
            for (int i = 0; i < len; ++i) {
                wifi_list_add(WiFi.SSID(i).c_str());
            }
            break;
        }
        default:
            break;
        }

    }
    if (lv_disp_get_inactive_time(NULL) < DEFAULT_SCREEN_TIMEOUT) {
        lv_task_handler();
    } else {
        low_energy();
    }
#else
    if (xQueueReceive(g_event_queue_handle, &data, 5 / portTICK_RATE_MS) == pdPASS) {
        switch (data) {
        case Q_EVENT_BMA_INT:
            do {
                rlst =  bma.readInterrupt();
                // delay(2);
            } while (!rlst);

            Serial.printf("BMA STATUS:%x\n", bma.getIrqStatus());

            if (bma.isStepCounter()) {
                updateStepCounter(bma.getCounter());
            }
            if (bma.isDoubleClick()) {
                Serial.println("isDoubleClick irq");
                low_energy();
            }
            if (bma.isTilt()) {
                Serial.println("isTilt irq");
                // low_energy();
                if (bl.isOn() == false) {
                    // bl.on();
                    low_energy();
                }
            }

            if (bma.isActivity()) {
                Serial.print("isActivity irq: ");
                Serial.println(bma.getActivity());
            }

            if (bma.isAnyNoMotion()) {
                Serial.println("isAnyNoMotion irq");
            }
            break;

        case Q_EVENT_AXP_INT:
            axp.readIRQ();
            if (axp.isVbusPlugInIRQ()) {
                updateBatteryIcon(LV_ICON_CHARGE);
                // xEventGroupClearBits(g_event_group, G_EVENT_VBUS_REMOVE);
                // xEventGroupSetBits(g_event_group, G_EVENT_VBUS_PLUGIN);
            }
            if (axp.isVbusRemoveIRQ()) {
                updateBatteryIcon(LV_ICON_CALCULATION);
                // xEventGroupClearBits(g_event_group, G_EVENT_VBUS_PLUGIN);
                // xEventGroupClearBits(g_event_group, G_EVENT_CHARGE_DONE);
                // xEventGroupSetBits(g_event_group, G_EVENT_VBUS_REMOVE);
            }
            if (axp.isChargingDoneIRQ()) {
                // xEventGroupSetBits(g_event_group, G_EVENT_CHARGE_DONE);
                updateBatteryIcon(LV_ICON_CALCULATION);
            }
            if (axp.isPEKShortPressIRQ()) {
                low_energy();
            }
            axp.clearIRQ();
            break;
        case Q_EVENT_WIFI_SCAN_DONE: {
            int16_t len =  WiFi.scanComplete();
            for (int i = 0; i < len; ++i) {
                wifi_list_add(WiFi.SSID(i).c_str());
            }
        }
        break;
        default:
            break;
        }
    }

    if (esp_sleep)return;

    if (lv_disp_get_inactive_time(NULL) < DEFAULT_SCREEN_TIMEOUT) {
        lv_task_handler();
    } else {
        if (!esp_sleep) {
            low_energy();
        }
    }
#endif
}

void per_task(void *)
{
    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(g_event_group, 0xFFFFFF, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & G_EVENT_WIFI_SCAN_START) {
            Serial.println("G_EVENT_WIFI_SCAN_START");
            // if (WiFi.getMode() == WIFI_OFF) {
            //     WiFi.mode(WIFI_STA);
            // }
            // WiFi.scanNetworks();
        }
        if (bits & G_EVENT_WIFI_BEGIN) {
            // if (WiFi.getMode() == WIFI_OFF) {
            //     WiFi.mode(WIFI_STA);
            // }
            // WiFi.begin();
            Serial.println("G_EVENT_WIFI_BEGIN");
        }
        if (bits & G_EVENT_WIFI_OFF) {
            // if (WiFi.getMode() != WIFI_OFF) {
            //     WiFi.mode(WIFI_OFF);
            // }
            Serial.println("G_EVENT_WIFI_OFF");
        }
    }
}


