#include "BLEDevice.h"
#include <Arduino.h>
#include "gui.h"
#include "bluetooth.h"

#define BLUETOOTH_SCAN_SECONDS      5
#define SCAN_MAX_COUNT      20
#define ARRAY_SIZE(x)   (sizeof(x)/sizeof(x[0]))


//! ble heart rate service  uuid
#define HEART_RATE_SERVICE_UUID                 "180D"
#define HEART_RATE_CHARACTERISTIC_UUID          "2A37"

#define BATTERY_SERVICE_UUID                    "180F"
#define BATTERY_CHARACTERISTIC_UUID             "2A19"
#define BLE_ADDRESS                             "F5:55:F4:88:44:B0"


//! ble soil service  uuid
#define CTRL_SERVICE_UUID                   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CTRL_CHARACTERISTIC_UUID            "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SENSOR_SERVICE_UUID                 "4fafc301-1fb5-459e-8fcc-c5c9c331914b"
#define SENSOR_CHARACTERISTIC_UUID          "beb5483c-36e1-4688-b7f5-ea07361b26a8"


static BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;
static BLEAdvertisedDevice *myDevice = nullptr;
static BLERemoteDescriptor *pRemoteSensorDescriptor = nullptr;
static BLERemoteCharacteristic *pRemoteSensorCharacteristic = nullptr;
static BLEScan *pBLEScan = nullptr;
static BLEClient  *pClient = nullptr;
static BLEAdvertisedDevice scanDevices[SCAN_MAX_COUNT];
static int devicesCount = 0;
static int ble_type = -1;

typedef struct {
    int type;
    const char *devName;
} SupportBLE_t;


SupportBLE_t devList[2] = {
    {0, "TTGO_Soil"},
    {1, "Heart Rate"}
};



class MyClientCallback : public BLEClientCallbacks
{
    void onConnect(BLEClient *pclient)
    {
        Serial.println("onConnect");
    }

    void onDisconnect(BLEClient *pclient)
    {
        // connected = false;
        Serial.println("onDisconnect");
        // bluetooth_icon_hidden();
        bluetooth_diconnect_cb();
        ble_type = -1;
    }
};


bool connect_heartrate()
{
    // //! ble heart rate
    BLERemoteService *pRemoteSensorService = pClient->getService(HEART_RATE_SERVICE_UUID);
    if (pRemoteSensorService == nullptr) {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(HEART_RATE_SERVICE_UUID);
    } else {
        Serial.print(" - Found our service");
        Serial.println(HEART_RATE_SERVICE_UUID);
        pRemoteSensorCharacteristic = pRemoteSensorService->getCharacteristic(HEART_RATE_CHARACTERISTIC_UUID);
        if (pRemoteSensorCharacteristic->canNotify()) {
            pRemoteSensorCharacteristic->registerForNotify([](BLERemoteCharacteristic * pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
                //! registerForNotify
                Serial.println(pData[1]);
            });
        }

        pRemoteSensorDescriptor =  pRemoteSensorCharacteristic->getDescriptor(BLEUUID("2902"));
        if (pRemoteSensorDescriptor != nullptr) {
            Serial.println(" - Found our pRemoteSensorDescriptor");
            pRemoteSensorDescriptor->writeValue(1);
        }
    }
    return true;
}

void buletooth_notify_en(bool en)
{
    switch (ble_type) {
    case 0:
        pRemoteSensorDescriptor =  pRemoteSensorCharacteristic->getDescriptor(BLEUUID("2902"));
        if (pRemoteSensorDescriptor != nullptr) {
            Serial.printf(" enable notify:%d  \n", en);
            pRemoteSensorDescriptor->writeValue(en ? 1 : 0, true);
        }
        break;
    case 1:
        break;
    default:
        break;
    }
}

bool connect_soil()
{
    BLERemoteService *pRemoteSensorService = pClient->getService(SENSOR_SERVICE_UUID);
    if (pRemoteSensorService == nullptr) {
        Serial.print("Failed to find our service UUID: ");
        Serial.println(SENSOR_SERVICE_UUID);
        pClient->disconnect();
        return false;
    }

    pRemoteSensorCharacteristic = pRemoteSensorService->getCharacteristic(SENSOR_CHARACTERISTIC_UUID);
    if (pRemoteSensorCharacteristic->canNotify())
        pRemoteSensorCharacteristic->registerForNotify([](BLERemoteCharacteristic * pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
        //! registerForNotify
        if (length == 12) {
            // Serial.printf("Humidity:%.2f Temperature:%.2f soil:%d%%\n",
            //               *(float *) & (pData[0]),
            //               * (float *) & (pData[4]),
            //               * (int *) & (pData[8])
            //              );
            char buffer[256] ;
            snprintf(buffer, sizeof(buffer), "\n\nHumidity:%.2f\n Temperature:%.2f\n soil:%d%%",
                     *(float *) & (pData[0]),
                     * (float *) & (pData[4]),
                     * (int *) & (pData[8])
                    );
            if (!update_soil_data(buffer)) {
                // Serial.println("Off notify now ...");
            }
        }
    });

    // pRemoteSensorDescriptor =  pRemoteSensorCharacteristic->getDescriptor(BLEUUID("2902"));
    // if (pRemoteSensorDescriptor != nullptr) {
    //     Serial.print(" - Found Descriptor");
    //     // pRemoteSensorDescriptor->writeValue(1);
    //     uint8_t r = pRemoteSensorDescriptor->readUInt8();
    //     Serial.printf("read r : %u\n", r);

    //     if(r == 1){
    //         pRemoteSensorDescriptor->writeValue(0);
    //     }
    // }

    // BLERemoteService *pRemoteService = pClient->getService(BLEUUID(CTRL_SERVICE_UUID));
    // if (pRemoteService == nullptr) {
    //     Serial.print("Failed to find our service UUID: ");
    //     Serial.println(CTRL_SERVICE_UUID);
    //     pClient->disconnect();
    //     return false;
    // }

    // Serial.println(" - Found our service");

    // pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(CTRL_CHARACTERISTIC_UUID));
    // if (pRemoteCharacteristic == nullptr) {
    //     Serial.print("Failed to find our characteristic UUID: ");
    //     Serial.println(CTRL_CHARACTERISTIC_UUID);
    //     pClient->disconnect();
    //     return false;
    // }

    // Serial.println(" - Found our characteristic");
    // // Read the value of the characteristic.
    // if (pRemoteCharacteristic->canRead()) {
    //     uint8_t *pData = pRemoteCharacteristic->readRawData();
    //     if (pData) {
    //         Serial.printf("pRemoteCharacteristic Read : %x\n", pData[0]);
    //         // ledstatus =  pData[0];
    //     }
    // }
    return true;
}

int bluetooth_get_connect_type()
{
    return ble_type;
}

Ble_Connect_t bluetooth_connect(const char *devname)
{
    ble_type = -1;

    for (int i = 0; i < ARRAY_SIZE(devList); i++) {
        if (!strcmp(devname, devList[i].devName)) {
            ble_type = devList[i].type;
            Serial.printf("Connect type :%d \n", ble_type);
            break;
        }
    }

    if (ble_type < 0 ) {
        Serial.println("No support device");
        return BLE_DEVICE_NO_SUPPORT;
    }

    for (int i = 0; i < devicesCount; i++) {
        const char *name = scanDevices[i].getName().c_str();
        if (!strcmp(name, devname)) {
            myDevice = &(scanDevices[i]);
            break;
        }
    }

    if (myDevice == nullptr) {
        return BLE_DEVICE_DISCONNECT;
    }


    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());

    pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    bool ret = false;
    switch (ble_type) {
    case 0:
        ret = connect_soil();
        break;
    case 1:
        //TODO : ...
        break;
    default:
        break;
    }
    return ret ? BLE_DEVICE_CONNECT : BLE_DEVICE_DISCONNECT;
}

static void scanCompleteCB(BLEScanResults result)
{
    Serial.printf(">> Dump scan results:");
    int count = result.getCount();
    for (int i = 0; i < count; i++) {
        Serial.printf( "- %s\n", result.getDevice(i).toString().c_str());
        std::string name = result.getDevice(i).getName();
        if (name != "" && devicesCount < SCAN_MAX_COUNT) {
            scanDevices[devicesCount] = result.getDevice(i);
            bluetooth_list_add(name.c_str());
            ++devicesCount;
        }
    }

    if (count == 0 || devicesCount == 0) {
        bluetooth_list_add(NULL);
        return;
    }
}

/*****************************************************************
 *
 *          ! BLUETOOTH
 *
 */

void setupBuletooth()
{
    BLEDevice::init("TWatch");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
}

bool bluetooth_isConnect()
{
    return pClient != nullptr && pClient->isConnected();
}

void bluetooth_start()
{
    bluetooth_stop();
    pBLEScan->start(BLUETOOTH_SCAN_SECONDS, scanCompleteCB);
}

void bluetooth_stop()
{
    if (pClient != nullptr) {
        pClient->disconnect();
        // delete pClient;
        // pClient = nullptr;
    }
}

