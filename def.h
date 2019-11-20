#ifndef __DEF_H
#define __DEF_H


#ifndef _BV
#define _BV(x)  (1<<(x))
#endif /*_BV */




#define G_EVENT_VBUS_PLUGIN  _BV(0)
#define G_EVENT_VBUS_REMOVE  _BV(1)
#define G_EVENT_CHARGE_DONE  _BV(2)

#define G_EVENT_WIFI_SCAN_START  _BV(3)
#define G_EVENT_WIFI_SCAN_DONE   _BV(4)
#define G_EVENT_WIFI_CONNECTED   _BV(5)
#define G_EVENT_WIFI_BEGIN       _BV(6)
#define G_EVENT_WIFI_OFF         _BV(7)



enum {
    Q_EVENT_WIFI_SCAN_DONE,
    Q_EVENT_WIFI_CONNECT,
    Q_EVENT_BMA_INT,
    Q_EVENT_AXP_INT,
} ;




#endif /*__DEF_H */