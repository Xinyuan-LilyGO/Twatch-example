#ifndef __STORAGE_H
#define __STORAGE_H

#include <stdint.h>
#include "esp_system.h"
#include "nvs.h"

class Storage
{
public:
    Storage() {};
    ~Storage() {};
    bool save(const char *name, const char *key, void *data, uint16_t len)
    {
        esp_err_t ret;
        nvs_handle handle;
        if ( ESP_OK != nvs_open(name, NVS_READWRITE, &handle))return false;

        if (ESP_OK != nvs_set_blob(handle, key, data, len)) {
            nvs_close(handle);
            return false;
        }
        ret = nvs_commit(handle) ;
        nvs_close(handle);
        return ret == ESP_OK;
    }

    bool load(const char *name, const char *key, void *data)
    {
        esp_err_t ret = ESP_ERR_INVALID_ARG;
        nvs_handle handle;
        size_t required_size = 0;
        if ( ESP_OK != nvs_open(name, NVS_READWRITE, &handle))return false;

        ret = nvs_get_blob(handle, key, NULL, &required_size);
        if (ret != ESP_OK) {
            nvs_close(handle);
            return false;
        }
        if (required_size == 0) {
            printf( "the target you want to load has never been saved\n");
            nvs_close(handle);
            return false;
        }
        ret = nvs_get_blob(handle, key, data, &required_size);
        nvs_close(handle);
        return ret == ESP_OK;
    }

    bool erase(const char *name, const char *key)
    {
        esp_err_t ret = ESP_ERR_INVALID_ARG;
        nvs_handle handle;
        ret = nvs_open(name, NVS_READWRITE, &handle);
        if (ret != ESP_OK) {
            return false;
        }
        ret = nvs_erase_key(handle, key);
        if (ret != ESP_OK) {
            nvs_close(handle);
            return false;
        }
        ret = nvs_commit(handle);
        nvs_close(handle);
        return ret == ESP_OK;
    }
private:

};




#endif  /* __STORAGE_H */