// /////////////////////////////////////////////////////////////////
// /*
// MIT License

// Copyright (c) 2019 lewis he

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// FT5206.cpp - Arduino library for FT5206 chip.
// Created by Lewis on April 17, 2019.
// github:https://github.com/lewisxhe/FT5206_Library
// */
// /////////////////////////////////////////////////////////////////
// #include "FT5206.h"


// I2CBus *FT5206_Class::_bus = nullptr;

// uint16_t FT5206_Class::read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
// {
//     return _bus->readBytes(addr, reg, data, len);
// }

// uint16_t FT5206_Class::write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
// {
//     return _bus->writeBytes(addr, reg, data, len);
// }

// FT5206_Class::FT5206_Class(I2CBus &port, uint8_t addr)
// {
//     _bus = &port;
//     _address = addr;
// };

// bool FT5206_Class::begin()
// {
//     return __init();
// }

// bool FT5206_Class::__init()
// {
//     uint8_t val;
//     read(_address, FT5206_VENDID_REG, &val, 1);
//     Serial.printf("[touchpad]id : %x\n", val);
//     if (val != FT5206_VENDID) {
//         Serial.println("[touchpad] init fail");
//         return false;
//     }
//     read(_address, FT5206_CHIPID_REG, &val, 1);
//     if ((val != FT6206_CHIPID) && (val != FT6236_CHIPID) && (val != FT6236U_CHIPID) && (val != FT5206U_CHIPID)) {
//         Serial.println("[touchpad] init fail");
//         return false;
//     }
//     _init = true;
//     return _init;
// }

// // valid touching detect threshold.
// void FT5206_Class::adjustTheshold(uint8_t thresh)
// {
//     if (!_init)return;
//     write(_address, FT5206_THRESHHOLD_REG, &thresh, 1);
// }

// TP_Point FT5206_Class::getPoint(uint8_t num)
// {
//     if (!_init) return TP_Point(0, 0);
//     _readRegister();
//     if ((_touches == 0) || (num > 1)) {
//         return TP_Point(0, 0);
//     } else {
//         return TP_Point(_x[num], _y[num]);
//     }
// }

// uint8_t FT5206_Class::touched()
// {
//     if (!_init)return 0;
//     uint8_t val = 0;
//     read(_address, FT5206_TOUCHES_REG, &val, 1);
//     return val > 2 ? 0 : val;
// }

// void FT5206_Class::sleep()
// {
//     if (!_init)return;
//     uint8_t val = FT5206_SLEEP_IN;
//     write(_address, FT5206_POWER_REG, &val, 1);
// }

// void FT5206_Class::monitor()
// {
//     if (!_init)return;
//     uint8_t val = FT5206_MONITOR;
//     write(_address, FT5206_POWER_REG, &val, 1);
// }

// void FT5206_Class::_readRegister()
// {
//     read(_address, DEVIDE_MODE, _data, 16);
//     _touches = _data[TD_STATUS];
//     if ((_touches > 2) || (_touches == 0)) {
//         _touches = 0;
//         return;
//     }
//     for (uint8_t i = 0; i < 2; i++) {
//         _x[i] = _data[TOUCH1_XH + i * 6] & 0x0F;
//         _x[i] <<= 8;
//         _x[i] |= _data[TOUCH1_XL + i * 6];
//         _y[i] = _data[TOUCH1_YH + i * 6] & 0x0F;
//         _y[i] <<= 8;
//         _y[i] |= _data[TOUCH1_YL + i * 6];
//         _id[i] = _data[TOUCH1_YH + i * 6] >> 4;
//     }
// }