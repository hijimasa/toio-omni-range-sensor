// Pin map for M5Stack Timer Camera (K034 / K034-F "Timer Camera F").
// Values taken from m5stack/TimerCam-arduino
//   src/utility/Camera_Class.h and src/utility/Power_Class.h
#pragma once

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  25
#define SIOC_GPIO_NUM  23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 32

#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#define CAMERA_LED_GPIO 2

// Held HIGH to keep the on-board LDO enabled. Without this the unit powers
// down as soon as the power button is released when running on battery.
#define POWER_HOLD_PIN 33
#define BAT_ADC_PIN    38
// Battery is fed to the ADC through a divider; Vbat = Vadc / SCALE.
#define BAT_ADC_SCALE 0.661f
