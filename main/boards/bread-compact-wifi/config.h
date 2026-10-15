#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// 如果使用 Duplex I2S 模式，请注释下面一行
#define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX

#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16

#else

#define AUDIO_I2S_GPIO_WS GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7

#endif


#define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define TOUCH_BUTTON_GPIO       GPIO_NUM_47
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_40
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_39

#define DISPLAY_SDA_PIN GPIO_NUM_41
#define DISPLAY_SCL_PIN GPIO_NUM_42
#define DISPLAY_WIDTH   128

#if CONFIG_OLED_SSD1306_128X32
#define DISPLAY_HEIGHT  32
#elif CONFIG_OLED_SSD1306_128X64
#define DISPLAY_HEIGHT  64
#elif CONFIG_OLED_SH1106_128X64
#define DISPLAY_HEIGHT  64
#define SH1106
#else
#error "OLED display type is not selected"
#endif

#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true


// A MCP Test: Control a lamp
#define LAMP_GPIO GPIO_NUM_18

// Fan relay (HIGH = on)
#define FAN_GPIO GPIO_NUM_8

// Flame sensor DO pin. Most LM393 modules pull DO LOW when flame is detected.
// If yours does the opposite, change FLAME_ACTIVE_LEVEL to 1.
#define FLAME_GPIO GPIO_NUM_2
#define FLAME_ACTIVE_LEVEL 0

// Automatic fire alarm: when the flame sensor sees a fire, the robot plays a recorded
// male voice warning by itself (firealarm.ogg, built into the firmware). It works even
// without Wi-Fi and interrupts the AI if it is talking.
// The sensor is read every 100 ms. A fire is confirmed after FLAME_CONFIRM_TICKS
// readings in a row (5 = 0.5 s) and cleared after FLAME_CLEAR_TICKS (30 = 3 s).
#define FLAME_ALERT_ENABLED 1
#define FLAME_CONFIRM_TICKS 5
#define FLAME_CLEAR_TICKS 30
// Pause between voice warnings while the fire lasts (10 = 1 second after each warning ends).
#define FLAME_REPEAT_TICKS 10


// Temperature sensor (red = 3.3V, black = GND, yellow = DATA).
// A 4.7k-10k resistor between DATA and 3.3V is recommended.
// Sensor type: 1 = DHT11 (blue sensor, whole degrees, about +/-2 C), 0 = DHT22 / AM2302.
#define TEMP_SENSOR_DHT11 1
#define TEMP_START_US (TEMP_SENSOR_DHT11 ? 20000 : 2000)
#define TEMP_GPIO GPIO_NUM_9
#define TEMP_READ_INTERVAL_MS 3000
// The fan turns off again when the temperature drops this many degrees below the rule.
#define TEMP_HYSTERESIS_C 1

// Gas / smoke sensor: MQ module powered from 3.3V, DO pin (LOW = gas detected on most modules).
// If your module does the opposite, change GAS_ACTIVE_LEVEL to 1.
#define GAS_GPIO GPIO_NUM_10
#define GAS_ACTIVE_LEVEL 0
#define GAS_ALERT_ENABLED 1
// MQ sensors need time to heat up. The alarm is switched on only after this many seconds.
#define GAS_WARMUP_SECONDS 120
#define GAS_CONFIRM_TICKS 5
#define GAS_CLEAR_TICKS 30
#define GAS_REPEAT_TICKS 10


// Phone notifications (ntfy). The same TOPIC must be subscribed in the ntfy app on your phone.
// The topic acts like a password: keep it long and random, and do not share it.
#define NOTIFY_ENABLED 1
#define NOTIFY_SERVER "https://ntfy.sh"
#define NOTIFY_TOPIC "alhassan-robot-6fkgyrcpwhstre"
// While a fire or gas alarm continues, a new message is sent every this many seconds.
#define NOTIFY_REPEAT_SECONDS 120

#endif // _BOARD_CONFIG_H_
