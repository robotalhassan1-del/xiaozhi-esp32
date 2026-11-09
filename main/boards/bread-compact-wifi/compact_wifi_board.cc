#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/led.h"
#include <driver/ledc.h>
#include <driver/gpio.h>
#include "assets/lang_config.h"
#include "settings.h"

// Short attention sound played at once when a danger is found, while the AI is still connecting.
#if AI_ALERT_ENABLED
#define ALERT_ATTN_SOUND Lang::Sounds::OGG_EXCLAMATION
#else
#define ALERT_ATTN_SOUND ""
#endif

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "CompactWifiBoard"

// ---------------------------------------------------------------------------
// Shared state (fan, fire alarm, temperature rule)
// ---------------------------------------------------------------------------
static std::atomic<bool> g_fan_on{false};
static std::atomic<bool> g_fan_auto_on{false};      // true when the temperature rule switched the fan on
static std::atomic<bool> g_fire_active{false};      // true while the fire alarm is sounding
static std::atomic<bool> g_gas_active{false};       // true while gas / smoke is detected
static std::atomic<bool> g_alarm_voice{false};      // true while a danger warning voice is playing (red eyes stay on)
static std::atomic<int>  g_fan_rule_x10{0};         // fan rule: temperature in tenths of a degree (383 = 38.3 C), 0 = no rule
static std::atomic<bool> g_rule_latched{false};     // the rule already acted in this hot period
static std::atomic<float> g_temp_c{NAN};
static std::atomic<float> g_humidity{NAN};
static std::atomic<int64_t> g_temp_time_us{0};
static std::atomic<bool> g_lamp_on{false};
static std::atomic<bool> g_pir_mode{false};         // motion-light mode: motion turns the lamp on, calm turns it off
static std::atomic<int>  g_motion_lamps_mask{1};    // room lamps used by the motion rule: bit0 = lamp 1, bit1 = lamp 2, bit2 = lamp 3
static std::atomic<bool> g_motion_lit{false};       // the motion rule has switched the room lamps on
static std::atomic<int64_t> g_pir_armed_us{0};      // the motion rule starts working at this time
static std::atomic<int>  g_pir_off_seconds{60};     // lamp goes off after this many seconds without motion
static std::atomic<int64_t> g_last_motion_us{0};
static std::atomic<bool> g_motion_now{false};
// Eyes controlled by voice (AI): colour in percent, effect 0 = solid, 1 = blink, 2 = breathe, 3 = rainbow.
static std::atomic<int> g_eye_r{0}, g_eye_g{0}, g_eye_b{0};
static std::atomic<int> g_eye_effect{0};
static std::atomic<int64_t> g_eye_override_until_us{0};
// Away mode: the motion sensor watches the house and a phone message is sent when it sees movement.
static std::atomic<bool> g_away_mode{false};
static std::atomic<int64_t> g_away_armed_us{0};      // the watch starts at this time (leaving delay)
static std::atomic<int64_t> g_intruder_until_us{0};  // red slow blink until this time
static std::atomic<int64_t> g_last_away_notify_us{0};


// ---------------------------------------------------------------------------
// Phone notifications through ntfy (https://ntfy.sh). Messages are queued by the alarm
// tasks and sent by a separate task, so a slow network never delays an alarm.
// ---------------------------------------------------------------------------
struct NotifyMessage {
    char title[48];
    char body[160];
    char tags[48];
    int priority;
};
static QueueHandle_t g_notify_queue = nullptr;

static void QueueNotify(const char* title, const char* body, const char* tags, int priority) {
#if NOTIFY_ENABLED
    if (g_notify_queue == nullptr) {
        return;
    }
    NotifyMessage m = {};
    snprintf(m.title, sizeof(m.title), "%s", title);
    snprintf(m.body, sizeof(m.body), "%s", body);
    snprintf(m.tags, sizeof(m.tags), "%s", tags);
    m.priority = priority;
    xQueueSend(g_notify_queue, &m, 0);  // never wait: if the queue is full the message is dropped
#endif
}

#if NOTIFY_ENABLED
static bool SendNtfy(const NotifyMessage& m) {
    auto network = Board::GetInstance().GetNetwork();
    if (network == nullptr) {
        return false;
    }
    auto http = network->CreateHttp(0);
    if (!http) {
        return false;
    }
    http->SetTimeout(10000);
    http->SetHeader("Title", m.title);
    http->SetHeader("Priority", std::to_string(m.priority));
    http->SetHeader("Tags", m.tags);
    http->SetHeader("Content-Type", "text/plain");
    http->SetContent(std::string(m.body));

    std::string url = std::string(NOTIFY_SERVER) + "/" + NOTIFY_TOPIC;
    auto opened = http->Open("POST", url);
    if (!opened) {
        ESP_LOGE(TAG, "Notification: cannot connect: %s", opened.error().ToString().c_str());
        return false;
    }
    auto status = http->GetStatusCode();
    bool ok = status && *status >= 200 && *status < 300;
    if (!ok) {
        ESP_LOGE(TAG, "Notification: server answered with status %d", status ? *status : -1);
    }
    http->ReadAll();
    http->Close();
    return ok;
}

static void NotifyTask(void* arg) {
    NotifyMessage m;
    while (true) {
        if (xQueueReceive(g_notify_queue, &m, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        bool ok = false;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            ok = SendNtfy(m);
            if (!ok) {
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
        }
        ESP_LOGI(TAG, "Notification \"%s\": %s", m.title, ok ? "sent" : "FAILED");
    }
}
#endif

// ---------------------------------------------------------------------------
// Commands to the wall switch (room lamps and fan). The robot asks the server, which hands the
// command to the ESP32 at the wall. Commands are queued and sent by a separate task.
// ---------------------------------------------------------------------------
struct RoomCmd {
    char device[8];   // "lamp1", "lamp2", "lamp3", "fan"
    bool on;
};
static QueueHandle_t g_room_queue = nullptr;

static void RoomSend(const char* device, bool on) {
    if (g_room_queue == nullptr) {
        return;
    }
    RoomCmd c = {};
    snprintf(c.device, sizeof(c.device), "%s", device);
    c.on = on;
    xQueueSend(g_room_queue, &c, 0);
}

static void RoomLamps(int mask, bool on) {
    for (int i = 0; i < 3; i++) {
        if (mask & (1 << i)) {
            char d[8];
            snprintf(d, sizeof(d), "lamp%d", i + 1);
            RoomSend(d, on);
        }
    }
}

static bool SendRoomCommand(const RoomCmd& c) {
    auto network = Board::GetInstance().GetNetwork();
    if (network == nullptr) {
        return false;
    }
    auto http = network->CreateHttp(0);
    if (!http) {
        return false;
    }
    http->SetTimeout(15000);
    std::string url = std::string(ROOM_SERVER) + "/room/set?token=" + ROOM_TOKEN + "&device=" + c.device +
                      "&on=" + (c.on ? "1" : "0");
    auto opened = http->Open("GET", url);
    if (!opened) {
        ESP_LOGE(TAG, "Room command: cannot connect: %s", opened.error().ToString().c_str());
        return false;
    }
    auto status = http->GetStatusCode();
    bool ok = status && *status >= 200 && *status < 300;
    if (!ok) {
        ESP_LOGE(TAG, "Room command: server answered with status %d", status ? *status : -1);
    }
    http->ReadAll();
    http->Close();
    return ok;
}

static void RoomTask(void* arg) {
    RoomCmd c;
    while (true) {
        if (xQueueReceive(g_room_queue, &c, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        bool ok = false;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            ok = SendRoomCommand(c);
            if (!ok) {
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }
        ESP_LOGI(TAG, "Room command %s %s: %s", c.device, c.on ? "ON" : "OFF", ok ? "sent" : "FAILED");
    }
}

// "1", "2", "3", "1,3", "all" ... -> bit mask of room lamps
static int ParseLampMask(const std::string& s) {
    if (s.find("all") != std::string::npos || s.find("All") != std::string::npos ||
        s.find("duk") != std::string::npos) {
        return 7;
    }
    int mask = 0;
    for (char ch : s) {
        if (ch >= '1' && ch <= '3') {
            mask |= 1 << (ch - '1');
        }
    }
    return mask != 0 ? mask : 1;
}

static std::string LampMaskText(int mask) {
    std::string t;
    for (int i = 0; i < 3; i++) {
        if (mask & (1 << i)) {
            if (!t.empty()) t += ",";
            t += std::to_string(i + 1);
        }
    }
    return t;
}

static void SetFan(bool on) {
    g_fan_on = on;
    gpio_set_level(FAN_GPIO, on ? 1 : 0);
}

static void SetLamp(bool on) {
    g_lamp_on = on;
    gpio_set_level(LAMP_GPIO, on ? 1 : 0);
}

// Waits until the pin reaches `level`. Returns the time waited in microseconds, or -1 on timeout.
static int WaitForLevel(gpio_num_t pin, int level, int timeout_us) {
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(pin) != level) {
        if (esp_timer_get_time() - start > timeout_us) {
            return -1;
        }
    }
    return (int)(esp_timer_get_time() - start);
}

// Reads a DHT11 or AM2302 / DHT22 sensor (one-wire protocol). Returns true on success.
static bool ReadAm2302(gpio_num_t pin, float* temperature, float* humidity, int* stage) {
    *stage = 0;
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    uint8_t data[5] = {0, 0, 0, 0, 0};

    gpio_reset_pin(pin);  // make sure the pin is a plain GPIO
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(pin, 0);
    esp_rom_delay_us(TEMP_START_US);  // start signal: DHT11 needs >= 18 ms low, DHT22 only 1 ms

    bool ok = true;
    portENTER_CRITICAL(&mux);
    gpio_set_level(pin, 1);  // release the line
    if (WaitForLevel(pin, 0, 150) < 0) {
        ok = false; *stage = 1;      // the sensor never answered (nothing connected / wrong pin)
    } else if (WaitForLevel(pin, 1, 150) < 0) {
        ok = false; *stage = 2;      // the line is stuck low
    } else if (WaitForLevel(pin, 0, 150) < 0) {
        ok = false; *stage = 3;
    }
    for (int i = 0; ok && i < 40; i++) {
        if (WaitForLevel(pin, 1, 150) < 0) { ok = false; *stage = 4; break; }
        int high_us = WaitForLevel(pin, 0, 150);
        if (high_us < 0) { ok = false; *stage = 4; break; }
        data[i / 8] <<= 1;
        if (high_us > 40) {
            data[i / 8] |= 1;
        }
    }
    portEXIT_CRITICAL(&mux);

    if (!ok) {
        return false;
    }
    if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]) {
        *stage = 5;                  // data arrived but is corrupted
        return false;
    }
#if TEMP_SENSOR_DHT11
    // DHT11: whole degrees in byte 2 (some versions add tenths in byte 3).
    float h = data[0] + (data[1] & 0x0F) / 10.0f;
    float t = data[2] + (data[3] & 0x0F) / 10.0f;
    if (data[3] & 0x80) {
        t = -t;
    }
#else
    float h = ((data[0] << 8) | data[1]) / 10.0f;
    float t = (((data[2] & 0x7F) << 8) | data[3]) / 10.0f;
    if (data[2] & 0x80) {
        t = -t;
    }
#endif
    if (h > 100.0f || t < -40.0f || t > 80.0f) {
        *stage = 6;
        return false;
    }
    *temperature = t;
    *humidity = h;
    return true;
}

// ---------------------------------------------------------------------------
// Eyes: red, green and blue LEDs (each colour on one pin, shared by both eyes), driven with PWM.
//   Starting / looking for Wi-Fi ........ blue blinks
//   Idle (waiting for the wake word) .... red, full brightness
//   Robot speaking ...................... green, full brightness
//   Waiting for you to talk ............. blue, low brightness
//   You are talking ..................... blue, brighter
//   Fire / gas (and while the warning
//   voice is playing) ................... red blinks fast, the other colours off
//   Away mode, movement seen ............ red blinks slowly (once a second)
//   Colour/effect asked by voice ........ for a few seconds (self.eyes.show), then back to normal
// ---------------------------------------------------------------------------
class EyeLeds : public Led {
public:
    EyeLeds() {
        ledc_timer_config_t t = {};
        t.speed_mode = LEDC_LOW_SPEED_MODE;
        t.duty_resolution = LEDC_TIMER_10_BIT;
        t.timer_num = LEDC_TIMER_2;
        t.freq_hz = 1000;
        t.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&t));
        const gpio_num_t pins[3] = {EYE_RED_GPIO, EYE_GREEN_GPIO, EYE_BLUE_GPIO};
        for (int i = 0; i < 3; i++) {
            ledc_channel_config_t c = {};
            c.gpio_num = pins[i];
            c.speed_mode = LEDC_LOW_SPEED_MODE;
            c.channel = (ledc_channel_t)(LEDC_CHANNEL_5 + i);
            c.timer_sel = LEDC_TIMER_2;
            c.duty = 0;
            c.hpoint = 0;
            ESP_ERROR_CHECK(ledc_channel_config(&c));
        }
        // Limit the current each pin can give (protects the pins, because green and blue have no resistor).
        gpio_set_drive_capability(EYE_RED_GPIO, GPIO_DRIVE_CAP_2);
        gpio_set_drive_capability(EYE_GREEN_GPIO, GPIO_DRIVE_CAP_1);
        gpio_set_drive_capability(EYE_BLUE_GPIO, GPIO_DRIVE_CAP_1);
        xTaskCreate(Task, "eye_leds", 3072, this, tskIDLE_PRIORITY + 1, nullptr);
    }

    void OnStateChanged() override {}  // the task below follows the state by itself

private:
    float cur_[3] = {0, 0, 0};

    static void Set(int ch, float level) {  // level 0..1
        if (level < 0) level = 0;
        if (level > 1) level = 1;
        float g = level * level;            // gamma: gives smoother low brightness
        uint32_t duty = (uint32_t)(g * 1023.0f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(LEDC_CHANNEL_5 + ch), duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(LEDC_CHANNEL_5 + ch));
    }

    static void Task(void* arg) {
        auto* self = static_cast<EyeLeds*>(arg);
        int tick = 0;
        while (true) {
            float target[3] = {0, 0, 0};  // red, green, blue
            float rate = 0.30f;
            int64_t now = esp_timer_get_time();
            if (g_fire_active || g_gas_active || g_alarm_voice) {
                target[0] = ((tick / 4) % 2 == 0) ? EYE_RED_LEVEL : 0.0f;  // fast blink (about 3 per second)
                rate = 0.7f;
            } else if (g_away_mode && now < g_intruder_until_us) {
                target[0] = ((tick / 12) % 2 == 0) ? EYE_RED_LEVEL : 0.0f;  // slow blink (once a second)
                rate = 0.5f;
            } else if (now < g_eye_override_until_us) {
                float c[3] = {g_eye_r / 100.0f, g_eye_g / 100.0f, g_eye_b / 100.0f};
                switch (g_eye_effect) {
                    case 1: {  // blink
                        bool on = (tick / 8) % 2 == 0;
                        for (int i = 0; i < 3; i++) target[i] = on ? c[i] : 0.0f;
                        rate = 0.6f;
                        break;
                    }
                    case 2: {  // breathe
                        float k = 0.5f * (1.0f - cosf(tick * 0.0785f));
                        for (int i = 0; i < 3; i++) target[i] = c[i] * k;
                        rate = 0.5f;
                        break;
                    }
                    case 3: {  // rainbow: the colour wheel turns once in about 3.6 seconds
                        float h = fmodf(tick * 4.0f, 360.0f) / 60.0f;
                        int sector = (int)h;
                        float f = h - sector;
                        float rgb[3];
                        switch (sector % 6) {
                            case 0: rgb[0] = 1; rgb[1] = f; rgb[2] = 0; break;
                            case 1: rgb[0] = 1 - f; rgb[1] = 1; rgb[2] = 0; break;
                            case 2: rgb[0] = 0; rgb[1] = 1; rgb[2] = f; break;
                            case 3: rgb[0] = 0; rgb[1] = 1 - f; rgb[2] = 1; break;
                            case 4: rgb[0] = f; rgb[1] = 0; rgb[2] = 1; break;
                            default: rgb[0] = 1; rgb[1] = 0; rgb[2] = 1 - f; break;
                        }
                        for (int i = 0; i < 3; i++) target[i] = rgb[i];
                        rate = 0.6f;
                        break;
                    }
                    default:
                        for (int i = 0; i < 3; i++) target[i] = c[i];
                        break;
                }
            } else {
                switch (Application::GetInstance().GetDeviceState()) {
                    case kDeviceStateIdle:
                        target[0] = EYE_RED_IDLE;
                        break;
                    case kDeviceStateSpeaking:
                    case kDeviceStateNotifying:
                        target[1] = EYE_GREEN_FULL;
                        break;
                    case kDeviceStateListening:
                    case kDeviceStateAudioTesting:
                        target[2] = Application::GetInstance().IsVoiceDetected() ? EYE_BLUE_FULL : EYE_BLUE_DIM;
                        break;
                    case kDeviceStateConnecting:
                        target[2] = EYE_BLUE_DIM;
                        break;
                    default:  // starting, Wi-Fi setup, activating, upgrading, error: blue blinks
                        target[2] = ((tick / 12) % 2 == 0) ? EYE_BLUE_FULL : 0.0f;  // about 0.5 s on, 0.5 s off
                        rate = 0.6f;
                        break;
                }
            }
            for (int i = 0; i < 3; i++) {
                self->cur_[i] += (target[i] - self->cur_[i]) * rate;
                Set(i, self->cur_[i]);
            }
            tick++;
            vTaskDelay(pdMS_TO_TICKS(40));
        }
    }
};

class CompactWifiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSsd1306Display() {
        // SSD1306 config
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif
        ESP_LOGI(TAG, "SSD1306 driver installed");

        // Reset the display
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        // Set the display to on
        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

#if AI_ALERT_ENABLED
    // Asks the AI (robot -> server -> Gemini) to speak a danger warning in its own voice.
    // The word "ALERT_FIRE" / "ALERT_GAS" is sent to the server like a wake word; the server
    // turns it into an urgent instruction for Gemini.
    // Returns  1 = the AI spoke the warning,
    //          0 = it did not start speaking in time (no Wi-Fi / server asleep / Gemini down),
    //         -1 = the robot was busy connecting, try again in a moment.
    static int AiAlarm(const char* key) {
        auto& app = Application::GetInstance();
        DeviceState s = app.GetDeviceState();
        if (s == kDeviceStateListening || s == kDeviceStateSpeaking || s == kDeviceStateNotifying) {
            // Close the open conversation (or stop the speech) so a fresh one can start with the alert.
            app.WakeWordInvoke(key);
            vTaskDelay(pdMS_TO_TICKS(1500));
            s = app.GetDeviceState();
        }
        if (s != kDeviceStateIdle) {
            return -1;
        }
        app.WakeWordInvoke(key);
        for (int i = 0; i < 120; i++) {  // wait up to 12 s for the AI to start speaking
            vTaskDelay(pdMS_TO_TICKS(100));
            if (app.GetDeviceState() == kDeviceStateSpeaking) {
                for (int j = 0; j < 400 && app.GetDeviceState() == kDeviceStateSpeaking; j++) {
                    vTaskDelay(pdMS_TO_TICKS(100));  // let it finish (max 40 s)
                }
                return 1;
            }
        }
        return 0;
    }
#endif

#if FLAME_ALERT_ENABLED
    // Watches the flame sensor in the background. When a fire is seen it cuts off whatever
    // the AI is saying and plays a recorded male voice warning, again and again, until the
    // fire is gone. It never turns the fan on: moving air can make a fire bigger.
    static void FlameMonitorTask(void* arg) {
        // Let the device finish booting, joining Wi-Fi and activation first.
        vTaskDelay(pdMS_TO_TICKS(20000));

        int active_ticks = 0;
        int clear_ticks = 0;
        int ticks_since_voice = 0;
        int64_t last_notify_us = 0;
        bool fire = false;

        while (true) {
            bool detected = gpio_get_level(FLAME_GPIO) == FLAME_ACTIVE_LEVEL;
            if (detected) {
                active_ticks++;
                clear_ticks = 0;
            } else {
                clear_ticks++;
                active_ticks = 0;
            }

            if (!fire && active_ticks >= FLAME_CONFIRM_TICKS) {
                fire = true;
                g_fire_active = true;
                ticks_since_voice = FLAME_REPEAT_TICKS;
                last_notify_us = esp_timer_get_time();
                QueueNotify("FIRE ALERT", "Fire detected by the robot's flame sensor. Check the house now!",
                            "fire,rotating_light", 5);
                ESP_LOGW(TAG, "FIRE detected by the flame sensor");
                // Interrupt the AI if it is talking and drop the audio it queued.
                Application::GetInstance().Schedule([]() {
                    auto& app = Application::GetInstance();
                    app.AbortSpeaking(kAbortReasonNone);
                    app.GetAudioService().ResetDecoder();
                });
                vTaskDelay(pdMS_TO_TICKS(500));
            } else if (fire && clear_ticks >= FLAME_CLEAR_TICKS) {
                fire = false;
                g_fire_active = false;
                ESP_LOGI(TAG, "Flame sensor no longer sees a fire");
                QueueNotify("Fire cleared", "The flame sensor no longer sees a fire.", "white_check_mark", 3);
                Application::GetInstance().Schedule([]() {
                    Application::GetInstance().Alert("Info", "Flame gone", "neutral");
                });
            }

            if (fire && detected &&
                esp_timer_get_time() - last_notify_us > (int64_t)NOTIFY_REPEAT_SECONDS * 1000000) {
                last_notify_us = esp_timer_get_time();
                QueueNotify("FIRE ALERT (still burning)", "The flame sensor still sees a fire!",
                            "fire,rotating_light", 5);
            }
            if (fire && detected) {
                ticks_since_voice++;
                if (ticks_since_voice >= FLAME_REPEAT_TICKS) {
                    ticks_since_voice = 0;
                    Application::GetInstance().Schedule([]() {
                        Application::GetInstance().Alert("FIRE!", "FIRE DETECTED!", "angry", ALERT_ATTN_SOUND);
                    });
#if AI_ALERT_ENABLED
                    // The AI speaks the warning. If it cannot (no internet, server asleep), the
                    // recorded voice is played instead, so the alarm never stays silent.
                    g_alarm_voice = true;
                    int r = AiAlarm("ALERT_FIRE");
                    if (r == 0) {
                        Application::GetInstance().GetAudioService().PlaySound(Lang::Sounds::OGG_FIREALARM);
                    }
                    g_alarm_voice = false;
                    if (r != -1) {
                        ticks_since_voice = FLAME_REPEAT_TICKS - AI_ALERT_GAP_TICKS;
                    }
#else
                    // Blocks until the voice has played (about 8 seconds). The red eyes stay on meanwhile.
                    g_alarm_voice = true;
                    Application::GetInstance().GetAudioService().PlaySound(Lang::Sounds::OGG_FIREALARM);
                    g_alarm_voice = false;
#endif
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
#endif

#if GAS_ALERT_ENABLED
    // Watches the gas / smoke sensor (MQ module, digital output). Same behaviour as the fire
    // alarm: cuts off the AI and repeats a recorded voice warning until the gas is gone.
    // It never switches the fan or any relay: a relay can make a spark.
    static void GasMonitorTask(void* arg) {
        // MQ sensors need time to heat up and give false readings at first.
        vTaskDelay(pdMS_TO_TICKS(GAS_WARMUP_SECONDS * 1000));
        ESP_LOGI(TAG, "Gas sensor is warm and armed");

        int active_ticks = 0;
        int clear_ticks = 0;
        int ticks_since_voice = 0;
        int64_t last_notify_us = 0;
        bool gas = false;

        while (true) {
            bool detected = gpio_get_level(GAS_GPIO) == GAS_ACTIVE_LEVEL;
            if (detected) {
                active_ticks++;
                clear_ticks = 0;
            } else {
                clear_ticks++;
                active_ticks = 0;
            }

            if (!gas && active_ticks >= GAS_CONFIRM_TICKS) {
                gas = true;
                g_gas_active = true;
                ticks_since_voice = GAS_REPEAT_TICKS;
                last_notify_us = esp_timer_get_time();
                QueueNotify("GAS / SMOKE ALERT", "Gas or smoke detected by the robot's gas sensor. Open windows, avoid flames and switches!",
                            "warning,skull", 5);
                ESP_LOGW(TAG, "GAS or SMOKE detected by the gas sensor");
                if (!g_fire_active) {
                    Application::GetInstance().Schedule([]() {
                        auto& app = Application::GetInstance();
                        app.AbortSpeaking(kAbortReasonNone);
                        app.GetAudioService().ResetDecoder();
                    });
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            } else if (gas && clear_ticks >= GAS_CLEAR_TICKS) {
                gas = false;
                g_gas_active = false;
                ESP_LOGI(TAG, "Gas sensor no longer detects gas");
                QueueNotify("Gas cleared", "The gas sensor no longer detects gas or smoke.", "white_check_mark", 3);
                Application::GetInstance().Schedule([]() {
                    Application::GetInstance().Alert("Info", "Gas gone", "neutral");
                });
            }

            if (gas && detected &&
                esp_timer_get_time() - last_notify_us > (int64_t)NOTIFY_REPEAT_SECONDS * 1000000) {
                last_notify_us = esp_timer_get_time();
                QueueNotify("GAS / SMOKE ALERT (still)", "The gas sensor still detects gas or smoke!",
                            "warning,skull", 5);
            }
            if (gas && detected && !g_fire_active) {
                ticks_since_voice++;
                if (ticks_since_voice >= GAS_REPEAT_TICKS) {
                    ticks_since_voice = 0;
                    Application::GetInstance().Schedule([]() {
                        Application::GetInstance().Alert("GAS!", "GAS / SMOKE DETECTED!", "angry", ALERT_ATTN_SOUND);
                    });
#if AI_ALERT_ENABLED
                    g_alarm_voice = true;
                    int r = AiAlarm("ALERT_GAS");
                    if (r == 0) {
                        Application::GetInstance().GetAudioService().PlaySound(Lang::Sounds::OGG_GASALARM);
                    }
                    g_alarm_voice = false;
                    if (r != -1) {
                        ticks_since_voice = GAS_REPEAT_TICKS - AI_ALERT_GAP_TICKS;
                    }
#else
                    // Blocks until the voice has played. The red eyes stay on meanwhile.
                    g_alarm_voice = true;
                    Application::GetInstance().GetAudioService().PlaySound(Lang::Sounds::OGG_GASALARM);
                    g_alarm_voice = false;
#endif
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
#endif

    // Reads the temperature sensor every few seconds and applies the fan rule that was set by voice.
    // Motion sensor (PIR). While the motion-light mode is on, any movement turns the lamp on and
    // the lamp turns off again after the set number of seconds without movement. Works without the server.
    static void PirTask(void* arg) {
        vTaskDelay(pdMS_TO_TICKS(PIR_WARMUP_SECONDS * 1000));  // the sensor needs time to settle
        ESP_LOGI(TAG, "Motion sensor is ready");
        while (true) {
            bool motion = gpio_get_level(PIR_GPIO) == PIR_ACTIVE_LEVEL;
            g_motion_now = motion;
            if (motion) {
                g_last_motion_us = esp_timer_get_time();
            }
            if (g_away_mode && motion) {
                int64_t now = esp_timer_get_time();
                if (now >= g_away_armed_us) {
                    g_intruder_until_us = now + 60LL * 1000000;
                    int64_t last = g_last_away_notify_us;
                    if (last == 0 || now - last > 120LL * 1000000) {  // at most one message every 2 minutes
                        g_last_away_notify_us = now;
                        QueueNotify("MOTION AT HOME", "The robot saw movement while you are away. Check the house!",
                                    "warning,eyes", 5);
                        ESP_LOGW(TAG, "Away mode: movement seen, phone message queued");
                    }
                }
            }
            if (g_pir_mode && esp_timer_get_time() >= g_pir_armed_us) {
                if (motion && !g_motion_lit) {
                    g_motion_lit = true;
                    RoomLamps(g_motion_lamps_mask, true);
                    ESP_LOGI(TAG, "Motion: room lamps ON");
                } else if (!motion && g_motion_lit &&
                           esp_timer_get_time() - g_last_motion_us > (int64_t)g_pir_off_seconds * 1000000) {
                    g_motion_lit = false;
                    RoomLamps(g_motion_lamps_mask, false);
                    ESP_LOGI(TAG, "No motion for %d s: room lamps OFF", (int)g_pir_off_seconds);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    static void TemperatureTask(void* arg) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        int failures = 0;
        while (true) {
            float t = 0, h = 0;
            bool ok = false;
            int stage = 0;
            for (int attempt = 0; attempt < 3 && !ok; attempt++) {
                ok = ReadAm2302(TEMP_GPIO, &t, &h, &stage);
                if (!ok) {
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
            }
            if (ok) {
                failures = 0;
                g_temp_c = t;
                g_humidity = h;
                g_temp_time_us = esp_timer_get_time();
                ESP_LOGI(TAG, "Temperature %.1f C, humidity %.1f %%", t, h);

                int threshold = g_fan_rule_x10;
                if (threshold > 0) {
                    int t10 = (int)lroundf(t * 10.0f);
                    if (t10 >= threshold && !g_rule_latched) {
                        g_rule_latched = true;
                        RoomSend("fan", true);
                        g_fan_auto_on = true;
                        ESP_LOGW(TAG, "Temperature rule: room fan ON at %.1f C", t);
                        Application::GetInstance().Schedule([t]() {
                            char msg[40];
                            snprintf(msg, sizeof(msg), "Fan ON: %.1f C", t);
                            Application::GetInstance().Alert("Temperature", msg, "neutral");
                        });
                    } else if (t10 <= threshold - TEMP_HYSTERESIS_C * 10 && g_rule_latched) {
                        g_rule_latched = false;
                        if (g_fan_auto_on) {
                            RoomSend("fan", false);
                            ESP_LOGI(TAG, "Temperature rule: room fan OFF at %.1f C", t);
                            Application::GetInstance().Schedule([t]() {
                                char msg[40];
                                snprintf(msg, sizeof(msg), "Fan OFF: %.1f C", t);
                                Application::GetInstance().Alert("Temperature", msg, "neutral");
                            });
                        }
                        g_fan_auto_on = false;
                    }
                }
            } else if (++failures % 5 == 0) {
                ESP_LOGE(TAG, "Temperature sensor error, stage %d (1 = no answer, 2 = line stuck low, 4/5 = bad data). Check the wiring on GPIO %d", stage, (int)TEMP_GPIO);
            }
            vTaskDelay(pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS));
        }
    }

    // 物联网初始化，逐步迁移到 MCP 协议
    void InitializeTools() {
        // Lamp relay (own tools, so the motion light can switch it and the state stays correct)
        gpio_config_t lamp_config = {
            .pin_bit_mask = (1ULL << LAMP_GPIO),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&lamp_config));
        SetLamp(false);

        // Fan relay
        gpio_config_t fan_config = {
            .pin_bit_mask = (1ULL << FAN_GPIO),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&fan_config));
        gpio_set_level(FAN_GPIO, 0);

        auto& mcp_server = McpServer::GetInstance();
        // Load the temperature rule that was saved by voice before the last restart.
        {
            Settings settings("temp_rule", false);
            int x10 = settings.GetInt("fan_x10", 0);
            if (x10 == 0) {
                x10 = settings.GetInt("fan_on_at", 0) * 10;   // rule saved by an older firmware (whole degrees)
            }
            g_fan_rule_x10 = x10;
        }

        mcp_server.AddTool("self.lamp.get_state", "Get the power state of the lamp", PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                return std::string(g_lamp_on ? "{\"power\": true}" : "{\"power\": false}");
            });
        mcp_server.AddTool("self.lamp.turn_on", "Turn on the lamp", PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                SetLamp(true);
                return true;
            });
        mcp_server.AddTool("self.lamp.turn_off", "Turn off the lamp", PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                SetLamp(false);
                return true;
            });

        // Motion sensor (PIR) and the voice-set motion-light mode
        gpio_config_t pir_config = {
            .pin_bit_mask = (1ULL << PIR_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&pir_config));
        {
            Settings settings("motion_light", false);
            g_pir_mode = settings.GetInt("on", 0) != 0;
            g_pir_off_seconds = settings.GetInt("secs", PIR_DEFAULT_OFF_SECONDS);
            g_motion_lamps_mask = settings.GetInt("lamps", 1);
        }
        g_last_motion_us = esp_timer_get_time();
        mcp_server.AddTool("self.motion_light.turn_on",
            "Turn on the motion-light mode for the ROOM lamps (the lamps at the wall switch): whenever the motion "
            "sensor sees movement, the chosen room lamps turn on by themselves, and when there is no movement for the "
            "given number of seconds, they turn off again. The mode starts working about 10 seconds after the command. "
            "Use it when the user says things like 'turn on lamp 2 whenever you sense motion'. "
            "lamps = which room lamps the user asked for: '1', '2', '3', a list like '1,3', or 'all' for all three lamps. "
            "seconds = how long without movement before the lamps turn off (default 60).",
            PropertyList({Property("lamps", kPropertyTypeString, "1"),
                          Property("seconds", kPropertyTypeInteger, PIR_DEFAULT_OFF_SECONDS, 5, 3600)}),
            [](const PropertyList& properties) -> ReturnValue {
                int secs = properties["seconds"].value<int>();
                int mask = ParseLampMask(properties["lamps"].value<std::string>());
                g_pir_off_seconds = secs;
                g_motion_lamps_mask = mask;
                g_motion_lit = false;
                g_last_motion_us = esp_timer_get_time();
                g_pir_armed_us = esp_timer_get_time() + (int64_t)PIR_START_DELAY_SECONDS * 1000000;
                g_pir_mode = true;
                {
                    Settings settings("motion_light", true);
                    settings.SetInt("on", 1);
                    settings.SetInt("secs", secs);
                    settings.SetInt("lamps", mask);
                }
                char buf[160];
                snprintf(buf, sizeof(buf),
                         "{\"success\": true, \"motion_light\": true, \"lamps\": \"%s\", \"off_after_seconds\": %d, "
                         "\"starts_in_seconds\": %d}",
                         LampMaskText(mask).c_str(), secs, (int)PIR_START_DELAY_SECONDS);
                return std::string(buf);
            });
        mcp_server.AddTool("self.motion_light.turn_off",
            "Turn off the motion-light mode. The room lamps then stay as they are now.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                g_pir_mode = false;
                g_motion_lit = false;
                {
                    Settings settings("motion_light", true);
                    settings.SetInt("on", 0);
                }
                return std::string("{\"success\": true, \"motion_light\": false}");
            });
        mcp_server.AddTool("self.motion_light.get_state",
            "Get the motion-light mode (on or off), which room lamps it uses, how many seconds without movement turn "
            "them off, and whether movement is detected right now",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                char buf[192];
                snprintf(buf, sizeof(buf),
                         "{\"motion_light\": %s, \"lamps\": \"%s\", \"off_after_seconds\": %d, \"motion_detected_now\": %s}",
                         g_pir_mode ? "true" : "false", LampMaskText(g_motion_lamps_mask).c_str(),
                         (int)g_pir_off_seconds, g_motion_now ? "true" : "false");
                return std::string(buf);
            });
        // ---- Away mode: watch the house with the motion sensor ----
        {
            Settings settings("away", false);
            if (settings.GetInt("on", 0) != 0) {
                g_away_mode = true;
                g_away_armed_us = esp_timer_get_time() + (int64_t)AWAY_LEAVE_DELAY_SECONDS * 1000000;
            }
        }
        mcp_server.AddTool("self.away.start",
            "Start the away (guard) mode. Use it when the user says he is going out, e.g. 'I am leaving now'. "
            "The robot waits about 10 seconds so the user can leave, then watches with the motion sensor. "
            "If it sees movement it sends a message to the user's phone and its eyes blink red.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                g_away_armed_us = esp_timer_get_time() + (int64_t)AWAY_LEAVE_DELAY_SECONDS * 1000000;
                g_intruder_until_us = 0;
                g_last_away_notify_us = 0;
                g_away_mode = true;
                {
                    Settings settings("away", true);
                    settings.SetInt("on", 1);
                }
                return std::string("{\"success\": true, \"away_mode\": true, \"watch_starts_in_seconds\": " +
                                   std::to_string(AWAY_LEAVE_DELAY_SECONDS) + "}");
            });
        mcp_server.AddTool("self.away.stop",
            "Stop the away (guard) mode. Use it when the user says he is back, e.g. 'I am back'.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                g_away_mode = false;
                g_intruder_until_us = 0;
                {
                    Settings settings("away", true);
                    settings.SetInt("on", 0);
                }
                return std::string("{\"success\": true, \"away_mode\": false}");
            });
        mcp_server.AddTool("self.away.get_state",
            "Get whether the away (guard) mode is on and whether movement was seen since it started",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                bool seen = g_last_away_notify_us != 0;
                return std::string(std::string("{\"away_mode\": ") + (g_away_mode ? "true" : "false") +
                                   ", \"movement_seen\": " + (seen ? "true" : "false") + "}");
            });

        // ---- Eyes: colours and effects by voice ----
        mcp_server.AddTool("self.eyes.show",
            "Change the colour of the robot's eyes (red, green and blue LEDs) for a few seconds. "
            "Use it for requests like 'play the eyes game', 'make your eyes red', 'blink', 'rainbow'. "
            "red, green, blue = brightness 0 to 100 each (mix them: red+green = yellow, green+blue = cyan, "
            "red+blue = magenta, all three = white). effect = solid, blink, breathe or rainbow "
            "(rainbow ignores the colour). seconds = how long (1 to 60, default 10); afterwards the eyes "
            "return to their normal job. For a long game call it again every few seconds with new colours.",
            PropertyList({Property("red", kPropertyTypeInteger, 0, 0, 100),
                          Property("green", kPropertyTypeInteger, 0, 0, 100),
                          Property("blue", kPropertyTypeInteger, 100, 0, 100),
                          Property("effect", kPropertyTypeString, "solid"),
                          Property("seconds", kPropertyTypeInteger, 10, 1, 60)}),
            [](const PropertyList& properties) -> ReturnValue {
                g_eye_r = properties["red"].value<int>();
                g_eye_g = properties["green"].value<int>();
                g_eye_b = properties["blue"].value<int>();
                std::string e = properties["effect"].value<std::string>();
                g_eye_effect = e == "blink" ? 1 : e == "breathe" ? 2 : e == "rainbow" ? 3 : 0;
                int secs = properties["seconds"].value<int>();
                g_eye_override_until_us = esp_timer_get_time() + (int64_t)secs * 1000000;
                return std::string("{\"success\": true}");
            });
        mcp_server.AddTool("self.eyes.reset",
            "Return the eyes to their normal behaviour at once (after a colour game or effect).",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                g_eye_override_until_us = 0;
                return std::string("{\"success\": true}");
            });
        g_room_queue = xQueueCreate(16, sizeof(RoomCmd));
        xTaskCreate(RoomTask, "room_cmd", 6144, nullptr, 2, nullptr);
        xTaskCreate(PirTask, "pir_monitor", 3072, nullptr, 3, nullptr);

        mcp_server.AddTool("self.fan.get_state", "Get the power state of the fan", PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                return std::string(g_fan_on ? "{\"power\": true}" : "{\"power\": false}");
            });
        mcp_server.AddTool("self.fan.turn_on", "Turn on the fan", PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                g_fan_auto_on = false;  // manual control: the temperature rule will not turn it off
                SetFan(true);
                return true;
            });
        mcp_server.AddTool("self.fan.turn_off", "Turn off the fan", PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                g_fan_auto_on = false;
                SetFan(false);
                return true;
            });

#if NOTIFY_ENABLED
        g_notify_queue = xQueueCreate(4, sizeof(NotifyMessage));
        xTaskCreate(NotifyTask, "notify", 6144, nullptr, 2, nullptr);
        mcp_server.AddTool("self.notify.send_test",
            "Send a test notification to the owner's phone to check that phone alerts work",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                QueueNotify("Robot test", "This is a test message from your robot. Phone alerts work!",
                            "robot", 3);
                return std::string("{\"success\": true, \"note\": \"the test message is being sent\"}");
            });
#endif

        // Temperature sensor (AM2302 / DHT22) and the voice-set fan rule
        mcp_server.AddTool("self.temperature.get_state",
            "Get the current room temperature in degrees Celsius and the humidity in percent",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                float t = g_temp_c;
                float h = g_humidity;
                if (std::isnan(t) || esp_timer_get_time() - g_temp_time_us > 30LL * 1000000) {
                    return std::string("{\"error\": \"temperature sensor is not ready or not connected\"}");
                }
                char buf[96];
                snprintf(buf, sizeof(buf), "{\"temperature_celsius\": %.1f, \"humidity_percent\": %.1f}", t, h);
                return std::string(buf);
            });
        mcp_server.AddTool("self.temperature.set_fan_rule",
            "Set an automatic fan rule for the ROOM fan (the fan at the wall switch): when the room temperature "
            "reaches or goes above the given temperature in degrees Celsius, the fan turns on by itself, and it "
            "turns off again when the room has cooled down by a degree. This replaces any earlier rule. "
            "The temperature may have a decimal, written as text, for example '38.3'. Use it when the user says "
            "things like 'turn on the fan when the temperature reaches 38.3'.",
            PropertyList({Property("temperature", kPropertyTypeString, "38")}),
            [](const PropertyList& properties) -> ReturnValue {
                float v = strtof(properties["temperature"].value<std::string>().c_str(), nullptr);
                if (!(v >= 10.0f && v <= 60.0f)) {
                    return std::string("{\"success\": false, \"error\": \"temperature must be between 10 and 60\"}");
                }
                int x10 = (int)lroundf(v * 10.0f);
                g_fan_rule_x10 = x10;
                g_rule_latched = false;
                {
                    Settings settings("temp_rule", true);
                    settings.SetInt("fan_x10", x10);
                }
                char buf[96];
                snprintf(buf, sizeof(buf), "{\"success\": true, \"fan_turns_on_at_celsius\": %.1f}", x10 / 10.0f);
                return std::string(buf);
            });
        mcp_server.AddTool("self.temperature.get_fan_rule",
            "Get the automatic fan rule that is currently set (the temperature at which the room fan turns on)",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                int x10 = g_fan_rule_x10;
                if (x10 <= 0) {
                    return std::string("{\"rule_active\": false}");
                }
                char buf[96];
                snprintf(buf, sizeof(buf), "{\"rule_active\": true, \"fan_turns_on_at_celsius\": %.1f}", x10 / 10.0f);
                return std::string(buf);
            });
        mcp_server.AddTool("self.temperature.clear_fan_rule",
            "Remove the automatic temperature rule for the fan",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                g_fan_rule_x10 = 0;
                g_rule_latched = false;
                {
                    Settings settings("temp_rule", true);
                    settings.SetInt("fan_x10", 0);
                    settings.SetInt("fan_on_at", 0);
                }
                return std::string("{\"success\": true, \"rule_active\": false}");
            });
        xTaskCreate(TemperatureTask, "temp_monitor", 4096, nullptr, 3, nullptr);

        // Flame sensor (digital output, read on demand)
        gpio_config_t flame_config = {
            .pin_bit_mask = (1ULL << FLAME_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&flame_config));

        mcp_server.AddTool("self.flame.get_state",
            "Check the flame sensor. Returns whether a flame or fire is currently detected",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                bool detected = gpio_get_level(FLAME_GPIO) == FLAME_ACTIVE_LEVEL;
                return detected ? "{\"flame_detected\": true}" : "{\"flame_detected\": false}";
            });

#if FLAME_ALERT_ENABLED
        xTaskCreate(FlameMonitorTask, "flame_monitor", 4096, nullptr, 3, nullptr);
#endif

#if GAS_ALERT_ENABLED
        gpio_config_t gas_config = {
            .pin_bit_mask = (1ULL << GAS_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&gas_config));
        mcp_server.AddTool("self.gas.get_state",
            "Check the gas and smoke sensor. Returns whether gas or smoke is currently detected",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                bool detected = gpio_get_level(GAS_GPIO) == GAS_ACTIVE_LEVEL;
                return std::string(detected ? "{\"gas_detected\": true}" : "{\"gas_detected\": false}");
            });
        xTaskCreate(GasMonitorTask, "gas_monitor", 4096, nullptr, 3, nullptr);
#endif
    }

public:
    CompactWifiBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(TOUCH_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        InitializeTools();
    }

    virtual Led* GetLed() override {
        static EyeLeds led;
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(CompactWifiBoard);
