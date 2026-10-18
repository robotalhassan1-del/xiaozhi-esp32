#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
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
static std::atomic<int>  g_fan_rule_celsius{0};     // 0 = no rule
static std::atomic<bool> g_rule_latched{false};     // the rule already acted in this hot period
static std::atomic<float> g_temp_c{NAN};
static std::atomic<float> g_humidity{NAN};
static std::atomic<int64_t> g_temp_time_us{0};
static std::atomic<bool> g_lamp_on{false};
static std::atomic<bool> g_pir_mode{false};         // motion-light mode: motion turns the lamp on, calm turns it off
static std::atomic<int>  g_pir_off_seconds{60};     // lamp goes off after this many seconds without motion
static std::atomic<int64_t> g_last_motion_us{0};
static std::atomic<bool> g_motion_now{false};


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
                    int r = AiAlarm("ALERT_FIRE");
                    if (r == 0) {
                        Application::GetInstance().GetAudioService().PlaySound(Lang::Sounds::OGG_FIREALARM);
                    }
                    if (r != -1) {
                        ticks_since_voice = FLAME_REPEAT_TICKS - AI_ALERT_GAP_TICKS;
                    }
#else
                    // Blocks until the voice has played (about 8 seconds).
                    Application::GetInstance().GetAudioService().PlaySound(Lang::Sounds::OGG_FIREALARM);
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
                    int r = AiAlarm("ALERT_GAS");
                    if (r == 0) {
                        Application::GetInstance().GetAudioService().PlaySound(Lang::Sounds::OGG_GASALARM);
                    }
                    if (r != -1) {
                        ticks_since_voice = GAS_REPEAT_TICKS - AI_ALERT_GAP_TICKS;
                    }
#else
                    // Blocks until the voice has played.
                    Application::GetInstance().GetAudioService().PlaySound(Lang::Sounds::OGG_GASALARM);
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
            if (g_pir_mode) {
                if (motion && !g_lamp_on) {
                    SetLamp(true);
                    ESP_LOGI(TAG, "Motion: lamp ON");
                } else if (!motion && g_lamp_on &&
                           esp_timer_get_time() - g_last_motion_us > (int64_t)g_pir_off_seconds * 1000000) {
                    SetLamp(false);
                    ESP_LOGI(TAG, "No motion for %d s: lamp OFF", (int)g_pir_off_seconds);
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

                int threshold = g_fan_rule_celsius;
                if (threshold > 0) {
                    if (t >= threshold && !g_rule_latched) {
                        g_rule_latched = true;
                        if (!g_fan_on) {
                            SetFan(true);
                            g_fan_auto_on = true;
                            ESP_LOGW(TAG, "Temperature rule: fan ON at %.1f C", t);
                            Application::GetInstance().Schedule([t]() {
                                char msg[40];
                                snprintf(msg, sizeof(msg), "Fan ON: %.1f C", t);
                                Application::GetInstance().Alert("Temperature", msg, "neutral");
                            });
                        }
                    } else if (t <= threshold - TEMP_HYSTERESIS_C && g_rule_latched) {
                        g_rule_latched = false;
                        if (g_fan_auto_on && g_fan_on) {
                            SetFan(false);
                            ESP_LOGI(TAG, "Temperature rule: fan OFF at %.1f C", t);
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
            g_fan_rule_celsius = settings.GetInt("fan_on_at", 0);
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
        }
        g_last_motion_us = esp_timer_get_time();
        mcp_server.AddTool("self.motion_light.turn_on",
            "Turn on the motion-light mode: whenever the motion sensor sees movement, the lamp turns on by "
            "itself, and when there is no movement for the given number of seconds, the lamp turns off again. "
            "Use it when the user says things like 'turn on the lamp whenever you sense motion'. "
            "Optional: seconds = how long without movement before the lamp turns off (default 60).",
            PropertyList({Property("seconds", kPropertyTypeInteger, PIR_DEFAULT_OFF_SECONDS, 5, 3600)}),
            [](const PropertyList& properties) -> ReturnValue {
                int secs = properties["seconds"].value<int>();
                g_pir_off_seconds = secs;
                g_last_motion_us = esp_timer_get_time();
                g_pir_mode = true;
                {
                    Settings settings("motion_light", true);
                    settings.SetInt("on", 1);
                    settings.SetInt("secs", secs);
                }
                char buf[96];
                snprintf(buf, sizeof(buf), "{\"success\": true, \"motion_light\": true, \"off_after_seconds\": %d}", secs);
                return std::string(buf);
            });
        mcp_server.AddTool("self.motion_light.turn_off",
            "Turn off the motion-light mode. The lamp then obeys only direct commands and stays as it is now.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                g_pir_mode = false;
                {
                    Settings settings("motion_light", true);
                    settings.SetInt("on", 0);
                }
                return std::string("{\"success\": true, \"motion_light\": false}");
            });
        mcp_server.AddTool("self.motion_light.get_state",
            "Get the motion-light mode (on or off), how many seconds without movement turn the lamp off, "
            "and whether movement is detected right now",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                char buf[128];
                snprintf(buf, sizeof(buf),
                         "{\"motion_light\": %s, \"off_after_seconds\": %d, \"motion_detected_now\": %s}",
                         g_pir_mode ? "true" : "false", (int)g_pir_off_seconds, g_motion_now ? "true" : "false");
                return std::string(buf);
            });
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
            "Set an automatic fan rule: when the room temperature reaches or goes above the given "
            "temperature in degrees Celsius, the fan turns on by itself, and it turns off again when the "
            "room has cooled down by a degree. This replaces any earlier rule. Use it when the user says "
            "things like 'turn on the fan when the temperature goes above 39'.",
            PropertyList({Property("temperature", kPropertyTypeInteger, 10, 60)}),
            [](const PropertyList& properties) -> ReturnValue {
                int threshold = properties["temperature"].value<int>();
                g_fan_rule_celsius = threshold;
                g_rule_latched = false;
                {
                    Settings settings("temp_rule", true);
                    settings.SetInt("fan_on_at", threshold);
                }
                char buf[96];
                snprintf(buf, sizeof(buf), "{\"success\": true, \"fan_turns_on_at_celsius\": %d}", threshold);
                return std::string(buf);
            });
        mcp_server.AddTool("self.temperature.get_fan_rule",
            "Get the automatic fan rule that is currently set (the temperature at which the fan turns on)",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                int threshold = g_fan_rule_celsius;
                if (threshold <= 0) {
                    return std::string("{\"rule_active\": false}");
                }
                char buf[96];
                snprintf(buf, sizeof(buf), "{\"rule_active\": true, \"fan_turns_on_at_celsius\": %d}", threshold);
                return std::string(buf);
            });
        mcp_server.AddTool("self.temperature.clear_fan_rule",
            "Remove the automatic temperature rule for the fan",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                g_fan_rule_celsius = 0;
                g_rule_latched = false;
                {
                    Settings settings("temp_rule", true);
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
        static SingleLed led(BUILTIN_LED_GPIO);
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
