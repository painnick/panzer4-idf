// RC Tank platform - Bluepad32 + rctank (README.md 기능)
// C 함수 형태 구현 (Agent.md 코딩 규칙)

#include <string.h>

#include <platform/uni_platform.h>
#include <uni.h>
#include "controller/uni_controller.h"
#include "controller/uni_gamepad.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rctank.h"
#include "rctank_dfplayer.h"
#include "rctank_led.h"
#include "rctank_motor.h"
#include "rctank_servo.h"
#include "rctank_storage.h"

#define AXIS_MAX 512
#define AXIS_DEADZONE 60
#define DEBOUNCE_MS 100
#define TURN_SLOWDOWN_FACTOR 60   /* 대회전 시 전체 속도 비율 (60 = 60%) */
#define HEADLIGHT_DEBOUNCE_MS 400 /* 헤드라이트 토글 최소 간격 (둔감) */
#define GUN_PULL_MS 100           /* 1단계: 포신 당기기 지연 시간 */
#define GUN_TRACK_MS 200          /* 2단계: 트랙 반동 시간 */
#define GUN_RETURN_WAIT_MS 200    /* 3단계: 트랙 정지 후 포신 복구 대기 시간 */
#define GUN_DELAY_MS 400          /* 포신: MP3 재생 요청 후 서보/LED/럼블 지연 (DFPlayer 지연 보정) */
#define MG_FIRE_MS 500            /* 기관총 발사 시간 (LED 깜빡임) */
#define MG_LED_BLINK_MS 75        /* 기관총 LED 깜빡임 주기 */
#define MG_DELAY_MS 300           /* 기관총: MP3 재생 요청 후 LED/럼블 지연 */
#define TURRET_SPEED 511          /* 터렛 회전 속도 (최대, 100% 듀티) */
#define SELECT_START_HOLD_MS 3000
#define RECOIL_POWER 450 /* 반동 속도 */

typedef struct my_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;
} my_platform_instance_t;

static void trigger_event_on_gamepad(uni_hid_device_t* d);
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);

static int64_t last_y_ms = 0;
static int64_t last_l1_ms = 0;
static int64_t last_r1_ms = 0;
static int64_t select_start_pressed_at = 0;
static int64_t recoil_end_time = 0; /* 반동 종료 시간 */
static uint8_t prev_buttons = 0;
static uint8_t prev_misc = 0;
static esp_timer_handle_t gun_timer = NULL;
static esp_timer_handle_t gun_delayed_start_timer = NULL;
static uni_hid_device_t* gun_delayed_device = NULL; /* 500ms 후 럼블용 */
static esp_timer_handle_t restart_timer = NULL;
static esp_timer_handle_t mg_blink_timer = NULL;
static esp_timer_handle_t mg_stop_timer = NULL;
static esp_timer_handle_t mg_delayed_start_timer = NULL;
static uni_hid_device_t* mg_delayed_device = NULL; /* 500ms 후 럼블용 */
static int mg_led_toggle = 0;

static esp_timer_handle_t gun_track_timer = NULL;
static esp_timer_handle_t gun_return_timer = NULL;
static esp_timer_handle_t gun_detach_timer = NULL;
static esp_timer_handle_t waiting_idle_timer = NULL; /* 연결 대기 중 30초 주기 알림용 */

static void waiting_idle_cb(void* arg) {
    (void)arg;
    rctank_dfplayer_play(RCTANK_DFPLAYER_TRACK_IDLE);
}

static void gun_detach_timer_cb(void* arg) {
    (void)arg;
    /* 서보 비활성화 (토크 해제) */
    rctank_servo_gun_enable(false);
}

static void gun_return_timer_cb(void* arg) {
    (void)arg;
    rctank_led_gun_set(0);
    rctank_servo_gun_set_degree(RCTANK_SERVO_GUN_DEG_REST);

    /* 복귀 완료 후 서보 끄기 (500ms 후) */
    esp_timer_stop(gun_detach_timer);
    esp_timer_start_once(gun_detach_timer, 500 * 1000);
}

static void gun_fire_timer_cb(void* arg) {
    (void)arg;
    /* 2단계 종료: 트랙 정지 */
    rctank_motor_left_track_set_immediate(0);
    rctank_motor_right_track_set_immediate(0);

    /* 3단계 시작: 포신 복구 지연 */
    esp_timer_start_once(gun_return_timer, GUN_RETURN_WAIT_MS * 1000);
}

static void gun_track_timer_cb(void* arg) {
    (void)arg;
    /* 2단계 시작: 트랙 밀기 */
    rctank_motor_left_track_set_immediate(-RECOIL_POWER);
    rctank_motor_right_track_set_immediate(-RECOIL_POWER);

    /* 트랙 정지 예약 */
    esp_timer_start_once(gun_timer, GUN_TRACK_MS * 1000);
}

/* 포신: MP3 재생 요청 후 GUN_DELAY_MS 지난 뒤 서보/LED/럼블 시작 (DFPlayer 지연 보정) */
static void gun_delayed_start_cb(void* arg) {
    (void)arg;

    /* 1단계 시작: 포신 당기기 */
    int64_t now_ms = esp_timer_get_time() / 1000;
    /* 리코일 제어권 잠금: 포신 당기기 + 트랙 밀기 시간 동안 */
    recoil_end_time = now_ms + GUN_PULL_MS + GUN_TRACK_MS;

    rctank_led_gun_set(1);
    rctank_servo_gun_set_degree(60);

    uni_hid_device_t* d = gun_delayed_device;
    gun_delayed_device = NULL;
    if (d != NULL && d->report_parser.play_dual_rumble != NULL)
        d->report_parser.play_dual_rumble(d, 0, 400, 150, 255);

    /* 2단계(트랙 밀기) 지연 시작 */
    esp_timer_stop(gun_track_timer);
    esp_timer_start_once(gun_track_timer, GUN_PULL_MS * 1000);
}

static void delayed_restart_cb(void* arg) {
    (void)arg;
    rctank_storage_erase_and_restart();
}

static void mg_blink_timer_cb(void* arg) {
    (void)arg;
    mg_led_toggle = !mg_led_toggle;
    rctank_led_mg_set(mg_led_toggle);
}

static void mg_stop_timer_cb(void* arg) {
    (void)arg;
    esp_timer_stop(mg_blink_timer);
    rctank_led_mg_set(0);
}

/* MP3 재생 요청 후 500ms 지난 뒤 LED 깜빡임 + 럼블 시작 (DFPlayer 지연 보정) */
static void mg_delayed_start_cb(void* arg) {
    (void)arg;
    uni_hid_device_t* d = mg_delayed_device;
    mg_delayed_device = NULL;
    if (d != NULL && d->report_parser.play_dual_rumble != NULL)
        d->report_parser.play_dual_rumble(d, 0, 300, 150, 200);
    esp_timer_stop(mg_blink_timer);
    esp_timer_stop(mg_stop_timer);
    mg_led_toggle = 0;
    rctank_led_mg_set(1);
    esp_timer_start_periodic(mg_blink_timer, MG_LED_BLINK_MS * 1000);
    esp_timer_start_once(mg_stop_timer, MG_FIRE_MS * 1000);
}

static void my_platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    logi("custom: init()\n");

    esp_err_t err = rctank_init();
    if (err != ESP_OK) {
        loge("rctank_init failed: %s\n", esp_err_to_name(err));
        return;
    }

    const esp_timer_create_args_t gun_timer_args = {
        .callback = &gun_fire_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gun_fire",
    };
    esp_timer_create(&gun_timer_args, &gun_timer);

    const esp_timer_create_args_t gun_detach_timer_args = {
        .callback = &gun_detach_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gun_detach",
    };
    esp_timer_create(&gun_detach_timer_args, &gun_detach_timer);

    const esp_timer_create_args_t gun_track_timer_args = {
        .callback = &gun_track_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gun_track",
    };
    esp_timer_create(&gun_track_timer_args, &gun_track_timer);

    const esp_timer_create_args_t gun_return_timer_args = {
        .callback = &gun_return_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gun_return",
    };
    esp_timer_create(&gun_return_timer_args, &gun_return_timer);

    const esp_timer_create_args_t gun_delayed_start_args = {
        .callback = &gun_delayed_start_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gun_delayed",
    };
    esp_timer_create(&gun_delayed_start_args, &gun_delayed_start_timer);

    const esp_timer_create_args_t restart_timer_args = {
        .callback = &delayed_restart_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "restart",
    };
    esp_timer_create(&restart_timer_args, &restart_timer);

    const esp_timer_create_args_t mg_blink_args = {
        .callback = &mg_blink_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "mg_blink",
    };
    esp_timer_create(&mg_blink_args, &mg_blink_timer);

    const esp_timer_create_args_t mg_stop_args = {
        .callback = &mg_stop_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "mg_stop",
    };
    esp_timer_create(&mg_stop_args, &mg_stop_timer);

    const esp_timer_create_args_t mg_delayed_start_args = {
        .callback = &mg_delayed_start_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "mg_delayed",
    };
    esp_timer_create(&mg_delayed_start_args, &mg_delayed_start_timer);

    const esp_timer_create_args_t waiting_idle_args = {
        .callback = &waiting_idle_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "waiting_idle",
    };
    esp_timer_create(&waiting_idle_args, &waiting_idle_timer);
}

static void my_platform_on_init_complete(void) {
    logi("custom: on_init_complete()\n");
    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);
    /* 저장된 페어링 정보 유지 → 다음 연결 시 빠른 자동 재연결 (uni_bt_del_keys_unsafe 호출 안 함) */

    esp_err_t ret = rctank_dfplayer_init();
    if (ret != ESP_OK)
        return;

    uint8_t vol = rctank_storage_volume_get();
    rctank_dfplayer_set_volume(vol);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 초기 1회 재생 후 30초 주기 타이머 시작 */
    rctank_dfplayer_play(RCTANK_DFPLAYER_TRACK_IDLE);
    esp_timer_start_periodic(waiting_idle_timer, 30 * 1000 * 1000);
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    (void)addr;
    (void)name;
    (void)rssi;
    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
        logi("Ignoring keyboard\n");
        return UNI_ERROR_IGNORE_DEVICE;
    }
    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    logi("custom: device connected: %p\n", d);
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    logi("custom: device disconnected: %p\n", d);
    rctank_motor_left_track_set_immediate(0);
    rctank_motor_right_track_set_immediate(0);
    rctank_motor_turret_set(0);
    /* 연결 해제 시 다시 30초 주기 재생 시작 */
    rctank_dfplayer_play(RCTANK_DFPLAYER_TRACK_IDLE);
    esp_timer_start_periodic(waiting_idle_timer, 30 * 1000 * 1000);
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    logi("custom: device ready: %p\n", d);
    my_platform_instance_t* ins = get_my_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;

    esp_timer_stop(waiting_idle_timer); /* 연결 시 주기적 소리 중지 */
    rctank_dfplayer_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    rctank_dfplayer_play(RCTANK_DFPLAYER_TRACK_CONNECT);
    /* 더 이상 IDLE로 복구하지 않음 (운용 중 정적 유지) */

    trigger_event_on_gamepad(d);
    if (d->report_parser.play_dual_rumble != NULL)
        d->report_parser.play_dual_rumble(d, 0, 400, 128, 200);
    return UNI_ERROR_SUCCESS;
}

static int32_t clamp_axis(int32_t v) {
    if (v > 511)
        return 511;
    if (v < -512)
        return -512;
    return v;
}

static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD)
        return;

    uni_gamepad_t* gp = &ctl->gamepad;
    int64_t now_ms = esp_timer_get_time() / 1000;

    /* 트랙: 좌측 스틱 Y = 좌측 트랙, 우측 스틱 Y = 우측 트랙 (위로 = 전진) */
    int32_t ly = clamp_axis(gp->axis_y);
    int32_t ry = clamp_axis(gp->axis_ry);
    if (ly > -AXIS_DEADZONE && ly < AXIS_DEADZONE)
        ly = 0;
    if (ry > -AXIS_DEADZONE && ry < AXIS_DEADZONE)
        ry = 0;

#if defined(CONFIG_LOG_DEFAULT_LEVEL) && CONFIG_LOG_DEFAULT_LEVEL >= 4
    static int64_t last_log_time = 0;
    if (now_ms - last_log_time >= 500) {
        last_log_time = now_ms;
        logi("Raw Y: %d, RY: %d | Clamped ly: %d, ry: %d\n", gp->axis_y, gp->axis_ry, ly, ry);
    }
#endif

    /* 리코일 중이 아닐 때만 스틱 값으로 트랙 제어 */
    if (now_ms >= recoil_end_time) {
        int32_t turn_diff = (ly > ry) ? (ly - ry) : (ry - ly);
        int32_t left_val = -ly;
        int32_t right_val = -ry;
        if (turn_diff > AXIS_DEADZONE * 2) {
            int32_t reduction = 100 - ((turn_diff - AXIS_DEADZONE * 2) * (100 - TURN_SLOWDOWN_FACTOR)) / (AXIS_MAX * 2 - AXIS_DEADZONE * 2);
            left_val = left_val * reduction / 100;
            right_val = right_val * reduction / 100;
        }
        rctank_motor_left_track_set(left_val);
        rctank_motor_right_track_set(right_val);
    }

    /* 터렛: D-PAD 좌우 */
    int32_t turret = 0;
    if (gp->dpad & DPAD_LEFT)
        turret = -TURRET_SPEED;
    if (gp->dpad & DPAD_RIGHT)
        turret = TURRET_SPEED;
    rctank_motor_turret_set(turret);

    /* 포 마운트: D-PAD 상하 (서보 SG90, GPIO 13, 범위 80~110도, 초기 90도) */
    static float s_elevation_deg = RCTANK_SERVO_ELEVATION_DEG_REST;
    static int64_t s_last_elevation_time_ms = 0;

    if (s_last_elevation_time_ms == 0) {
        s_last_elevation_time_ms = now_ms;
    }
    int64_t dt_ms = now_ms - s_last_elevation_time_ms;
    s_last_elevation_time_ms = now_ms;

    if (dt_ms < 0 || dt_ms > 200) {
        dt_ms = 20; // 비정상적인 시간 간격 보정
    }

    float elevation_change = 0;
    if (gp->dpad & DPAD_UP) {
        elevation_change = (10.0f / 1000.0f) * dt_ms; // 초당 10도 증가
    } else if (gp->dpad & DPAD_DOWN) {
        elevation_change = -(10.0f / 1000.0f) * dt_ms; // 초당 10도 감소
    }

    if (elevation_change != 0) {
        s_elevation_deg += elevation_change;
        if (s_elevation_deg < RCTANK_SERVO_ELEVATION_MIN)
            s_elevation_deg = RCTANK_SERVO_ELEVATION_MIN;
        if (s_elevation_deg > RCTANK_SERVO_ELEVATION_MAX)
            s_elevation_deg = RCTANK_SERVO_ELEVATION_MAX;

        rctank_servo_elevation_set_degree((int)s_elevation_deg);
    }

    /* B: 포신 발사 (MP3 즉시 재생 요청, 500ms 후 서보/LED/럼블) */
    if (gp->buttons & BUTTON_B) {
        if (!(prev_buttons & BUTTON_B)) {
            /* 발사 시퀀스 시작 전에 서보 연결 (토크 주입) */
            rctank_servo_gun_enable(true);

            rctank_dfplayer_play(RCTANK_DFPLAYER_TRACK_GUN);

            gun_delayed_device = d;
            esp_timer_stop(gun_delayed_start_timer);
            esp_timer_start_once(gun_delayed_start_timer, GUN_DELAY_MS * 1000);
        }
    }

    /* A: 기관총 (MP3 즉시 재생 요청, 500ms 후 LED 깜빡임 + 럼블) */
    if (gp->buttons & BUTTON_A) {
        if (!(prev_buttons & BUTTON_A)) {
            rctank_dfplayer_play(RCTANK_DFPLAYER_TRACK_MG);
            mg_delayed_device = d;
            esp_timer_stop(mg_delayed_start_timer);
            esp_timer_start_once(mg_delayed_start_timer, MG_DELAY_MS * 1000);
        }
    }

    /* Y: 헤드라이트 토글 (눌렀을 때 한 번만, 최소 400ms 간격) */
    if (gp->buttons & BUTTON_Y) {
        if (!(prev_buttons & BUTTON_Y) && (now_ms - last_y_ms >= HEADLIGHT_DEBOUNCE_MS)) {
            last_y_ms = now_ms;
            int on = !rctank_led_headlight_get();
            rctank_led_headlight_set(on);
        }
    }

    /* L1: 볼륨 감소 (100ms 간격), 뗐을 때 NVS 저장 */
    if (gp->buttons & BUTTON_SHOULDER_L) {
        if (now_ms - last_l1_ms >= DEBOUNCE_MS) {
            last_l1_ms = now_ms;
            uint8_t v = rctank_storage_volume_get();
            if (v > RCTANK_VOLUME_MIN) {
                v--;
                rctank_storage_volume_set(v);
                rctank_dfplayer_set_volume(v);
            }
        }
    } else {
        if (prev_buttons & BUTTON_SHOULDER_L)
            rctank_storage_volume_set(rctank_storage_volume_get());
    }

    /* R1: 볼륨 증가 (100ms 간격), 뗐을 때 NVS 저장 */
    if (gp->buttons & BUTTON_SHOULDER_R) {
        if (now_ms - last_r1_ms >= DEBOUNCE_MS) {
            last_r1_ms = now_ms;
            uint8_t v = rctank_storage_volume_get();
            if (v < RCTANK_VOLUME_MAX) {
                v++;
                rctank_storage_volume_set(v);
                rctank_dfplayer_set_volume(v);
            }
        }
    } else {
        if (prev_buttons & BUTTON_SHOULDER_R)
            rctank_storage_volume_set(rctank_storage_volume_get());
    }

    /* Select + Start 3초: EEPROM 초기화 및 재시작, 진동 800ms 후 재시작 */
    uint8_t sel = (gp->misc_buttons & MISC_BUTTON_SELECT) ? 1 : 0;
    uint8_t sta = (gp->misc_buttons & MISC_BUTTON_START) ? 1 : 0;
    if (sel && sta) {
        if (select_start_pressed_at == 0)
            select_start_pressed_at = now_ms;
        if (now_ms - select_start_pressed_at >= SELECT_START_HOLD_MS) {
            if (d->report_parser.play_dual_rumble != NULL)
                d->report_parser.play_dual_rumble(d, 0, 800, 255, 255);
            esp_timer_stop(restart_timer);
            esp_timer_start_once(restart_timer, 800 * 1000);
        }
    } else {
        select_start_pressed_at = 0;
    }

    prev_buttons = gp->buttons;
    prev_misc = gp->misc_buttons;
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    switch (event) {
        case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON: {
            uni_hid_device_t* d = data;
            if (d == NULL) {
                loge("ERROR: my_platform_on_oob_event: Invalid NULL device\n");
                return;
            }
            my_platform_instance_t* ins = get_my_platform_instance(d);
            ins->gamepad_seat = ins->gamepad_seat == GAMEPAD_SEAT_A ? GAMEPAD_SEAT_B : GAMEPAD_SEAT_A;
            trigger_event_on_gamepad(d);
            break;
        }
        case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
            logi("custom: Bluetooth enabled: %d\n", (bool)(data));
            break;
        default:
            logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
            break;
    }
}

static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
    return (my_platform_instance_t*)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    my_platform_instance_t* ins = get_my_platform_instance(d);
    if (d->report_parser.play_dual_rumble != NULL)
        d->report_parser.play_dual_rumble(d, 0, 150, 128, 40);
    if (d->report_parser.set_player_leds != NULL)
        d->report_parser.set_player_leds(d, ins->gamepad_seat);
    if (d->report_parser.set_lightbar_color != NULL) {
        uint8_t red = (ins->gamepad_seat & 0x01) ? 0xff : 0;
        uint8_t green = (ins->gamepad_seat & 0x02) ? 0xff : 0;
        uint8_t blue = (ins->gamepad_seat & 0x04) ? 0xff : 0;
        d->report_parser.set_lightbar_color(d, red, green, blue);
    }
}

struct uni_platform* get_my_platform(void) {
    static struct uni_platform plat = {
        .name = "custom",
        .init = my_platform_init,
        .on_init_complete = my_platform_on_init_complete,
        .on_device_discovered = my_platform_on_device_discovered,
        .on_device_connected = my_platform_on_device_connected,
        .on_device_disconnected = my_platform_on_device_disconnected,
        .on_device_ready = my_platform_on_device_ready,
        .on_oob_event = my_platform_on_oob_event,
        .on_controller_data = my_platform_on_controller_data,
        .get_property = my_platform_get_property,
    };
    return &plat;
}
