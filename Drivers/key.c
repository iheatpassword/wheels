#include "key.h"

static LED_MODE gLedMode = LED_MODE_OFF;
static uint32_t gStartTime = 0;
static uint32_t gBlinkLastTime = 0;

//检测一个按键状态, 返回按键状态, 输入按键端口和引脚名
uint8_t key_pressed(GPIO_Regs* key_port, uint32_t key_pin)
{
    static uint8_t last_raw_state[5] = {0};
    static uint8_t debounced_state[5] = {0};
    static uint32_t last_time[5] = {0};
    static uint8_t reported_pressed[5] = {0};
    uint8_t key_index;
    uint8_t current_state;

    if (key_port == KEY_key0_PORT && key_pin == KEY_key0_PIN) {
        key_index = 0;
    } else if (key_port == KEY_key1_PORT && key_pin == KEY_key1_PIN) {
        key_index = 1;
    } else if (key_port == KEY_key2_PORT && key_pin == KEY_key2_PIN) {
        key_index = 2;
    } else if (key_port == KEY_key3_PORT && key_pin == KEY_key3_PIN) {
        key_index = 3;
    } else if (key_port == KEY_key4_PORT && key_pin == KEY_key4_PIN) {
        key_index = 4;
    } else {
        return 0;
    }

    current_state = (DL_GPIO_readPins(key_port, key_pin) == 0) ? 1 : 0;
    
    if (current_state != last_raw_state[key_index]) {
        last_time[key_index] = millis();
        last_raw_state[key_index] = current_state;
    }

    if ((millis() - last_time[key_index]) > KEY_DEBOUNCE_MS) {
        debounced_state[key_index] = current_state;
    }

    if (debounced_state[key_index] && !reported_pressed[key_index]) {
        reported_pressed[key_index] = 1;
        return 1;
    } else if (!debounced_state[key_index]) {
        reported_pressed[key_index] = 0;
    }

    return 0;
}

static void led_on(void)
{
    DL_GPIO_setPins(LED_PORT, LED_led0_PIN);
}

static void led_off(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_led0_PIN);
}

static void led_toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_led0_PIN);
}

/*expect phenomenon: if key0 is pressing, led0 on; 
key1 pressed, led on; 
key2 pressed led 0ff; 
key3 pressed led on for 3 sec;
key4 pressed led blink at 1 sec intervals
*/
void key_test(void)
{
    if (key_pressed(KEY_key1_PORT, KEY_key1_PIN)) {
        gLedMode = LED_MODE_ON;
        led_on();
    }

    if (key_pressed(KEY_key2_PORT, KEY_key2_PIN)) {
        gLedMode = LED_MODE_OFF;
        led_off();
    }

    if (key_pressed(KEY_key3_PORT, KEY_key3_PIN)) {
        gLedMode = LED_MODE_TIMER_3S;
        gStartTime = millis();
        led_on();
    }

    if (key_pressed(KEY_key4_PORT, KEY_key4_PIN)) {
        gLedMode = LED_MODE_BLINK;
        gBlinkLastTime = millis();
        led_on();
    }

    if (DL_GPIO_readPins(KEY_key0_PORT, KEY_key0_PIN) != 0) {
        led_on();
    } else {
        if (gLedMode == LED_MODE_OFF) {
            led_off();
        }
    }

    switch (gLedMode) {
        case LED_MODE_TIMER_3S:
            if ((millis() - gStartTime) >= TIMER_3S_MS) {
                gLedMode = LED_MODE_OFF;
                led_off();
            }
            break;

        case LED_MODE_BLINK:
            if ((millis() - gBlinkLastTime) >= BLINK_INTERVAL_MS) {
                led_toggle();
                gBlinkLastTime = millis();
            }
            break;

        default:
            break;
    }
}
