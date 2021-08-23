#include "doorbirdpico.h"

void on_uart_rx()
{
    printf("on_uart_rx... ");
    if (uart_is_readable(UART_ID))
        on_doorbird_rx();

    if (uart_is_readable(RS485_ID))
        on_rs485_rx();
}

void on_doorbird_rx()
{
    printf("on_doorbird_rx... ");
    while (uart_is_readable(UART_ID))
    {
        uint8_t ch = uart_getc(UART_ID);
        printf("0x%02X ", ch);
        if (ch == 0x02)
        {
            ch = uart_getc(UART_ID);
            printf("%02X\n", ch);
            if (ch == 0x01)
            {
                if (uart_is_writable(UART_ID))
                {
                    printf("received startup bytes from Doorbird. Sending UART_INIT_SEQUENCE...\n");
                    uart_puts(UART_ID, UART_INIT_SEQUENCE);
                    printf("'%s' sent successfully!\n", UART_INIT_SEQUENCE);
                    // setup_uart(true);
                    return;
                }
            }
        }
    }
}

rs485_key_t rs485_key_from_str (char *msg)
{
    rs485_key_t key = RS485_K_NONE;
    if (strlen(msg) <= RS485_KEY_LEN+1)
    {
        printf("RS485: buffer too short!\n");
        return key;
    }
    if (msg[RS485_KEY_LEN] != '=')
    {
        printf("RS485: can't parse KEY=VALUE command!\n");
        return key;
    }
    for (key; key < RS485_K_LAST; key++)
    {
        if (strncmp(msg, rs485_key_str[key], RS485_KEY_LEN) == 0)
            return key;
    }
    printf("RS485: unhandled key\n");
    return RS485_K_NONE;
}

void on_rs485_rx()
{
    uint32_t now = time_us_32();
    uint32_t start = now;
    uint8_t buf[RS485_BUF_LEN+1];
    uint8_t p = 0;
    uint8_t ch = ' ';
    uart_set_irq_enables(RS485_ID, false, false);
    //printf("on_rs485_rx...");

    while (p < RS485_BUF_LEN && now < start+RS485_READ_TIMEOUT)
    {
        now = time_us_32();
        if (uart_is_readable(RS485_ID))
        {
            ch = uart_getc(RS485_ID);
            if (ch == 0)
            {
                uart_set_irq_enables(RS485_ID, true, false);
                return;
            }
            // printf("%02X", ch);
            if (ch == '\n' || ch == '\r')
            {
                p++;
                break;
            }
            buf[p++] = ch;
        }
    }
    buf[p] = '\0';

    multicore_fifo_push_blocking((uintptr_t) &mc_handle_rs485_command);
    multicore_fifo_push_blocking((uintptr_t) &buf);

    int ret = multicore_fifo_pop_blocking();
    uart_set_irq_enables(RS485_ID, true, false);
}

bool mc_handle_rs485_command(char *buf)
{
    rs485_key_t key;
    if (strlen(buf) == 1)
        return false;

    printf("\r\nRS485: received '%s' from knxadapter\n", buf);

    key = rs485_key_from_str (buf);

    uint8_t *val = buf+RS485_KEY_LEN+1;
    switch (key)
    {
        case RS485_K_LOCKSTATE:
        {
            lock_state_t new_state = LOCK_S_UNKNOWN;
            if (strcmp(val, lock_state_str[LOCK_S_LOCKED]) == 0)
            {
                new_state = LOCK_S_LOCKED;
            } else if (strcmp(val, lock_state_str[LOCK_S_UNLOCKED]) == 0)
            {
                new_state = LOCK_S_UNLOCKED;
            }
            if (new_state != LOCK_S_UNKNOWN) {
                printf("RS485: New lock state set (previous state: %s) -> ", lock_state_str[lock_state]);
                set_key_c_color(LOCK_A_ACTUATED);
                add_alarm_in_ms(DELAY_ACTUATED_LOCK_MS, actuated_lock_alarm_cb, (void*) new_state, false);
                return true;
            }
            printf("RS485: unhandled value '%s'!\n", val);
            return false;
            break;
        }
        case RS485_K_BUZZER:
        {
            int buzzer = atoi(val);
            printf("RS485: %s Buzzer ", buzzer ? "enable" : "disable");
            gpio_put(PIN_RELAY_OUT, buzzer);
            if (buzzer)
                add_alarm_in_ms(BUZZER_SAFETY_MS, buzzer_alarm_cb, NULL, false);
            break;
        }
        case RS485_K_NONE:
        {
            return false;
        }
        case RS485_K_BRIGHTNESS:
        {
            global_brightness = atoi(val);
            printf("RS485: Brightness %d\r\n", global_brightness);
            for (uint8_t i = 0; i < MAX_PWM_LEDS; i++)
            {
                if (pwm_leds[i].led_out)
                {
                    uint16_t level = get_gamma(pwm_leds[i].val_init);
                    pwm_set_gpio_level(pwm_leds[i].led_out, level);
                }
            }
            break;
        }
        case RS485_K_COLOR:
        {
            char *keypos = val, *keyctx;
            char *bndpos, *bndctx;
            char *valpos, *valctx;
            int pin_idx = 0;
            // Command Syntax: COLOR=R,G,B/R,G,B;R,G,B/R,G,B
            //                   KEY A init/end ; init/end KEY B
            // printf("RS485: Color previous (R, G, B) values: KEY A init=(%d, %d, %d) end=(%d, %d, %d)\t",
            // pwm_leds[0].val_init, pwm_leds[1].val_init, pwm_leds[2].val_init, pwm_leds[0].val_end, pwm_leds[1].val_end, pwm_leds[2].val_end);
            // printf("KEY B init=(%d, %d, %d) end=(%d, %d, %d)\r\n",
            // pwm_leds[3].val_init, pwm_leds[4].val_init, pwm_leds[5].val_init, pwm_leds[3].val_end, pwm_leds[4].val_end, pwm_leds[5].val_end);

            for (int key_idx = 0; key_idx <= 1; key_idx++)
            {
                keypos = strtok_r(val,";", &keyctx);
                // printf("\r\n keypos='%s'", keypos);
                int pin_min_idx = pin_idx;
                int pin_max_idx = pin_idx+3;
                // read R, G, B initial values for key

                bndpos = strtok_r(keypos,"/", &bndctx);
                // printf("\r\n valpos1='%s'", bndpos);
                for (pin_idx=pin_min_idx; pin_idx < pin_max_idx; pin_idx++)
                {
                    valpos = strtok_r(bndpos,",", &valctx);
                    uint8_t val_init = atoi(valpos);
                    pwm_leds[pin_idx].val_init = val_init;
                    // printf("[%d]=%d\t", pin_idx, val_init);
                    uint16_t level = get_gamma(val_init);
                    pwm_set_gpio_level(pwm_leds[pin_idx].led_out, level);
                    bndpos = NULL;
                }

                bndpos = strtok_r(NULL,"/", &bndctx);
                // printf("\r\n valpos2='%s'", bndpos);
                for (pin_idx=pin_min_idx; pin_idx < pin_max_idx; pin_idx++)
                {
                    valpos = strtok(bndpos,",");
                    pwm_leds[pin_idx].val_end = atoi(valpos);
                    // printf("[%d]=%d\t", pin_idx, pwm_leds[pin_idx].val_end);
                    // printf("Color[%d] pin=%i val_end=%d)\r\n", key_idx, pin_idx, pwm_leds[pin_idx].val_end);
                    bndpos = NULL;
                }
                val = NULL;
            }
            printf("RS485: New Colors (R, G, B) values: KEY A init=(%d, %d, %d) end=(%d, %d, %d)\t",
            pwm_leds[0].val_init, pwm_leds[1].val_init, pwm_leds[2].val_init, pwm_leds[0].val_end, pwm_leds[1].val_end, pwm_leds[2].val_end);
            printf("KEY B init=(%d, %d, %d) end=(%d, %d, %d)\r\n",
            pwm_leds[3].val_init, pwm_leds[4].val_init, pwm_leds[5].val_init, pwm_leds[3].val_end, pwm_leds[4].val_end, pwm_leds[5].val_end);
            break;
        }
        default:
            printf("command '%s' (%d) from knxadapter unhandled\n", rs485_key_str[key], key);
            return false;
    }
    return true;
}

void uart_send_code(char *code)
{
    uart_puts(UART_ID, code);
    uart_puts(UART_ID, "\r\n");
}

void rs485_send_msg(rs485_key_t key, const char* val)
{
    printf("RS485: send msg ");
    gpio_put(RS485_DRIVER_ENABLE_PIN, 1);
    char msg[RS485_BUF_LEN];
    snprintf (msg, RS485_BUF_LEN, "%s=%s\r\n\r\n", rs485_key_str[key], val);
    uart_write_blocking	(RS485_ID, msg, strlen(msg));
    printf("%s", msg);
    gpio_put(RS485_DRIVER_ENABLE_PIN, 0);
}

uint8_t get_active_led_count()
{
    uint8_t count = 0;
    for (uint8_t i; i < MAX_PWM_LEDS; i++)
    {
        if (pwm_leds[i].active)
            count++;
    }
    return count;
}

void key_callback(uint gpio)
{
    float divider;
    uint8_t i = 0;

    printf("key_callback(%d) ", gpio);
    switch (gpio)
    {
        case PIN_A_KEY:
            uart_send_code(KEY_A_CODE);
            printf("Key A rang\r\n");
            break;
        case PIN_B_KEY:
            uart_send_code(KEY_B_CODE);
            printf("Key B rang\r\n");
            break;
    }

    for (i; i < MAX_PWM_LEDS; i++)
    {
        pwm_led_t p = pwm_leds[i];
        if (p.key_in == gpio && p.val_init != p.val_end)
        {
            printf("activate LED on pin %d ", p.led_out);
            pwm_leds[i].active = true;
            pwm_leds[i].cycle = p.cycle % 2 ? 1 : 0;
        }
    }

    divider = (float)get_active_led_count() * PWM_LED_INIT_DIV;
    // divider = PWM_LED_INIT_DIV;
    printf("start pwm with divider %.2f ", divider);
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&pwm_conf, divider);

    for (i = 0; i < MAX_PWM_LEDS; i++)
    {
        pwm_led_t p = pwm_leds[i];
        if (pwm_fading) {
            printf("don't re-init pwm because it's already running!\r\n");
        }
        if (p.active)
        {
            pwm_leds[i].fade = p.val_init;
            if ((pwm_hw->inte & 1u << p.slice_num) == false)
            {
                printf("initializing pwm slice %d for LED on pin %d\r\n", p.slice_num, p.led_out);
                pwm_set_irq_enabled(p.slice_num, true);
                pwm_init(p.slice_num, &pwm_conf, true);
            } else {
                printf("don't re-init pwm slice %d for LED on pin %d because it's already running!\r\n", p.slice_num, p.led_out);
            }
        }
    }
}

void on_pwm_wrap()
{
    uint8_t i = 0;

    printf("on_pwm_wrap() ");

    for (i = 0; i < MAX_PWM_LEDS; i++)
    {
        pwm_led_t p = pwm_leds[i];
        uint16_t level;
        bool do_clear_slice = true;

        // printf("[%d]=", i);

        if (!p.led_out)
            continue;

        // printf("slice=%i ", p.slice_num);

        for (uint8_t j = 0; j < MAX_PWM_LEDS; j++)
        {
            if (p.slice_num == pwm_leds[j].slice_num && i != j && pwm_leds[j].active)
            {
                printf(" {X} ", p.slice_num, i, j);
                do_clear_slice = false;
            }
        }

        if (do_clear_slice) {
            // Clear the interrupt flag that brought us here
            pwm_clear_irq(p.slice_num);
        }

        if (p.active) {
            printf("[%d]: ", i);
            int8_t dir = p.val_end > p.val_init ? (+1) : (-1);
            printf("c=%u f=%u %d  ", pwm_leds[i].cycle, pwm_leds[i].fade, dir);

            if (pwm_leds[i].cycle % 2 == 0)
            {
                if (pwm_leds[i].fade == p.val_end)
                {
                    pwm_leds[i].cycle += 1;
                } else {
                    pwm_leds[i].fade += dir;
                }

            }
            else
            {
                if (pwm_leds[i].fade == p.val_init)
                {
                    pwm_leds[i].cycle += 1;
                } else {
                    pwm_leds[i].fade -= dir;
                }
            }
            if (pwm_leds[i].cycle == PWM_LED_CYCLES)
            {
                // if button still pressed, start over with cycles
                if (gpio_get(p.key_in)) {
                    pwm_leds[i].cycle = 0;
                    continue;
                }

                uint8_t num_active = get_active_led_count()-1;
                // float divider = (float)num_active * PWM_LED_INIT_DIV;
                float divider = PWM_LED_INIT_DIV;
                printf(" i=%d finished. num_active=%u ", i, num_active);
                pwm_leds[i].cycle = 0;
                pwm_leds[i].active = false;
                bool do_stop_fade = true;

                for (uint8_t j = 0; j < MAX_PWM_LEDS; j++)
                {
                    pwm_led_t q = pwm_leds[j];
                    if (q.active) {
                        printf("setting new divider %.2f for gpio %d\n", divider, p.led_out);
                        pwm_config_set_clkdiv(&pwm_conf, divider);
                        pwm_init(q.slice_num, &pwm_conf, true);
                        if ((p.slice_num == q.slice_num && i != j)/* || (p.key_in == q.key_in)*/) {
                            printf("don't stop fading slice %d on led [%d] because [%d] is still active\r\n", p.slice_num, i, j);
                            do_stop_fade = false;
                        }
                    }
                }
                level = get_gamma(p.val_init);
                if (do_stop_fade) {
                    printf("...stop fading\r\n");
                    pwm_set_irq_enabled(p.slice_num, false);
                    pwm_fading = false;
                }
                continue;
            } else {
                level = get_gamma(pwm_leds[i].fade);
            }
            pwm_set_gpio_level(p.led_out, level);
            printf("%hu ", level);
        }
    }
    printf("\r\n");
}

void set_key_c_color(lock_action_t action)
{
    bool r, g, b = 1;

    switch (action)
    {
    case LOCK_A_OFF: // black
        r = 1;
        g = 1;
        b = 1;
        break;
    case LOCK_A_LOCKING: // purple
        r = 0;
        g = 1;
        b = 0;
        break;
    case LOCK_A_LOCKED: // red
        r = 0;
        g = 1;
        b = 1;
        break;
    case LOCK_A_LEAVING: // blue
        r = 1;
        g = 1;
        b = 0;
        break;
    case LOCK_A_UNLOCKING: // cyan
        r = 1;
        g = 0;
        b = 0;
        break;
    case LOCK_A_UNLOCKED: // green
        r = 1;
        g = 0;
        b = 1;
        break;
    case LOCK_A_ACTUATED: // yellow
        r = 0;
        g = 0;
        b = 1;
        break;
    case LOCK_A_PRESSED: // white
        r = 0;
        g = 0;
        b = 0;
        break;
    default:
        break;
    }
    gpio_put(PIN_C_RED, r);
    gpio_put(PIN_C_GREEN, g);
    gpio_put(PIN_C_BLUE, b);
}

int64_t buzzer_alarm_cb(alarm_id_t id, void *user_data)
{
    if (gpio_get(PIN_RELAY_OUT)) {
        printf("buzzer_alarm_cb -> Buzzer off!\r\n");
        gpio_put(PIN_RELAY_OUT, 0);
    }
    return 0;
}

int64_t actuated_lock_alarm_cb(alarm_id_t id, void *user_data)
{
    lock_state_t new_state;
    if (user_data) {
        // state received via RS485
        new_state = (lock_state_t)user_data;
    } else {
        // relais toggled
        new_state = gpio_get(PIN_RELAY_IN2) ? LOCK_S_LOCKED : LOCK_S_UNLOCKED;
    }
    printf("actuated_lock_alarm_cb -> new_state: %s\r\n", lock_state_str[new_state]);
    if (new_state == LOCK_S_LOCKED)
    {
        set_key_c_color(LOCK_A_LOCKED);
    }
    else if (new_state == LOCK_S_UNLOCKED)
    {
        set_key_c_color(LOCK_A_UNLOCKED);
        if (gpio_get(PIN_RELAY_IN1))
        {
            printf("Door got unlocked & Relay 1 still on -> Buzzer on!\r\n");
            gpio_put(PIN_RELAY_OUT, 1);
            add_alarm_in_ms(BUZZER_SAFETY_MS, buzzer_alarm_cb, NULL, false);
        }
        if (lock_state == LOCK_S_WAITING_FOR_UNLOCK)
        {
            printf("Door got unlocked while waiting -> Buzzer on for %d ms!\r\n", BUZZER_MS);
            gpio_put(PIN_RELAY_OUT, 1);
            add_alarm_in_ms(BUZZER_MS, buzzer_alarm_cb, NULL, false);
        }
    }
    lock_state = new_state;
    return 0;
}

int64_t delayed_lock_alarm_cb(alarm_id_t id, void *user_data)
{
    printf("...delayed_lock_alarm_cb(%d) send %s\r\n", (int)id, LOCK_CODE);
    send_locking_action(LOCK_A_LOCKING);
    return 0;
}

void send_locking_action(lock_action_t action)
{
    set_key_c_color(action);

    if (action == LOCK_A_LOCKING) {
        uart_send_code(LOCK_CODE);
    } else if (action == LOCK_A_UNLOCKING) {
        uart_send_code(UNLOCK_CODE);
    }

    rs485_send_msg(RS485_K_LOCKACTION, lock_action_str[action]);
}

int64_t lock_key_pressed_cb(alarm_id_t id, void *user_data)
{
    printf("...lock_key_pressed_cb(%d) ", (int)id);

    bool long_pressed = gpio_get(PIN_C_KEY);

    if (long_pressed)
    {
        printf("LONG pressed -> LOCK immediately (send %s)\r\n", LOCK_CODE);
        send_locking_action(LOCK_A_LOCKING);
    }
    else
    {
        printf("short pressed -> LOCK delayed ");
        add_alarm_in_ms(DELAY_LEAVING_LOCK_MS, delayed_lock_alarm_cb, NULL, false);
    }

    return 0;
}

void sensor_input(uint gpio, uint32_t events)
{
    static uint32_t last_irq_ts = 0;
    uint32_t now = time_us_32();

    if (now - last_irq_ts > DEBOUNCE_MS * 1000)
    {
        bool beginVal = gpio_get(gpio);
        printf("sensor_input(%d, %02X)=%d...", gpio, events, beginVal);
        busy_wait_us(3000);
        bool RisingEdge = gpio_get(gpio);
        if (beginVal != RisingEdge || events == 0x0C)
        {
            printf(" was a glitch! ignored.\r\n");
            last_irq_ts = now;
            return;
        }
        bool FallingEdge = !RisingEdge;
        printf(" is %s\t", RisingEdge ? "RisingEdge" : "FallingEdge");

        switch (gpio)
        {
        case PIN_A_KEY:
        case PIN_B_KEY:
            if (RisingEdge)
                key_callback(gpio);
            break;
        case PIN_C_KEY:
            printf("lock toggle pressed in state %s ", lock_state_str[lock_state]);
            if (!RisingEdge)
                break;
            if (lock_state != LOCK_S_UNLOCKED)
            {
                printf(" -> UNLOCK (send %s)\r\n", UNLOCK_CODE);
                send_locking_action(LOCK_A_UNLOCKING);
            }
            else
            {
                set_key_c_color(LOCK_A_PRESSED);
                add_alarm_in_ms(LONG_PRESS_MS, lock_key_pressed_cb, NULL, false);
            }
            break;
        case PIN_REED_LETTER:
            if (FallingEdge)
            {
                printf("Letterbox opened (send %s)\r\n", LETTER_CODE);
                uart_send_code(LETTER_CODE);
            }
            door_state_t letter_state = RisingEdge ? DOOR_CLOSED : DOOR_OPEN;
            rs485_send_msg(RS485_K_LETTERREED, door_state_str[letter_state]);
            break;
        case PIN_REED_DOOR:
            door_state = RisingEdge ? DOOR_CLOSED : DOOR_OPEN;
            rs485_send_msg(RS485_K_DOORREED, door_state_str[door_state]);
            printf("Reed door toggle -> door_state = %s\r\n", door_state_str[door_state]);
            break;
        case PIN_RELAY_IN1:
            if (FallingEdge)
            {
                printf("Relay 1 falling -> Buzzer off!\r\n");
                gpio_put(PIN_RELAY_OUT, 0);
            }
            else
            {
                if (lock_state == LOCK_S_UNLOCKED)
                {
                    printf("Relay 1 rising -> Buzzer on!\r\n");
                    add_alarm_in_ms(BUZZER_SAFETY_MS, buzzer_alarm_cb, NULL, false);
                    gpio_put(PIN_RELAY_OUT, 1);
                }
                else
                {
                    printf("Relay 1 rising while in LOCKED ");
                    if (DO_UNLOCK_ON_OPEN)
                    {
                        printf(" -> UNLOCK (send %s)...\r\n", UNLOCK_CODE);
                        send_locking_action(LOCK_A_UNLOCKING);
                    }
                    else
                    {
                        printf(" -> waiting for unlocked...\r\n");
                        lock_state = LOCK_S_WAITING_FOR_UNLOCK;
                        send_locking_action(LOCK_A_LEAVING);
                    }
                }
            }
            break;
        case PIN_RELAY_IN2:
            printf("Relay 2 toggle (previous state: %s) -> ", lock_state_str[lock_state]);
            set_key_c_color(LOCK_A_ACTUATED);
            add_alarm_in_ms(DELAY_ACTUATED_LOCK_MS, actuated_lock_alarm_cb, NULL, false);
            break;
        }
    }
    last_irq_ts = now;
}

int setup_irq()
{
    stdio_init_all();
    printf("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback(PIN_A_KEY, GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_B_KEY, GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_C_KEY, GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_REED_LETTER, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_REED_DOOR, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_RELAY_IN1, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_RELAY_IN2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &sensor_input);
}

int setup_gpio()
{
    gpio_init_mask(
        1 << PIN_A_BLUE |
        1 << PIN_A_GREEN |
        1 << PIN_A_RED |
        1 << PIN_B_BLUE |
        1 << PIN_B_GREEN |
        1 << PIN_B_RED |
        1 << PIN_C_BLUE |
        1 << PIN_C_GREEN |
        1 << PIN_C_RED |
        1 << PIN_RELAY_OUT |
        1 << PIN_STATUS |
        1 << RS485_DRIVER_ENABLE_PIN);
    gpio_set_dir(PIN_RELAY_OUT, GPIO_OUT);
    gpio_set_dir(PIN_A_BLUE, GPIO_OUT);
    gpio_set_dir(PIN_A_GREEN, GPIO_OUT);
    gpio_set_dir(PIN_A_RED, GPIO_OUT);
    gpio_set_dir(PIN_B_BLUE, GPIO_OUT);
    gpio_set_dir(PIN_B_GREEN, GPIO_OUT);
    gpio_set_dir(PIN_B_RED, GPIO_OUT);
    gpio_set_dir(PIN_C_BLUE, GPIO_OUT);
    gpio_set_dir(PIN_C_GREEN, GPIO_OUT);
    gpio_set_dir(PIN_C_RED, GPIO_OUT);
    gpio_set_dir(PIN_STATUS, GPIO_OUT);
    gpio_set_dir(RS485_DRIVER_ENABLE_PIN, GPIO_OUT);

    gpio_put(PIN_A_BLUE, 1);
    gpio_put(PIN_A_GREEN, 1);
    gpio_put(PIN_A_RED, 1);
    gpio_put(PIN_B_BLUE, 1);
    gpio_put(PIN_B_GREEN, 1);
    gpio_put(PIN_B_RED, 1);

    set_key_c_color(LOCK_A_OFF);

    // GPIO26 & 27 are analog by default, need to be set to input explicitely in order for IRQ to work
    gpio_set_input_enabled(PIN_RELAY_IN1, true);
    gpio_set_input_enabled(PIN_RELAY_IN2, true);

    gpio_pull_down(PIN_RELAY_IN1);
    gpio_pull_down(PIN_RELAY_IN2);
    gpio_pull_down(PIN_REED_DOOR);
    gpio_pull_down(PIN_REED_LETTER);

    gpio_put(PIN_RELAY_OUT, 0);
    gpio_put(RS485_DRIVER_ENABLE_PIN, 0);

    door_state = DOOR_UNKNOWN;
    lock_state = LOCK_S_UNKNOWN;
}

int setup_pwm()
{
    for (uint8_t i = 0; i < MAX_PWM_LEDS; i++)
    {
      pwm_led_t p = pwm_leds[i];
      pwm_leds[i].active = false;
      pwm_leds[i].key_in = pwm_leds[i].led_out = 0;
      pwm_leds[i].cycle = 0;
    }

    pwm_leds[0].key_in = PIN_A_KEY;
    pwm_leds[0].led_out = PIN_A_RED;
    pwm_leds[0].val_init = 0;
    pwm_leds[0].val_end = 0;

    pwm_leds[1].key_in = PIN_A_KEY;
    pwm_leds[1].led_out = PIN_A_GREEN;
    pwm_leds[1].val_init = 255;
    pwm_leds[1].val_end = 0;

    pwm_leds[2].key_in = PIN_A_KEY;
    pwm_leds[2].led_out = PIN_A_BLUE;
    pwm_leds[2].val_init = 0;
    pwm_leds[2].val_end = 0;

    pwm_leds[3].key_in = PIN_B_KEY;
    pwm_leds[3].led_out = PIN_B_RED;
    pwm_leds[3].val_init = 200;
    pwm_leds[3].val_end = 20;

    pwm_leds[4].key_in = PIN_B_KEY;
    pwm_leds[4].led_out = PIN_B_GREEN;
    pwm_leds[4].val_init = 0;
    pwm_leds[4].val_end = 0;

    pwm_leds[5].key_in = PIN_B_KEY;
    pwm_leds[5].led_out = PIN_B_BLUE;
    pwm_leds[5].val_init = 255;
    pwm_leds[5].val_end = 55;

    global_brightness = 200;

    uint slice_num;

    pwm_conf = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwm_conf, 2.0);

    for (uint8_t i = 0; i < MAX_PWM_LEDS; i++)
    {
        pwm_led_t p = pwm_leds[i];

        if (p.led_out == 0)
            continue;

        if (p.val_init == p.val_end) {
            continue;
        }

        // Tell the LED pin that the PWM is in charge of its value.
        gpio_set_function(p.led_out, GPIO_FUNC_PWM);

        // Figure out which slice we just connected to the LED pin
        slice_num = pwm_gpio_to_slice_num(p.led_out);
        pwm_leds[i].slice_num = slice_num;

        if (pwm_hw->inte & 1u << slice_num)
        {
            printf("led[%d] slice_num %u is already in use, don't re-enable \r\n", p.led_out, slice_num);
        } else {
            uint16_t level = get_gamma(p.val_init);
            printf("led[%d] setting idle val %d = %u, slice_num=%u\r\n", p.led_out, p.val_init, level, slice_num);
            pwm_clear_irq(slice_num);
            pwm_set_irq_enabled(slice_num, false);
            pwm_init(slice_num, &pwm_conf, true);
            pwm_set_gpio_level(p.led_out, level);
        }
    }
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_fading = false;
}

uint16_t get_gamma(uint8_t value)
{
    value = 255 - (value * (global_brightness/255.0));
    uint16_t gamma = pwm_gamma[value];
    return gamma;
}

int setup_uart(bool enable_rs485)
{
// Initialise UART 0 for Doorbird communication
    uart_init(UART_ID, BAUD_RATE);

    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
    printf("UART0 (Doorbird communication) initialized.\r\n");

    // Initialise UART 1 for RS485 link to knxadapter
    uart_init(RS485_ID, BAUD_RATE);

    gpio_set_function(RS485_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RS485_RX_PIN, GPIO_FUNC_UART);

    uart_set_hw_flow(RS485_ID, false, false);
    uart_set_fifo_enabled(RS485_ID, false);

    irq_set_exclusive_handler(UART1_IRQ, on_rs485_rx);
    irq_set_enabled(UART1_IRQ, true);

    uart_set_irq_enables(RS485_ID, true, false);
    uart_set_translate_crlf(RS485_ID, true);
    printf("UART1 (RS485 communication) initialized.\r\n");
}

#define FLAG_VALUE 123

void core1_dispatcher() {
    while (1) {
        // Function pointer is passed to us via the FIFO
        // We have one incoming int32_t as a parameter, and will provide an
        // int32_t return value by simply pushing it back on the FIFO
        // which also indicates the result is ready.
        int32_t (*func)() = (int32_t(*)()) multicore_fifo_pop_blocking();
        int32_t p = multicore_fifo_pop_blocking();
        int32_t result = (*func)(p);
        multicore_fifo_push_blocking(result);
    }
}

int main()
{
    setup_gpio();

    setup_irq();
    setup_uart(false);
    sleep_ms(6000);
    setup_pwm();

    gpio_put(PIN_STATUS, 1);

    multicore_launch_core1(core1_dispatcher);

    // Everything after this point happens in the interrupt handlers,
    // so we can chill here
    while (1)
    {
        gpio_put(PIN_STATUS, 1);
        sleep_ms(250);
        gpio_put(PIN_STATUS, 0);
        sleep_ms(250);
        tight_loop_contents();
    }
}
