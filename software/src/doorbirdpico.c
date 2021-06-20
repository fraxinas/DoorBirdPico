#include "doorbirdpico.h"

void on_uart_rx()
{
    printf("on_uart_rx... ");
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
                    return;
                }
            }
        }
    }
    printf("UART error occured!\n");
}

rs485_key_t rs485_key_from_str (char *msg)
{
    rs485_key_t key = RS485_K_NONE;
    if (strlen(msg) <= RS485_KEY_LEN+1)
    {
        printf("RS485: buffer too short!\n");
        return key;
    }
    if (msg[RS485_KEY_LEN+1] != '=')
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
    while (uart_is_readable(RS485_ID))
    {
        uint8_t buf[RS485_BUF_LEN];
        rs485_key_t key;

        uart_read_blocking(RS485_ID, buf, RS485_BUF_LEN);
        printf("RS485: received '%s' from knxadapter\n", buf);

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
                    return;
                }
                goto unhandled_value;
                break;
            }
            case RS485_K_BUZZER:
            {
                int buzzer = atoi(val);
                printf("RS485: %s Buzzer ", buzzer ? "enable" : "disable");
                gpio_put(PIN_RELAY_OUT, buzzer);
                break;
            }
            case RS485_K_BRIGHTNESS:
            default:
                return;
        }
    }
    return;

unhandled_value:
    printf("RS485: unhandled value!\n");
}

void uart_send_code(char *code)
{
    uart_puts(UART_ID, code);
    uart_puts(UART_ID, "\r\n");
}

void rs485_send_msg(rs485_key_t key, const char* val)
{
    char msg[RS485_BUF_LEN];
    snprintf (msg, RS485_BUF_LEN, "%s=%s\r\n", rs485_key_str[key], val);
    uart_puts(uart0, msg);
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
        if (p.key_in == gpio)
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
        if (pwm_running) {
            printf("don't re-init pwm because it's already running!\r\n");
        }
        if (p.key_in == gpio)
        {
            if (pwm_hw->slice[p.slice_num].ctr == PWM_CH0_CTR_RESET) {
                printf("initializing pwm slice %d for LED on pin %d\r\n", p.slice_num, p.led_out);
                pwm_leds[i].fade = p.val_init;
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
        printf("[%d]=", i);

        if (!p.led_out)
            continue;

        // printf(" checking whether active\r\n");

        // printf("=%i ", p.active);

        printf("slice=%i ", p.slice_num);

        bool do_clear_slice = true;

        for (uint8_t j = 0; j < MAX_PWM_LEDS; j++)
        {
            if (p.slice_num == pwm_leds[j].slice_num && i != j && pwm_leds[j].active)
            {
                printf("don't clear slice %d on led [%d] because [%d] is still active\r\n", p.slice_num, i, j);
                do_clear_slice = false;
            }
        }

        if (do_clear_slice) {
            // Clear the interrupt flag that brought us here
            pwm_clear_irq(p.slice_num);
        }

        if (p.active) {
            int8_t dir = p.val_end > p.val_init ? (+1) : (-1);
            printf("cycle=%u fade=%u dir=%d  ", pwm_leds[i].cycle, pwm_leds[i].fade, dir);

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
                bool do_stop_pwm = true;

                for (uint8_t j = 0; j < MAX_PWM_LEDS; j++)
                {
                    pwm_led_t q = pwm_leds[j];
                    if (q.active) {
                        printf("setting new divider %.2f for gpio %d\n", divider, p.led_out);
                        pwm_config_set_clkdiv(&pwm_conf, divider);
                        pwm_init(q.slice_num, &pwm_conf, true);
                        if ((p.slice_num == q.slice_num && i != j)/* || (p.key_in == q.key_in)*/) {
                            printf("don't stop pwm slice %d on led [%d] because [%d] is still active\r\n", p.slice_num, i, j);
                            do_stop_pwm = false;
                        }
                    }
                }
                level = pwm_gamma[p.val_end];
                if (do_stop_pwm) {
                    printf("...stop PWM\r\n");
                    pwm_init(p.slice_num, &pwm_conf, false);
                    pwm_running = false;
                }
                continue;
            } else {
                level = pwm_gamma[pwm_leds[i].fade];
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
    printf("buzzer_alarm_cb -> Buzzer off!\r\n");
    gpio_put(PIN_RELAY_OUT, 0);
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
        send_locking_action(LOCK_A_LOCKING);
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
        printf("sensor_input(%d, %02X) ...", gpio, events);
        busy_wait_us(1000);
        bool RisingEdge = gpio_get(gpio);
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
        1 << PIN_STATUS);
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
    pwm_leds[0].led_out = PIN_A_GREEN;
    pwm_leds[0].val_init = 250;
    pwm_leds[0].val_end = 0;

    pwm_leds[1].key_in = PIN_B_KEY;
    pwm_leds[1].led_out = PIN_B_BLUE;
    pwm_leds[1].val_init = 250;
    pwm_leds[1].val_end = 50;

    pwm_leds[2].key_in = PIN_B_KEY;
    pwm_leds[2].led_out = PIN_B_RED;
    pwm_leds[2].val_init = 200;
    pwm_leds[2].val_end = 0;

    uint slice_num;

    for (uint8_t i = 0; i < MAX_PWM_LEDS; i++)
    {
        pwm_led_t p = pwm_leds[i];

        if (p.led_out == 0)
            continue;

        // Tell the LED pin that the PWM is in charge of its value.
        gpio_set_function(p.led_out, GPIO_FUNC_PWM);
        pwm_set_gpio_level(p.led_out, pwm_gamma[p.val_end]);

        // Figure out which slice we just connected to the LED pin
        slice_num = pwm_gpio_to_slice_num(p.led_out);
        pwm_leds[i].slice_num = slice_num;

        if (pwm_hw->inte & 1u << slice_num)
        {
            printf("led[%d] slice_num %u is already in use, don't re-enable \r\n", p.led_out, slice_num);
        } else {
            printf("led[%d] setting idle val %d = %u, slice_num=%u\r\n", p.led_out, p.val_end, pwm_gamma[p.val_end], slice_num);
            pwm_clear_irq(slice_num);
            pwm_set_irq_enabled(slice_num, true);
        }
    }
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_running = false;

    pwm_conf = pwm_get_default_config();
}

int setup_uart()
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

    // Initialise UART 1 for RS485 link to knxadapter
    uart_init(RS485_ID, BAUD_RATE);

    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(RS485_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RS485_RX_PIN, GPIO_FUNC_UART);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART1_IRQ, on_rs485_rx);
    irq_set_enabled(UART1_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}

int main()
{
    setup_gpio();
    setup_irq();
    setup_uart();

    sleep_ms(2000);
    setup_pwm();

    gpio_put(PIN_STATUS, 1);

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
