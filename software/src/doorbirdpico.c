#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_INIT_SEQUENCE "MTT_14_001_001\r\n"

#define PIN_REED_DOOR 2
#define PIN_REED_LETTER 3

#define PIN_A_KEY 6
#define PIN_A_BLUE 7
#define PIN_A_GREEN 8
#define PIN_A_RED 9

#define PIN_B_KEY 11
#define PIN_B_BLUE 12
#define PIN_B_GREEN 13
#define PIN_B_RED 14

#define PIN_C_KEY 22
#define PIN_C_BLUE 21
#define PIN_C_GREEN 20
#define PIN_C_RED 19

#define PIN_RELAY_OUT 28

#define PIN_RELAY_IN1 27
#define PIN_RELAY_IN2 26

#define LONG_PRESS_MS 800
#define DELAY_LEAVING_LOCK_MS 15000
#define DELAY_YELLOW_LOCK_MS 2500

#define PIN_STATUS 25

#define KEY_A_CODE "101#"
#define KEY_B_CODE "102#"
#define LOCK_CODE "103#"
#define UNLOCK_CODE "104#"
#define LETTER_CODE "105#"

#define DEBOUNCE_MS 15

typedef enum
{
    DOOR_UNKNOWN,
    DOOR_CLOSED,
    DOOR_OPEN
} door_state_t;

typedef enum
{
    LOCK_S_UNKNOWN,
    LOCK_S_LOCKED,
    LOCK_S_UNLOCKED
} lock_state_t;

typedef enum
{
    LOCK_A_OFF,
    LOCK_A_LOCKING,
    LOCK_A_LOCKED,
    LOCK_A_LEAVING,
    LOCK_A_UNLOCKING,
    LOCK_A_UNLOCKED,
    LOCK_A_ACTUATED,
    LOCK_A_PRESSED,
} lock_action_t;

door_state_t door_state;
lock_state_t lock_state;
pwm_config pwm_conf;
int fade_pins[2] = {-1, -1};

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

static char event_str[128];

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
                    uart_puts(uart0, UART_INIT_SEQUENCE);
                    printf("'%s' sent successfully!\n", UART_INIT_SEQUENCE);
                    return;
                }
            }
        }
    }
    printf("UART error occured!\n");
}

void uart_send_code(char *code)
{
    uart_puts(uart0, code);
    uart_puts(uart0, "\r\n");
}

void gpio_event_string(char *buf, uint32_t events)
{
    for (uint i = 0; i < 4; i++)
    {
        uint mask = (1 << i);
        if (events & mask)
        {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0')
            {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events)
            {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

void key_callback(uint gpio, uint32_t events)
{
    gpio_event_string(event_str, events);
    printf("GPIO KEY %d %s ", gpio, event_str);
    if (events & 0x8)
    {
        // Load the configuration into our PWM slice, and set it running.
        int i, irq_pin;

        switch (gpio)
        {
        case PIN_A_KEY:
            irq_pin = PIN_A_GREEN;
            uart_send_code(KEY_A_CODE);
            printf("Key A rang\r\n");
            i = 0;
            break;
        case PIN_B_KEY:
            irq_pin = PIN_B_BLUE;
            uart_send_code(KEY_B_CODE);
            printf("Key B rang\r\n");
            i = 1;
            break;
        }
        fade_pins[i] = irq_pin;
        printf("i=%d pin=%d", i, fade_pins[i]);

        float divider = 4.;

        if (fade_pins[0] > 0 && fade_pins[1] > 0)
        {
            // half time if both pins are fading
            divider = 8.;
        }

        printf(" start pwm divider %f", divider);
        // Set divider, reduces counter clock to sysclock/this value
        pwm_config_set_clkdiv(&pwm_conf, divider);

        for (int i = 0; i < 2; i++)
        {
            if (fade_pins[i] > 0)
            {
                pwm_init(pwm_gpio_to_slice_num(fade_pins[i]), &pwm_conf, true);
            }
        }
    }
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

int64_t set_key_c_color_alarm_cb(alarm_id_t id, void *user_data)
{
    printf("set_key_c_color_alarm_cb(%d)\r\n", (int)id);
    if (lock_state == LOCK_S_LOCKED)
    {
        set_key_c_color(LOCK_A_LOCKED);
    }
    else
    {
        set_key_c_color(LOCK_A_UNLOCKED);
    }
    return 0;
}

int64_t delayed_lock_alarm_cb(alarm_id_t id, void *user_data)
{
    printf("...delayed_lock_alarm_cb(%d) send %s\r\n", (int)id, LOCK_CODE);
    set_key_c_color(LOCK_A_OFF);
    uart_send_code(LOCK_CODE);
    return 0;
}

int64_t lock_key_pressed_cb(alarm_id_t id, void *user_data)
{
    printf("...lock_key_pressed_cb(%d) ", (int)id);

    bool long_pressed = gpio_get(PIN_C_KEY);

    if (long_pressed)
    {
        printf("LONG pressed -> LOCK immediately (send %s)\r\n", LOCK_CODE);
        uart_send_code(LOCK_CODE);
        set_key_c_color(LOCK_A_LOCKING);
    }
    else
    {
        printf("short pressed -> LOCK delayed ");
        set_key_c_color(LOCK_A_LEAVING);
        add_alarm_in_ms(DELAY_LEAVING_LOCK_MS, delayed_lock_alarm_cb, NULL, false);
    }

    return 0;
}

void sensor_input(uint gpio, uint32_t events)
{
    static uint32_t last_irq_ts = 0;
    uint32_t now = time_us_32();

    gpio_event_string(event_str, events);
    printf("Sensor input port %d %s events=%02X ...", gpio, event_str, events);

    if (now - last_irq_ts < DEBOUNCE_MS * 1000)
    {
        printf(" ignore bounce (%lu ms)\r\n", now - last_irq_ts);
    }
    else if (events & GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)
    {
        busy_wait_us(1000);
        bool RisingEdge = gpio_get(gpio);
        bool FallingEdge = !RisingEdge;
        printf(" detected %s\t", RisingEdge ? "RisingEdge" : "FallingEdge");

        switch (gpio)
        {
        case PIN_A_KEY:
        case PIN_B_KEY:
            if (RisingEdge)
            {
                key_callback(gpio, events);
            }
            break;
        case PIN_C_KEY:
            printf("Lock toggle Button pressed while state=%s ", lock_state == LOCK_S_LOCKED ? "locked" : (lock_state == LOCK_S_UNLOCKED ? "unlocked" : "unknown"));
            if (lock_state != LOCK_S_UNLOCKED)
            {
                printf(" -> UNLOCK (send %s)\r\n", UNLOCK_CODE);
                set_key_c_color(LOCK_A_UNLOCKING);
                uart_send_code(UNLOCK_CODE);
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
            break;
        case PIN_REED_DOOR:
            door_state = RisingEdge ? DOOR_CLOSED : DOOR_OPEN;
            printf("Reed door toggle -> door_state = %s\r\n", door_state == DOOR_OPEN ? "OPENED" : "CLOSED");
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
                    printf("Relay 1 rising but door is locked, waiting for unlock...\r\n");
                }
            }
            break;
        case PIN_RELAY_IN2:
            lock_state = RisingEdge ? LOCK_S_LOCKED : LOCK_S_UNLOCKED;
            printf("Relay 2 toggle -> lock_state = %s\r\n", lock_state == LOCK_S_LOCKED ? "LOCKED" : "UNLOCKED");
            set_key_c_color(LOCK_A_ACTUATED);
            add_alarm_in_ms(DELAY_YELLOW_LOCK_MS, set_key_c_color_alarm_cb, NULL, false);

            if (lock_state == LOCK_S_UNLOCKED && gpio_get(PIN_RELAY_IN1))
            {
                printf("Door got unlocked & Relay 1 still on -> Buzzer on!\r\n");
                gpio_put(PIN_RELAY_OUT, 1);
            }
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

void on_pwm_wrap()
{
    static int fade = 0;
    static bool going_up = true;
    static int count = 0;

    for (int i = 0; i < 2; i++)
    {
        printf("w[%d]", i);
        int irq_pin = fade_pins[i];
        printf("=", irq_pin);
        if (irq_pin < 0)
            continue;
        uint slice = pwm_gpio_to_slice_num(irq_pin);
        // Clear the interrupt flag that brought us here
        pwm_clear_irq(slice);

        if (going_up)
        {
            ++fade;
            if (fade > 255)
            {
                fade = 255;
                going_up = false;
                printf("switch to down\n");
            }
        }
        else
        {
            --fade;
            if (fade < 0)
            {
                fade = 0;
                going_up = true;
                printf("switch to up, count=%d\n", count);
                if (++count == 5)
                {
                    count = 0;
                    pwm_set_gpio_level(irq_pin, 0x2000);
                    fade_pins[i] = -1;
                    if (fade_pins[0] == -1 && fade_pins[0] == -1)
                    {
                        // when all fades finished, stop pwm
                        printf("stop pwn\n");
                        pwm_init(slice, &pwm_conf, false);
                    }
                    else if (fade_pins[0] == -1 || fade_pins[1] == -1)
                    {
                        slice = (i == 0) ? pwm_gpio_to_slice_num(fade_pins[1]) : pwm_gpio_to_slice_num(fade_pins[0]);
                        // regular speed if only one pin is fading
                        pwm_config_set_clkdiv(&pwm_conf, 4.0f);
                        pwm_init(slice, &pwm_conf, true);
                    }
                }
            }
        }
        // Square the fade value to make the LED's brightness appear more linear
        // Note this range matches with the wrap value
        pwm_set_gpio_level(irq_pin, fade * fade);
    }
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
    // Tell the LED pin that the PWM is in charge of its value.
    gpio_set_function(PIN_A_GREEN, GPIO_FUNC_PWM);
    pwm_set_gpio_level(PIN_A_GREEN, 0x4000);

    // gpio_set_function(PIN_B_BLUE, GPIO_FUNC_PWM);
    // pwm_set_gpio_level(PIN_B_BLUE, 0xE000);

    uint slice_num;

    // Figure out which slice we just connected to the LED pin
    slice_num = pwm_gpio_to_slice_num(PIN_A_GREEN);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);

    slice_num = pwm_gpio_to_slice_num(PIN_B_BLUE);
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_conf = pwm_get_default_config();
}

int setup_uart()
{
    // Initialise UART 0
    uart_init(UART_ID, BAUD_RATE);

    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}

int main()
{
    setup_gpio();
    setup_irq();
    setup_pwm();
    setup_uart();

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
