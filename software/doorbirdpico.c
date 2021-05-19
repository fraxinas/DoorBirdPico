#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

#define PIN_A_KEY 5
#define PIN_A_ANODE 6
#define PIN_A_BLUE 7
#define PIN_A_GREEN 8
#define PIN_A_RED 9

uint slice_num_a, slice_num_b;
pwm_config config_a;

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

static char event_str[128];

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

void key_a_callback(uint gpio, uint32_t events)
{
    gpio_event_string(event_str, events);
    printf("GPIO KEY A %d %s\n", gpio, event_str);
    if (events & 0x8)
    {
        // Load the configuration into our PWM slice, and set it running.
        pwm_init(slice_num_a, &config_a, true);
    }
}

int setup_irq()
{
    stdio_init_all();
    printf("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback(PIN_A_KEY, GPIO_IRQ_EDGE_RISE, true, &key_a_callback);
}

void on_pwm_wrap()
{
    static int fade = 0;
    static bool going_up = true;
    static int count = 5;
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PIN_A_GREEN));

    if (going_up)
    {
        ++fade;
        if (fade > 255)
        {
            fade = 255;
            going_up = false;
        }
    }
    else
    {
        --fade;
        if (fade < 0)
        {
            fade = 0;
            going_up = true;
        }
    }
    // Square the fade value to make the LED's brightness appear more linear
    // Note this range matches with the wrap value
    pwm_set_gpio_level(PIN_A_GREEN, fade * fade);

    if (--count == 0)
    {
        pwm_init(slice_num_a, &config_a, false);
    }
    else
    {
        on_pwm_wrap();
    }
}

int setup_pwm()
{
    // Tell the LED pin that the PWM is in charge of its value.
    gpio_set_function(PIN_A_GREEN, GPIO_FUNC_PWM);
    pwm_set_gpio_level(PIN_A_GREEN, 0x2000);

    // Figure out which slice we just connected to the LED pin
    uint slice_num_a = pwm_gpio_to_slice_num(PIN_A_GREEN);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num_a);
    pwm_set_irq_enabled(slice_num_a, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    config_a = pwm_get_default_config();
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&config_a, 4.f);
}

int main()
{
    setup_irq();
    setup_pwm();

    // Everything after this point happens in the PWM interrupt handler, so we
    // can twiddle our thumbs
    while (1)
        tight_loop_contents();
}
