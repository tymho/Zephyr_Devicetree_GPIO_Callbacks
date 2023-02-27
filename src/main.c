/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Zephyr_Devicetree_GPIO_Callbacks,LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   10*60*1000

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
#define SW0_NODE	DT_ALIAS(button0) 
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

#define LED0_NODE	DT_ALIAS(heartbeat)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

//static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
//static const struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET(DT_ALIAS(error), gpios);
//static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(button0), gpios);

//static struct gpio_callback button_cb;
//static bool button_event = 0;

/* Callbacks */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    gpio_pin_toggle_dt(&led);
}
static struct gpio_callback button_cb_data;

/**
void button0_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
//button0_event = 1; // conditional statement in main() can now do something based on the event detection
gpio_pin_toggle_dt(&led0);
} 
**/

void main(void)
{
  int err;

  err = !device_is_ready(led.port);
	if (err) {
		LOG_ERR("gpio1 interface not ready.");
    return;
	}

  err = !device_is_ready(button.port);
  if (err) {
    LOG_ERR("gpio0 interface not ready.");
    return;
  }

	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		LOG_ERR("Cannot configure sleep button");
    return;
	}

  err = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (err < 0) {
		return;
	}

  /* Setup callbacks */
  /**
  err = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
  if (err < 0) {
    LOG_ERR("Cannot attach callback to button0.");
  }
  gpio_init_callback(&button0_cb, button0_callback, BIT(button0.pin));
  gpio_add_callback(button0.port, &button0_cb);
  **/
  err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE );
  gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	while (1) {
    /**
    if (button_event) {
    // Do something
      gpio_pin_toggle_dt(&led0);
      button_event = 0;
    }
    **/
		k_msleep(SLEEP_TIME_MS);
	}
}
