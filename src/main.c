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
#define LED_ON_TIME_MS 1000
#define DEC_ON_TIME_MS 100
#define INC_ON_TIME_MS 100
#define MIN_TIME_MS 100
#define MAX_TIME_MS 2000

/* LEDs */
#define HB_NODE	DT_ALIAS(heartbeat)
static const struct gpio_dt_spec hb = GPIO_DT_SPEC_GET(HB_NODE, gpios);

#define BZ_NODE	DT_ALIAS(buzzer)
static const struct gpio_dt_spec bz = GPIO_DT_SPEC_GET(BZ_NODE, gpios);

#define IV_NODE	DT_ALIAS(ivdrip)
static const struct gpio_dt_spec iv = GPIO_DT_SPEC_GET(IV_NODE, gpios);

#define ALRM_NODE	DT_ALIAS(alarm)
static const struct gpio_dt_spec alrm = GPIO_DT_SPEC_GET(ALRM_NODE, gpios);

#define ERROR_NODE	DT_ALIAS(error)
static const struct gpio_dt_spec errled = GPIO_DT_SPEC_GET(ERROR_NODE, gpios);

/* Buttons */
#define BUTTON0_NODE	DT_ALIAS(button0) 
static const struct gpio_dt_spec butt0 = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);

#define BUTTON1_NODE	DT_ALIAS(button1) 
static const struct gpio_dt_spec butt1 = GPIO_DT_SPEC_GET(BUTTON1_NODE, gpios);

#define BUTTON2_NODE	DT_ALIAS(button2) 
static const struct gpio_dt_spec butt2 = GPIO_DT_SPEC_GET(BUTTON2_NODE, gpios);

#define BUTTON3_NODE	DT_ALIAS(button3) 
static const struct gpio_dt_spec butt3 = GPIO_DT_SPEC_GET(BUTTON3_NODE, gpios);

//global variables
static int tog_count = 0;
static int inc_count = 0;
static int dec_count = 0;
static int err_count = 0;
static int sleep_count = 0;

/* Callbacks */
/* Decrease delay by 100 ms when pressed */
void button1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  if (err_count == 1){
    return;
  }
  if (LED_ON_TIME_MS - dec_count*DEC_ON_TIME_MS + inc_count*INC_ON_TIME_MS < MIN_TIME_MS){
    gpio_pin_set_dt(&bz, 0);
    gpio_pin_set_dt(&iv, 0);
    gpio_pin_set_dt(&alrm, 0);
    gpio_pin_set_dt(&errled, 1);
    err_count = 1;
  }
  else{
    dec_count = dec_count + 1;
  }
}
static struct gpio_callback butt1_cb_data;

/* Increase delay by 100 ms when pressed*/
void button2_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  if (err_count == 1){
    return;
  }
  if (LED_ON_TIME_MS - dec_count*DEC_ON_TIME_MS + inc_count*INC_ON_TIME_MS > MAX_TIME_MS){
    gpio_pin_set_dt(&bz, 0);
    gpio_pin_set_dt(&iv, 0);
    gpio_pin_set_dt(&alrm, 0);
    gpio_pin_set_dt(&errled, 1);
    err_count = 1;
  }
  else{
    inc_count = inc_count + 1;
  }
}
static struct gpio_callback butt2_cb_data;

/* Resets everything to base state*/
void button3_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  inc_count = 0;
  dec_count = 0;
  err_count = 0;
}
static struct gpio_callback butt3_cb_data;

/* Sleep and restores to before sleep*/
void button0_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  if (sleep_count == 0){
    err_count = 1;
    sleep_count = 1;
    gpio_pin_set_dt(&bz, 0);
    gpio_pin_set_dt(&iv, 0);
    gpio_pin_set_dt(&alrm, 0);
  }
  else {
    err_count = 0;
    sleep_count = 0;
  }
}
static struct gpio_callback butt0_cb_data;

/**function to toggle action LEDs**/
void toggle_leds()
{
  if (err_count == 1) {
    gpio_pin_set_dt(&bz, 0);
    gpio_pin_set_dt(&iv, 0);
    gpio_pin_set_dt(&alrm, 0);
    gpio_pin_set_dt(&errled, 1);
    return;
  }
  if (tog_count == 0) {
    gpio_pin_set_dt(&bz, 1);
    gpio_pin_set_dt(&iv, 0);
    gpio_pin_set_dt(&alrm, 0);
  }
  if (tog_count == 1) {
    gpio_pin_set_dt(&bz, 0);
    gpio_pin_set_dt(&iv, 1);
    gpio_pin_set_dt(&alrm, 0);
  }
  if (tog_count == 2) {
    gpio_pin_set_dt(&bz, 0);
    gpio_pin_set_dt(&iv, 0);
    gpio_pin_set_dt(&alrm, 1);
    tog_count = 0;
  }
  else {
    tog_count = tog_count + 1;
  }
}

void main(void)
{
  int err;

  /**capture exit or error codes**/
  err = !device_is_ready(errled.port);
	if (err) {
		LOG_ERR("gpio1 interface not ready.");
    return;
	}

  err = !device_is_ready(butt0.port);
  if (err) {
    LOG_ERR("gpio0 interface not ready.");
    return;
  }

  err = gpio_pin_configure_dt(&hb, GPIO_OUTPUT_LOW);
	if (err < 0) {
		LOG_ERR("Cannot configure heartbeat led");
    return;
	}

  err = gpio_pin_configure_dt(&bz, GPIO_OUTPUT_LOW);
	if (err < 0) {
		LOG_ERR("Cannot configure buzzer led");
    return;
	}

  err = gpio_pin_configure_dt(&iv, GPIO_OUTPUT_LOW);
	if (err < 0) {
		LOG_ERR("Cannot configure ivdrip led");
    return;
	}

  err = gpio_pin_configure_dt(&alrm, GPIO_OUTPUT_LOW);
	if (err < 0) {
		LOG_ERR("Cannot configure alarm led");
    return;
	}

  err = gpio_pin_configure_dt(&errled, GPIO_OUTPUT_LOW);
	if (err < 0) {
		LOG_ERR("Cannot configure error led");
    return;
	}

  err = gpio_pin_configure_dt(&butt0, GPIO_INPUT);
	if (err < 0) {
    LOG_ERR("Cannot configure button0");
		return;
	}

  err = gpio_pin_configure_dt(&butt1, GPIO_INPUT);
	if (err < 0) {
    LOG_ERR("Cannot configure button1");
		return;
	}

  err = gpio_pin_configure_dt(&butt2, GPIO_INPUT);
	if (err < 0) {
    LOG_ERR("Cannot configure button2");
		return;
	}

  err = gpio_pin_configure_dt(&butt3, GPIO_INPUT);
	if (err < 0) {
    LOG_ERR("Cannot configure button3");
		return;
	}
  /* Setup callbacks */
  err = gpio_pin_interrupt_configure_dt(&butt0, GPIO_INT_EDGE_TO_ACTIVE );
  if (err < 0) {
    LOG_ERR("Cannot configure button0");
		return;
	}
  gpio_init_callback(&butt0_cb_data, button0_pressed, BIT(butt0.pin));
	gpio_add_callback(butt0.port, &butt0_cb_data);

  err = gpio_pin_interrupt_configure_dt(&butt1, GPIO_INT_EDGE_TO_ACTIVE );
  if (err < 0) {
    LOG_ERR("Cannot configure button1");
		return;
	}
  gpio_init_callback(&butt1_cb_data, button1_pressed, BIT(butt1.pin));
	gpio_add_callback(butt1.port, &butt1_cb_data);

  err = gpio_pin_interrupt_configure_dt(&butt2, GPIO_INT_EDGE_TO_ACTIVE );
  if (err < 0) {
    LOG_ERR("Cannot configure button2");
		return;
	}
  gpio_init_callback(&butt2_cb_data, button2_pressed, BIT(butt2.pin));
	gpio_add_callback(butt2.port, &butt2_cb_data);

  err = gpio_pin_interrupt_configure_dt(&butt3, GPIO_INT_EDGE_TO_ACTIVE );
  if (err < 0) {
    LOG_ERR("Cannot configure button3");
		return;
	}
  gpio_init_callback(&butt3_cb_data, button3_pressed, BIT(butt3.pin));
	gpio_add_callback(butt3.port, &butt3_cb_data);

	while (1) {
    /* Setup recurring heartbeat LED */
    gpio_pin_toggle_dt(&hb);
    /* Toggle Action LEDs */
    toggle_leds();
    /* Increment time depending on inputs */
    k_msleep(LED_ON_TIME_MS - dec_count*DEC_ON_TIME_MS + inc_count*INC_ON_TIME_MS);
	}
}
