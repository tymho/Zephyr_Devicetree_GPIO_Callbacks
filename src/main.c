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
#include <zephyr/drivers/adc.h>

LOG_MODULE_REGISTER(Zephyr_DPC,LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define ONE_HZ 500
#define FIVE_HZ 100
#define ONE_TO_FIVE_DIFF_HZ 400
#define FIVE_TO_TEN_DIFF_HZ 50

#define ADC_DT_SPEC_GET_BY_ALIAS(node_id)                    \
{                                                            \
  .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(node_id))),        \
  .channel_id = DT_REG_ADDR(DT_ALIAS(node_id)),              \
  ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(node_id))            \
}                                                            \

#define DT_SPEC_AND_COMMA(node_id, prop, idx)                \
	      ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* ADC channels (specified in DT overlay) */
static const struct adc_dt_spec adc_vslow = ADC_DT_SPEC_GET_BY_ALIAS(vslow);
static const struct adc_dt_spec adc_vfast = ADC_DT_SPEC_GET_BY_ALIAS(vfast);

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
static float maxV = 3300;
static int dormant = 0;
static int sleep_count = 0;
static int32_t val_mv_slow = 0;
static int32_t val_mv_fast = 0;

/* Timers */
//timer for heartbeat
void fixed_hb(struct k_timer *heartbeats)
{
  gpio_pin_toggle_dt(&hb);
  //LOG_DBG("Hearbeat blinked");
}
K_TIMER_DEFINE(heartbeats, fixed_hb, NULL);

//timer for slower adc blink (sb)
void fixed_sb(struct k_timer *sb_blink)
{
  gpio_pin_toggle_dt(&iv);
  //LOG_DBG("LED3 blinked");
}
K_TIMER_DEFINE(sb_blink, fixed_sb, NULL);

//timer for faster adc blink (fb)
void fixed_fb(struct k_timer *fb_blink)
{
  gpio_pin_toggle_dt(&bz);
  //LOG_DBG("LED2 blinked");
}
K_TIMER_DEFINE(fb_blink, fixed_fb, NULL);

/* Callbacks */
/* Resets everything to base state*/
void button3_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  k_timer_stop(&heartbeats);
  k_timer_stop(&sb_blink);
  k_timer_stop(&fb_blink);
  k_timer_start(&heartbeats, K_MSEC(ONE_HZ), K_MSEC(ONE_HZ));
  k_timer_start(&sb_blink, K_MSEC(ONE_HZ), K_MSEC(ONE_HZ));
  k_timer_start(&fb_blink, K_MSEC(FIVE_HZ), K_MSEC(FIVE_HZ));
}
static struct gpio_callback butt3_cb_data;

/* Sleep and restores to before sleep*/
void button0_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  if (sleep_count == 0){
    sleep_count = 1;
  }
  else {
    sleep_count = 0;
  }
}
static struct gpio_callback butt0_cb_data;

/* Read ADCs */
int32_t read_adc_val_slow()
{
  /* Read adc channel 0*/
  int16_t buf;
  struct adc_sequence sequence = {
    .buffer = &buf,
    .buffer_size = sizeof(buf), // bytes
  };
  LOG_INF("Measuring %s (channel %d)... ", adc_vslow.dev->name, adc_vslow.channel_id);
  (void)adc_sequence_init_dt(&adc_vslow, &sequence);
  int ret;
  ret = adc_read(adc_vslow.dev, &sequence);
  if (ret < 0) {
    LOG_ERR("Could not read (%d)", ret);
  } else {
    LOG_DBG("Raw ADC Buffer: %d", buf);
  }
  int32_t val_mv;
  val_mv = buf;
  ret = adc_raw_to_millivolts_dt(&adc_vslow, &val_mv);
  if (ret < 0) {
    LOG_ERR("Buffer cannot be converted to mV; returning raw buffer value.");
  } else {
    LOG_INF("AIN0 ADC Value (mV): %d", val_mv);
  }
  return val_mv;
}

int32_t read_adc_val_fast()
{
  /* Read adc channel 1*/
  int16_t buf;
  struct adc_sequence sequence = {
    .buffer = &buf,
    .buffer_size = sizeof(buf), // bytes
  };
  LOG_INF("Measuring %s (channel %d)... ", adc_vfast.dev->name, adc_vfast.channel_id);
  (void)adc_sequence_init_dt(&adc_vfast, &sequence);
  int ret;
  ret = adc_read(adc_vfast.dev, &sequence);
  if (ret < 0) {
    LOG_ERR("Could not read (%d)", ret);
  } else {
    LOG_DBG("Raw ADC Buffer: %d", buf);
  }
  int32_t val_mv;
  val_mv = buf;
  ret = adc_raw_to_millivolts_dt(&adc_vfast, &val_mv);
  if (ret < 0) {
    LOG_ERR("Buffer cannot be converted to mV; returning raw buffer value.");
  } else {
    LOG_INF("AIN1 ADC Value (mV): %d", val_mv);
  }
  return val_mv;
}

float slow_period_calc()
{
  //calculate the current period for LED3
  return (float) ONE_HZ - (float) val_mv_slow/maxV * (float) ONE_TO_FIVE_DIFF_HZ;
}

float fast_period_calc()
{
  //calculate the current period for LED2
  return (float) FIVE_HZ - (float) val_mv_fast/maxV * (float) FIVE_TO_TEN_DIFF_HZ;
}

void main(void)
{
  int err;

  /**capture exit or error codes**/
  err = !device_is_ready(adc_vslow.dev);
  if (err){
    LOG_ERR("ADC controller device(s) not ready");
    return -1;
  }

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

  /* Setup ADCs */
  err = adc_channel_setup_dt(&adc_vslow);
  if (err < 0) {
    LOG_ERR("Could not setup vslow ADC channel (%d)", err);
    return err;
  }

  err = adc_channel_setup_dt(&adc_vfast);
  if (err < 0) {
    LOG_ERR("Could not setup vslow ADC channel (%d)", err);
    return err;
  }

  /* Setup LEDs */
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

  // err = gpio_pin_configure_dt(&alrm, GPIO_OUTPUT_LOW);
	// if (err < 0) {
	// 	LOG_ERR("Cannot configure alarm led");
  //   return;
	// }

  err = gpio_pin_configure_dt(&errled, GPIO_OUTPUT_LOW);
	if (err < 0) {
		LOG_ERR("Cannot configure error led");
    return;
	}

  /* Setup buttons */
  err = gpio_pin_configure_dt(&butt0, GPIO_INPUT);
	if (err < 0) {
    LOG_ERR("Cannot configure button0");
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

  err = gpio_pin_interrupt_configure_dt(&butt3, GPIO_INT_EDGE_TO_ACTIVE );
  if (err < 0) {
    LOG_ERR("Cannot configure button3");
		return;
	}
  gpio_init_callback(&butt3_cb_data, button3_pressed, BIT(butt3.pin));
	gpio_add_callback(butt3.port, &butt3_cb_data);

  /*start heartbeat timer*/
  k_timer_start(&heartbeats, K_MSEC(ONE_HZ), K_MSEC(ONE_HZ));

  while (1) {
    /* loop to sample voltage reading and blink LED */
    if (sleep_count == 0){
      //start blinking of LED
      k_timer_start(&sb_blink, K_MSEC((int)slow_period_calc()), K_MSEC((int)slow_period_calc()));
      k_timer_start(&fb_blink, K_MSEC((int)fast_period_calc()), K_MSEC((int)fast_period_calc()));
      //read voltage
      val_mv_slow = read_adc_val_slow();
      val_mv_fast = read_adc_val_fast();
      k_msleep(3*1000); //3 second delay
      //stop timer for reset
      k_timer_stop(&sb_blink);
      k_timer_stop(&fb_blink);
    }
    else {
      //set LEDs to sleep
      gpio_pin_set_dt(&bz, 0);
      gpio_pin_set_dt(&iv, 0);
      k_msleep(100); //0.1 second delay
    }
	}
}
