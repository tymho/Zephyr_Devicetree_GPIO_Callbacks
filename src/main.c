/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// load in the Zephyr library
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>

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

// define structs based on DT aliases
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

/* PWM */
static const struct pwm_dt_spec mtr_drv1 = PWM_DT_SPEC_GET(DT_ALIAS(drv1));
static const struct pwm_dt_spec mtr_drv2 = PWM_DT_SPEC_GET(DT_ALIAS(drv2));

//global variables
static float maxV = 3300;
static int dormant = 0;
static int sleep_count = 0;
static int32_t val_mv_slow = 0;
static int32_t val_mv_fast = 0;
static float saw[20] = {0, 0.05, 0.1 , 0.15, 0.2 , 0.25, 0.3 , 0.35, 0.4 , 0.45, 0.5 ,
       0.55, 0.6 , 0.65, 0.7 , 0.75, 0.8 , 0.85, 0.9 , 0.95};
static int func_ind = 0;
static float func_val = 0;
static float func_hz = 0;

/* Timers */
//timer for heartbeat
void fixed_hb(struct k_timer *heartbeats)
{
  gpio_pin_toggle_dt(&hb);
}
K_TIMER_DEFINE(heartbeats, fixed_hb, NULL);

//timer for LED3
void mod_led2(struct k_timer *pwmled)
{
  if (func_ind == 19) {
    //reset indexing of lookup table if at end of tooth
    func_ind = 0;
  }
  else {
    //else increment the table values
    func_ind += 1;
  }
  int err;
  //modulate PWM based on table value
  err = pwm_set_pulse_dt(&mtr_drv2, mtr_drv2.period * saw[func_ind]); // % duty cycle based on ADC1 reading    
  if (err) {
    LOG_ERR("Could not set motor driver 2 (PWM1)");
  }
}
K_TIMER_DEFINE(pwmled, mod_led2, NULL);

/* Callbacks */
/* Resets everything to base state*/
void button3_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  k_timer_stop(&heartbeats);
  k_timer_start(&heartbeats, K_MSEC(ONE_HZ), K_MSEC(ONE_HZ));
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

  if (!device_is_ready(mtr_drv1.dev)) {
    LOG_ERR("PWM device %s is not ready.", mtr_drv1.dev->name);
    return -1;
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

  /*start timer*/
  k_timer_start(&heartbeats, K_MSEC(ONE_HZ), K_MSEC(ONE_HZ));

  while (1) {
    /* loop to sample voltage reading and blink LED */
    if (sleep_count == 0){
      //read voltage of channel 0 and 1
      val_mv_slow = read_adc_val_slow();
      val_mv_fast = read_adc_val_fast();

      //modulate LED3 brightness based on adc channel 0 voltage
      err = pwm_set_pulse_dt(&mtr_drv1, mtr_drv1.period * (float) val_mv_slow / maxV); // % duty cycle based on ADC0 reading
      if (err) {
        LOG_ERR("Could not set motor driver 1 (PWM0)");
      }

      //calculate sawtooth hz based on adc channel 1 voltage
      float adchz = 10 - 5 * (float) val_mv_fast / maxV;
      //start timer for LED2 brightness
      k_timer_start(&pwmled, K_MSEC(adchz), K_MSEC(adchz));

      k_msleep(3*1000); //3 second delay
      k_timer_stop(&pwmled);
    }
    else {
      //set LEDs to sleep
      gpio_pin_set_dt(&bz, 0);
      gpio_pin_set_dt(&iv, 0);
      k_msleep(100); //0.1 second delay
    }
	}
}
