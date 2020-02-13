/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu.c
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "inv_mpu.h"

/* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 * reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin)
 * labs(long x)
 * fabsf(float x)
 * min(int a, int b)
 */
#if defined EMPL_TARGET_STM32F4
#include "i2c.h"   
#include "main.h"
#include "log.h"
#include "board-st_discovery.h"
   
#define i2c_write   Sensors_I2C_WriteRegister
#define i2c_read    Sensors_I2C_ReadRegister 
#define delay_ms    mdelay
#define get_ms      get_tick_count
#define log_i       MPL_LOGI
#define log_e       MPL_LOGE
#define min(a,b) ((a<b)?a:b)
   
#elif defined MOTION_DRIVER_TARGET_MSP430
#include "msp430.h"
#include "msp430_i2c.h"
#include "msp430_clock.h"
#include "msp430_interrupt.h"
#define i2c_write   msp430_i2c_write
#define i2c_read    msp430_i2c_read
#define delay_ms    msp430_delay_ms
#define get_ms      msp430_get_clock_ms
static inline int reg_int_cb(struct int_param_s *int_param)
{
    return msp430_reg_int_cb(int_param->cb, int_param->pin, int_param->lp_exit,
        int_param->active_low);
}
#define log_i(...)     do {} while (0)
#define log_e(...)     do {} while (0)
/* labs is already defined by TI's toolchain. */
/* fabs is for doubles. fabsf is for floats. */
#define fabs        fabsf
#define min(a,b) ((a<b)?a:b)
#elif defined EMPL_TARGET_MSP430
#include "msp430.h"
#include "msp430_i2c.h"
#include "msp430_clock.h"
#include "msp430_interrupt.h"
#include "log.h"
#define i2c_write   msp430_i2c_write
#define i2c_read    msp430_i2c_read
#define delay_ms    msp430_delay_ms
#define get_ms      msp430_get_clock_ms
static inline int reg_int_cb(struct int_param_s *int_param)
{
    return msp430_reg_int_cb(int_param->cb, int_param->pin, int_param->lp_exit,
        int_param->active_low);
}
#define log_i       mpl_logi
#define log_e       mpl_loge
/* labs is already defined by ti's toolchain. */
/* fabs is for doubles. fabsf is for floats. */
#define fabs        fabsf
#define min(a,b) ((a<b)?a:b)
#elif defined empl_target_uc3l0
/* instead of using the standard twi driver from the asf library, we're using
 * a twi driver that follows the slave address + register address convention.
 */
#include "twi.h"
#include "delay.h"
#include "sysclk.h"
#include "log.h"
#include "sensors_xplained.h"
#include "uc3l0_clock.h"
#define i2c_write(a, b, c, d)   twi_write(a, b, d, c)
#define i2c_read(a, b, c, d)    twi_read(a, b, d, c)
/* delay_ms is a function already defined in asf. */
#define get_ms  uc3l0_get_clock_ms
static inline int reg_int_cb(struct int_param_s *int_param)
{
    sensor_board_irq_connect(int_param->pin, int_param->cb, int_param->arg);
    return 0;
}
#define log_i       mpl_logi
#define log_e       mpl_loge
/* uc3 is a 32-bit processor, so abs and labs are equivalent. */
#define labs        abs
#define fabs(x)     (((x)>0)?(x):-(x))
#else
#error  gyro driver is missing the system layer implementations.
#endif

#if !defined mpu6050 && !defined mpu9150 && !defined mpu6500 && !defined mpu9250
#error  which gyro are you using? define mpuxxxx in your compiler options.
#endif

/* time for some messy macro work. =]
 * #define mpu9150
 * is equivalent to..
 * #define mpu6050
 * #define ak8975_secondary
 *
 * #define mpu9250
 * is equivalent to..
 * #define mpu6500
 * #define ak8963_secondary
 */
#if defined mpu9150
#ifndef mpu6050
#define mpu6050
#endif                          /* #ifndef mpu6050 */
#if defined ak8963_secondary
#error "mpu9150 and ak8963_secondary cannot both be defined."
#elif !defined ak8975_secondary /* #if defined ak8963_secondary */
#define ak8975_secondary
#endif                          /* #if defined ak8963_secondary */
#elif defined mpu9250           /* #if defined mpu9150 */
#ifndef mpu6500
#define mpu6500
#endif                          /* #ifndef mpu6500 */
#if defined ak8975_secondary
#error "mpu9250 and ak8975_secondary cannot both be defined."
#elif !defined ak8963_secondary /* #if defined ak8975_secondary */
#define ak8963_secondary
#endif                          /* #if defined ak8975_secondary */
#endif                          /* #if defined mpu9150 */

#if defined ak8975_secondary || defined ak8963_secondary
#define ak89xx_secondary
#else
/* #warning "no compass = less profit for invensense. lame." */
#endif

static int set_int_enable(unsigned char enable);

/* hardware registers needed by driver. */
struct gyro_reg_s {
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
    unsigned char prod_id;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char accel_cfg2;
    unsigned char lp_accel_odr;
    unsigned char motion_thr;
    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char accel_intel;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
    unsigned char mem_r_w;
    unsigned char accel_offs;
    unsigned char i2c_mst;
    unsigned char bank_sel;
    unsigned char mem_start_addr;
    unsigned char prgm_start_h;
#if defined ak89xx_secondary
    unsigned char s0_addr;
    unsigned char s0_reg;
    unsigned char s0_ctrl;
    unsigned char s1_addr;
    unsigned char s1_reg;
    unsigned char s1_ctrl;
    unsigned char s4_ctrl;
    unsigned char s0_do;
    unsigned char s1_do;
    unsigned char i2c_delay_ctrl;
    unsigned char raw_compass;
    /* the i2c_mst_vddio bit is in this register. */
    unsigned char yg_offs_tc;
#endif
};

/* information specific to a particular device. */
struct hw_s {
    unsigned char addr;
    unsigned short max_fifo;
    unsigned char num_reg;
    unsigned short temp_sens;
    short temp_offset;
    unsigned short bank_size;
#if defined ak89xx_secondary
    unsigned short compass_fsr;
#endif
};

/* when entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * todo: this is tacky. fix it.
 */
struct motion_int_cache_s {
    unsigned short gyro_fsr;
    unsigned char accel_fsr;
    unsigned short lpf;
    unsigned short sample_rate;
    unsigned char sensors_on;
    unsigned char fifo_sensors;
    unsigned char dmp_on;
};

/* cached chip configuration data.
 * todo: a lot of these can be handled with a bitmask.
 */
struct chip_cfg_s {
    /* matches gyro_cfg >> 3 & 0x03 */
    unsigned char gyro_fsr;
    /* matches accel_cfg >> 3 & 0x03 */
    unsigned char accel_fsr;
    /* enabled sensors. uses same masks as fifo_en, not pwr_mgmt_2. */
    unsigned char sensors;
    /* matches config register. */
    unsigned char lpf;
    unsigned char clk_src;
    /* sample rate, not rate divider. */
    unsigned short sample_rate;
    /* matches fifo_en register. */
    unsigned char fifo_enable;
    /* matches int enable register. */
    unsigned char int_enable;
    /* 1 if devices on auxiliary i2c bus appear on the primary. */
    unsigned char bypass_mode;
    /* 1 if half-sensitivity.
     * note: this doesn't belong here, but everything else in hw_s is const,
     * and this allows us to save some precious ram.
     */
    unsigned char accel_half;
    /* 1 if device in low-power accel-only mode. */
    unsigned char lp_accel_mode;
    /* 1 if interrupts are only triggered on motion events. */
    unsigned char int_motion_only;
    struct motion_int_cache_s cache;
    /* 1 for active low interrupts. */
    unsigned char active_low_int;
    /* 1 for latched interrupts. */
    unsigned char latched_int;
    /* 1 if dmp is enabled. */
    unsigned char dmp_on;
    /* ensures that dmp will only be loaded once. */
    unsigned char dmp_loaded;
    /* sampling rate used when dmp is enabled. */
    unsigned short dmp_sample_rate;
#ifdef ak89xx_secondary
    /* compass sample rate. */
    unsigned short compass_sample_rate;
    unsigned char compass_addr;
    short mag_sens_adj[3];
#endif
};

/* information for self-test. */
struct test_s {
    unsigned long gyro_sens;
    unsigned long accel_sens;
    unsigned char reg_rate_div;
    unsigned char reg_lpf;
    unsigned char reg_gyro_fsr;
    unsigned char reg_accel_fsr;
    unsigned short wait_ms;
    unsigned char packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;
#ifdef mpu6500
    float max_g_offset;
    unsigned short sample_wait_ms;
#endif
};

/* gyro driver state variables. */
struct gyro_state_s {
    const struct gyro_reg_s *reg;
    const struct hw_s *hw;
    struct chip_cfg_s chip_cfg;
    const struct test_s *test;
};

/* filter configurations. */
enum lpf_e {
    inv_filter_256hz_nolpf2 = 0,
    inv_filter_188hz,
    inv_filter_98hz,
    inv_filter_42hz,
    inv_filter_20hz,
    inv_filter_10hz,
    inv_filter_5hz,
    inv_filter_2100hz_nolpf,
    num_filter
};

/* full scale ranges. */
enum gyro_fsr_e {
    inv_fsr_250dps = 0,
    inv_fsr_500dps,
    inv_fsr_1000dps,
    inv_fsr_2000dps,
    num_gyro_fsr
};

/* full scale ranges. */
enum accel_fsr_e {
    inv_fsr_2g = 0,
    inv_fsr_4g,
    inv_fsr_8g,
    inv_fsr_16g,
    num_accel_fsr
};

/* clock sources. */
enum clock_sel_e {
    inv_clk_internal = 0,
    inv_clk_pll,
    num_clk
};

/* low-power accel wakeup rates. */
enum lp_accel_rate_e {
#if defined mpu6050
    inv_lpa_1_25hz,
    inv_lpa_5hz,
    inv_lpa_20hz,
    inv_lpa_40hz
#elif defined mpu6500
    inv_lpa_0_3125hz,
    inv_lpa_0_625hz,
    inv_lpa_1_25hz,
    inv_lpa_2_5hz,
    inv_lpa_5hz,
    inv_lpa_10hz,
    inv_lpa_20hz,
    inv_lpa_40hz,
    inv_lpa_80hz,
    inv_lpa_160hz,
    inv_lpa_320hz,
    inv_lpa_640hz
#endif
};

#define bit_i2c_mst_vddio   (0x80)
#define bit_fifo_en         (0x40)
#define bit_dmp_en          (0x80)
#define bit_fifo_rst        (0x04)
#define bit_dmp_rst         (0x08)
#define bit_fifo_overflow   (0x10)
#define bit_data_rdy_en     (0x01)
#define bit_dmp_int_en      (0x02)
#define bit_mot_int_en      (0x40)
#define bits_fsr            (0x18)
#define bits_lpf            (0x07)
#define bits_hpf            (0x07)
#define bits_clk            (0x07)
#define bit_fifo_size_1024  (0x40)
#define bit_fifo_size_2048  (0x80)
#define bit_fifo_size_4096  (0xc0)
#define bit_reset           (0x80)
#define bit_sleep           (0x40)
#define bit_s0_delay_en     (0x01)
#define bit_s2_delay_en     (0x04)
#define bits_slave_length   (0x0f)
#define bit_slave_byte_sw   (0x40)
#define bit_slave_group     (0x10)
#define bit_slave_en        (0x80)
#define bit_i2c_read        (0x80)
#define bits_i2c_master_dly (0x1f)
#define bit_aux_if_en       (0x20)
#define bit_actl            (0x80)
#define bit_latch_en        (0x20)
#define bit_any_rd_clr      (0x10)
#define bit_bypass_en       (0x02)
#define bits_wom_en         (0xc0)
#define bit_lpa_cycle       (0x20)
#define bit_stby_xa         (0x20)
#define bit_stby_ya         (0x10)
#define bit_stby_za         (0x08)
#define bit_stby_xg         (0x04)
#define bit_stby_yg         (0x02)
#define bit_stby_zg         (0x01)
#define bit_stby_xyza       (bit_stby_xa | bit_stby_ya | bit_stby_za)
#define bit_stby_xyzg       (bit_stby_xg | bit_stby_yg | bit_stby_zg)

#if defined ak8975_secondary
#define supports_ak89xx_high_sens   (0x00)
#define ak89xx_fsr                  (9830)
#elif defined ak8963_secondary
#define supports_ak89xx_high_sens   (0x10)
#define ak89xx_fsr                  (4915)
#endif

#ifdef ak89xx_secondary
#define akm_reg_whoami      (0x00)

#define akm_reg_st1         (0x02)
#define akm_reg_hxl         (0x03)
#define akm_reg_st2         (0x09)

#define akm_reg_cntl        (0x0a)
#define akm_reg_astc        (0x0c)
#define akm_reg_asax        (0x10)
#define akm_reg_asay        (0x11)
#define akm_reg_asaz        (0x12)

#define akm_data_ready      (0x01)
#define akm_data_overrun    (0x02)
#define akm_overflow        (0x80)
#define akm_data_error      (0x40)

#define akm_bit_self_test   (0x40)

#define akm_power_down          (0x00 | supports_ak89xx_high_sens)
#define akm_single_measurement  (0x01 | supports_ak89xx_high_sens)
#define akm_fuse_rom_access     (0x0f | supports_ak89xx_high_sens)
#define akm_mode_self_test      (0x08 | supports_ak89xx_high_sens)

#define akm_whoami      (0x48)
#endif

#if defined mpu6050
const struct gyro_reg_s reg = {
    .who_am_i       = 0x75,
    .rate_div       = 0x19,
    .lpf            = 0x1a,
    .prod_id        = 0x0c,
    .user_ctrl      = 0x6a,
    .fifo_en        = 0x23,
    .gyro_cfg       = 0x1b,
    .accel_cfg      = 0x1c,
    .motion_thr     = 0x1f,
    .motion_dur     = 0x20,
    .fifo_count_h   = 0x72,
    .fifo_r_w       = 0x74,
    .raw_gyro       = 0x43,
    .raw_accel      = 0x3b,
    .temp           = 0x41,
    .int_enable     = 0x38,
    .dmp_int_status = 0x39,
    .int_status     = 0x3a,
    .pwr_mgmt_1     = 0x6b,
    .pwr_mgmt_2     = 0x6c,
    .int_pin_cfg    = 0x37,
    .mem_r_w        = 0x6f,
    .accel_offs     = 0x06,
    .i2c_mst        = 0x24,
    .bank_sel       = 0x6d,
    .mem_start_addr = 0x6e,
    .prgm_start_h   = 0x70
#ifdef ak89xx_secondary
    ,.raw_compass   = 0x49,
    .yg_offs_tc     = 0x01,
    .s0_addr        = 0x25,
    .s0_reg         = 0x26,
    .s0_ctrl        = 0x27,
    .s1_addr        = 0x28,
    .s1_reg         = 0x29,
    .s1_ctrl        = 0x2a,
    .s4_ctrl        = 0x34,
    .s0_do          = 0x63,
    .s1_do          = 0x64,
    .i2c_delay_ctrl = 0x67
#endif
};
const struct hw_s hw = {
    .addr           = 0x68,
    .max_fifo       = 1024,
    .num_reg        = 118,
    .temp_sens      = 340,
    .temp_offset    = -521,
    .bank_size      = 256
#if defined ak89xx_secondary
    ,.compass_fsr    = ak89xx_fsr
#endif
};

const struct test_s test = {
    .gyro_sens      = 32768/250,
    .accel_sens     = 32768/16,
    .reg_rate_div   = 0,    /* 1khz. */
    .reg_lpf        = 1,    /* 188hz. */
    .reg_gyro_fsr   = 0,    /* 250dps. */
    .reg_accel_fsr  = 0x18, /* 16g. */
    .wait_ms        = 50,
    .packet_thresh  = 5,    /* 5% */
    .min_dps        = 10.f,
    .max_dps        = 105.f,
    .max_gyro_var   = 0.14f,
    .min_g          = 0.3f,
    .max_g          = 0.95f,
    .max_accel_var  = 0.14f
};

static struct gyro_state_s st = {
    .reg = &reg,
    .hw = &hw,
    .test = &test
};
#elif defined mpu6500
const struct gyro_reg_s reg = {
    .who_am_i       = 0x75,
    .rate_div       = 0x19,
    .lpf            = 0x1a,
    .prod_id        = 0x0c,
    .user_ctrl      = 0x6a,
    .fifo_en        = 0x23,
    .gyro_cfg       = 0x1b,
    .accel_cfg      = 0x1c,
    .accel_cfg2     = 0x1d,
    .lp_accel_odr   = 0x1e,
    .motion_thr     = 0x1f,
    .motion_dur     = 0x20,
    .fifo_count_h   = 0x72,
    .fifo_r_w       = 0x74,
    .raw_gyro       = 0x43,
    .raw_accel      = 0x3b,
    .temp           = 0x41,
    .int_enable     = 0x38,
    .dmp_int_status = 0x39,
    .int_status     = 0x3a,
    .accel_intel    = 0x69,
    .pwr_mgmt_1     = 0x6b,
    .pwr_mgmt_2     = 0x6c,
    .int_pin_cfg    = 0x37,
    .mem_r_w        = 0x6f,
    .accel_offs     = 0x77,
    .i2c_mst        = 0x24,
    .bank_sel       = 0x6d,
    .mem_start_addr = 0x6e,
    .prgm_start_h   = 0x70
#ifdef ak89xx_secondary
    ,.raw_compass   = 0x49,
    .s0_addr        = 0x25,
    .s0_reg         = 0x26,
    .s0_ctrl        = 0x27,
    .s1_addr        = 0x28,
    .s1_reg         = 0x29,
    .s1_ctrl        = 0x2a,
    .s4_ctrl        = 0x34,
    .s0_do          = 0x63,
    .s1_do          = 0x64,
    .i2c_delay_ctrl = 0x67
#endif
};
const struct hw_s hw = {
    .addr           = 0x68,
    .max_fifo       = 1024,
    .num_reg        = 128,
    .temp_sens      = 321,
    .temp_offset    = 0,
    .bank_size      = 256
#if defined ak89xx_secondary
    ,.compass_fsr    = ak89xx_fsr
#endif
};

const struct test_s test = {
    .gyro_sens      = 32768/250,
    .accel_sens     = 32768/2,  //fsr = +-2g = 16384 lsb/g
    .reg_rate_div   = 0,    /* 1khz. */
    .reg_lpf        = 2,    /* 92hz low pass filter*/
    .reg_gyro_fsr   = 0,    /* 250dps. */
    .reg_accel_fsr  = 0x0,  /* accel fsr setting = 2g. */
    .wait_ms        = 200,   //200ms stabilization time
    .packet_thresh  = 200,    /* 200 samples */
    .min_dps        = 20.f,  //20 dps for gyro criteria c
    .max_dps        = 60.f, //must exceed 60 dps threshold for gyro criteria b
    .max_gyro_var   = .5f, //must exceed +50% variation for gyro criteria a
    .min_g          = .225f, //accel must exceed min 225 mg for criteria b
    .max_g          = .675f, //accel cannot exceed max 675 mg for criteria b
    .max_accel_var  = .5f,  //accel must be within 50% variation for criteria a
    .max_g_offset   = .5f,   //500 mg for accel criteria c
    .sample_wait_ms = 10    //10ms sample time wait
};

static struct gyro_state_s st = {
    .reg = &reg,
    .hw = &hw,
    .test = &test
};
#endif

#define max_packet_length (12)
#ifdef mpu6500
#define hwst_max_packet_length (512)
#endif

#ifdef ak89xx_secondary
static int setup_compass(void);
#define max_compass_sample_rate (100)
#endif

/**
 *  @brief      enable/disable data ready interrupt.
 *  if the dmp is on, the dmp interrupt is enabled. otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
static int set_int_enable(unsigned char enable)
{
    unsigned char tmp;

    if (st.chip_cfg.dmp_on) {
        if (enable)
            tmp = bit_dmp_int_en;
        else
            tmp = 0x00;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
            return -1;
        st.chip_cfg.int_enable = tmp;
    } else {
        if (!st.chip_cfg.sensors)
            return -1;
        if (enable && st.chip_cfg.int_enable)
            return 0;
        if (enable)
            tmp = bit_data_rdy_en;
        else
            tmp = 0x00;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
            return -1;
        st.chip_cfg.int_enable = tmp;
    }
    return 0;
}

/**
 *  @brief      register dump for testing.
 *  @return     0 if successful.
 */
int mpu_reg_dump(void)
{
    unsigned char ii;
    unsigned char data;

    for (ii = 0; ii < st.hw->num_reg; ii++) {
        if (ii == st.reg->fifo_r_w || ii == st.reg->mem_r_w)
            continue;
        if (i2c_read(st.hw->addr, ii, 1, &data))
            return -1;
        log_i("%#5x: %#5x\r\n", ii, data);
    }
    return 0;
}

/**
 *  @brief      read from a single register.
 *  note: the memory and fifo read/write registers cannot be accessed.
 *  @param[in]  reg     register address.
 *  @param[out] data    register data.
 *  @return     0 if successful.
 */
int mpu_read_reg(unsigned char reg, unsigned char *data)
{
    if (reg == st.reg->fifo_r_w || reg == st.reg->mem_r_w)
        return -1;
    if (reg >= st.hw->num_reg)
        return -1;
    return i2c_read(st.hw->addr, reg, 1, data);
}

/**
 *  @brief      initialize hardware.
 *  initial configuration:\n
 *  gyro fsr: +/- 2000dps\n
 *  accel fsr +/- 2g\n
 *  dlpf: 42hz\n
 *  fifo rate: 50hz\n
 *  clock source: gyro pll\n
 *  fifo: disabled.\n
 *  data ready interrupt: disabled, active low, unlatched.
 *  @param[in]  int_param   platform-specific parameters to interrupt api.
 *  @return     0 if successful.
 */
int mpu_init(struct int_param_s *int_param)
{
    unsigned char data[6];

    /* reset device. */
    data[0] = bit_reset;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
        return -1;
    delay_ms(100);

    /* wake up chip. */
    data[0] = 0x00;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
        return -1;

   st.chip_cfg.accel_half = 0;

#ifdef mpu6500
    /* mpu6500 shares 4kb of memory between the dmp and the fifo. since the
     * first 3kb are needed by the dmp, we'll use the last 1kb for the fifo.
     */
    data[0] = bit_fifo_size_1024 | 0x8;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, data))
        return -1;
#endif

    /* set to invalid values to ensure no i2c writes are skipped. */
    st.chip_cfg.sensors = 0xff;
    st.chip_cfg.gyro_fsr = 0xff;
    st.chip_cfg.accel_fsr = 0xff;
    st.chip_cfg.lpf = 0xff;
    st.chip_cfg.sample_rate = 0xffff;
    st.chip_cfg.fifo_enable = 0xff;
    st.chip_cfg.bypass_mode = 0xff;
#ifdef ak89xx_secondary
    st.chip_cfg.compass_sample_rate = 0xffff;
#endif
    /* mpu_set_sensors always preserves this setting. */
    st.chip_cfg.clk_src = inv_clk_pll;
    /* handled in next call to mpu_set_bypass. */
    st.chip_cfg.active_low_int = 1;
    st.chip_cfg.latched_int = 0;
    st.chip_cfg.int_motion_only = 0;
    st.chip_cfg.lp_accel_mode = 0;
    memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache));
    st.chip_cfg.dmp_on = 0;
    st.chip_cfg.dmp_loaded = 0;
    st.chip_cfg.dmp_sample_rate = 0;

    if (mpu_set_gyro_fsr(2000))
        return -1;
    if (mpu_set_accel_fsr(2))
        return -1;
    if (mpu_set_lpf(42))
        return -1;
    if (mpu_set_sample_rate(50))
        return -1;
    if (mpu_configure_fifo(0))
        return -1;

#ifndef empl_target_stm32f4
    if (int_param)
        reg_int_cb(int_param);
#endif

#ifdef ak89xx_secondary
    setup_compass();
    if (mpu_set_compass_sample_rate(10))
        return -1;
#else
    /* already disabled by setup_compass. */
    if (mpu_set_bypass(0))
        return -1;
#endif

    mpu_set_sensors(0);
    return 0;
}

/**
 *  @brief      enter low-power accel-only mode.
 *  in low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  \n mpu6050: 1.25hz, 5hz, 20hz, 40hz
 *  \n mpu6500: 1.25hz, 2.5hz, 5hz, 10hz, 20hz, 40hz, 80hz, 160hz, 320hz, 640hz
 *  \n if the requested rate is not one listed above, the device will be set to
 *  the next highest rate. requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  \n to select a fractional wake-up frequency, round down the value passed to
 *  @e rate.
 *  @param[in]  rate        minimum sampling rate, or zero to disable lp
 *                          accel mode.
 *  @return     0 if successful.
 */
int mpu_lp_accel_mode(unsigned short rate)
{
    unsigned char tmp[2];

    if (rate > 40)
        return -1;

    if (!rate) {
        mpu_set_int_latched(0);
        tmp[0] = 0;
        tmp[1] = bit_stby_xyzg;
        if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
            return -1;
        st.chip_cfg.lp_accel_mode = 0;
        return 0;
    }
    /* for lp accel, we automatically configure the hardware to produce latched
     * interrupts. in lp accel mode, the hardware cycles into sleep mode before
     * it gets a chance to deassert the interrupt pin; therefore, we shift this
     * responsibility over to the mcu.
     *
     * any register read will clear the interrupt.
     */
    mpu_set_int_latched(1);
#if defined mpu6050
    tmp[0] = bit_lpa_cycle;
    if (rate == 1) {
        tmp[1] = inv_lpa_1_25hz;
        mpu_set_lpf(5);
    } else if (rate <= 5) {
        tmp[1] = inv_lpa_5hz;
        mpu_set_lpf(5);
    } else if (rate <= 20) {
        tmp[1] = inv_lpa_20hz;
        mpu_set_lpf(10);
    } else {
        tmp[1] = inv_lpa_40hz;
        mpu_set_lpf(20);
    }
    tmp[1] = (tmp[1] << 6) | bit_stby_xyzg;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
        return -1;
#elif defined mpu6500
    /* set wake frequency. */
    if (rate == 1)
        tmp[0] = inv_lpa_1_25hz;
    else if (rate == 2)
        tmp[0] = inv_lpa_2_5hz;
    else if (rate <= 5)
        tmp[0] = inv_lpa_5hz;
    else if (rate <= 10)
        tmp[0] = inv_lpa_10hz;
    else if (rate <= 20)
        tmp[0] = inv_lpa_20hz;
    else if (rate <= 40)
        tmp[0] = inv_lpa_40hz;
    else if (rate <= 80)
        tmp[0] = inv_lpa_80hz;
    else if (rate <= 160)
        tmp[0] = inv_lpa_160hz;
    else if (rate <= 320)
        tmp[0] = inv_lpa_320hz;
    else
        tmp[0] = inv_lpa_640hz;
    if (i2c_write(st.hw->addr, st.reg->lp_accel_odr, 1, tmp))
        return -1;
    tmp[0] = bit_lpa_cycle;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, tmp))
        return -1;
#endif
    st.chip_cfg.sensors = inv_xyz_accel;
    st.chip_cfg.clk_src = 0;
    st.chip_cfg.lp_accel_mode = 1;
    mpu_configure_fifo(0);

    return 0;
}

/**
 *  @brief      read raw gyro data directly from the registers.
 *  @param[out] data        raw data in hardware units.
 *  @param[out] timestamp   timestamp in milliseconds. null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_gyro_reg(short *data, unsigned long *timestamp)
{
    unsigned char tmp[6];

    if (!(st.chip_cfg.sensors & inv_xyz_gyro))
        return -1;

    if (i2c_read(st.hw->addr, st.reg->raw_gyro, 6, tmp))
        return -1;
    data[0] = (tmp[0] << 8) | tmp[1];
    data[1] = (tmp[2] << 8) | tmp[3];
    data[2] = (tmp[4] << 8) | tmp[5];
    if (timestamp)
        get_ms(timestamp);
    return 0;
}

/**
 *  @brief      read raw accel data directly from the registers.
 *  @param[out] data        raw data in hardware units.
 *  @param[out] timestamp   timestamp in milliseconds. null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_accel_reg(short *data, unsigned long *timestamp)
{
    unsigned char tmp[6];

    if (!(st.chip_cfg.sensors & inv_xyz_accel))
        return -1;

    if (i2c_read(st.hw->addr, st.reg->raw_accel, 6, tmp))
        return -1;
    data[0] = (tmp[0] << 8) | tmp[1];
    data[1] = (tmp[2] << 8) | tmp[3];
    data[2] = (tmp[4] << 8) | tmp[5];
    if (timestamp)
        get_ms(timestamp);
    return 0;
}

/**
 *  @brief      read temperature data directly from the registers.
 *  @param[out] data        data in q16 format.
 *  @param[out] timestamp   timestamp in milliseconds. null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_temperature(long *data, unsigned long *timestamp)
{
    unsigned char tmp[2];
    short raw;

    if (!(st.chip_cfg.sensors))
        return -1;

    if (i2c_read(st.hw->addr, st.reg->temp, 2, tmp))
        return -1;
    raw = (tmp[0] << 8) | tmp[1];
    if (timestamp)
        get_ms(timestamp);

    data[0] = (long)((35 + ((raw - (float)st.hw->temp_offset) / st.hw->temp_sens)) * 65536l);
    return 0;
}

/**
 *  @brief      read biases to the accel bias 6500 registers.
 *  this function reads from the mpu6500 accel offset cancellations registers.
 *  the format are g in +-8g format. the register is initialized with otp
 *  factory trim values.
 *  @param[in]  accel_bias  returned structure with the accel bias
 *  @return     0 if successful.
 */
int mpu_read_6500_accel_bias(long *accel_bias) {
	unsigned char data[6];
	if (i2c_read(st.hw->addr, 0x77, 2, &data[0]))
		return -1;
	if (i2c_read(st.hw->addr, 0x7a, 2, &data[2]))
		return -1;
	if (i2c_read(st.hw->addr, 0x7d, 2, &data[4]))
		return -1;
	accel_bias[0] = ((long)data[0]<<8) | data[1];
	accel_bias[1] = ((long)data[2]<<8) | data[3];
	accel_bias[2] = ((long)data[4]<<8) | data[5];
	return 0;
}

/**
 *  @brief      read biases to the accel bias 6050 registers.
 *  this function reads from the mpu6050 accel offset cancellations registers.
 *  the format are g in +-8g format. the register is initialized with otp
 *  factory trim values.
 *  @param[in]  accel_bias  returned structure with the accel bias
 *  @return     0 if successful.
 */
int mpu_read_6050_accel_bias(long *accel_bias) {
	unsigned char data[6];
	if (i2c_read(st.hw->addr, 0x06, 2, &data[0]))
		return -1;
	if (i2c_read(st.hw->addr, 0x08, 2, &data[2]))
		return -1;
	if (i2c_read(st.hw->addr, 0x0a, 2, &data[4]))
		return -1;
	accel_bias[0] = ((long)data[0]<<8) | data[1];
	accel_bias[1] = ((long)data[2]<<8) | data[3];
	accel_bias[2] = ((long)data[4]<<8) | data[5];
	return 0;
}

int mpu_read_6500_gyro_bias(long *gyro_bias) {
	unsigned char data[6];
	if (i2c_read(st.hw->addr, 0x13, 2, &data[0]))
		return -1;
	if (i2c_read(st.hw->addr, 0x15, 2, &data[2]))
		return -1;
	if (i2c_read(st.hw->addr, 0x17, 2, &data[4]))
		return -1;
	gyro_bias[0] = ((long)data[0]<<8) | data[1];
	gyro_bias[1] = ((long)data[2]<<8) | data[3];
	gyro_bias[2] = ((long)data[4]<<8) | data[5];
	return 0;
}

/**
 *  @brief      push biases to the gyro bias 6500/6050 registers.
 *  this function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. bias inputs are lsb
 *  in +-1000dps format.
 *  @param[in]  gyro_bias  new biases.
 *  @return     0 if successful.
 */
int mpu_set_gyro_bias_reg(long *gyro_bias)
{
    unsigned char data[6] = {0, 0, 0, 0, 0, 0};
    int i=0;
    for(i=0;i<3;i++) {
    	gyro_bias[i]= (-gyro_bias[i]);
    }
    data[0] = (gyro_bias[0] >> 8) & 0xff;
    data[1] = (gyro_bias[0]) & 0xff;
    data[2] = (gyro_bias[1] >> 8) & 0xff;
    data[3] = (gyro_bias[1]) & 0xff;
    data[4] = (gyro_bias[2] >> 8) & 0xff;
    data[5] = (gyro_bias[2]) & 0xff;
    if (i2c_write(st.hw->addr, 0x13, 2, &data[0]))
        return -1;
    if (i2c_write(st.hw->addr, 0x15, 2, &data[2]))
        return -1;
    if (i2c_write(st.hw->addr, 0x17, 2, &data[4]))
        return -1;
    return 0;
}

/**
 *  @brief      push biases to the accel bias 6050 registers.
 *  this function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. bias inputs are lsb
 *  in +-16g format.
 *  @param[in]  accel_bias  new biases.
 *  @return     0 if successful.
 */
int mpu_set_accel_bias_6050_reg(const long *accel_bias) {
    unsigned char data[6] = {0, 0, 0, 0, 0, 0};
    long accel_reg_bias[3] = {0, 0, 0};

    if(mpu_read_6050_accel_bias(accel_reg_bias))
        return -1;

    accel_reg_bias[0] -= (accel_bias[0] & ~1);
    accel_reg_bias[1] -= (accel_bias[1] & ~1);
    accel_reg_bias[2] -= (accel_bias[2] & ~1);

    data[0] = (accel_reg_bias[0] >> 8) & 0xff;
    data[1] = (accel_reg_bias[0]) & 0xff;
    data[2] = (accel_reg_bias[1] >> 8) & 0xff;
    data[3] = (accel_reg_bias[1]) & 0xff;
    data[4] = (accel_reg_bias[2] >> 8) & 0xff;
    data[5] = (accel_reg_bias[2]) & 0xff;

    if (i2c_write(st.hw->addr, 0x06, 2, &data[0]))
        return -1;
    if (i2c_write(st.hw->addr, 0x08, 2, &data[2]))
        return -1;
    if (i2c_write(st.hw->addr, 0x0a, 2, &data[4]))
        return -1;

    return 0;
}



/**
 *  @brief      push biases to the accel bias 6500 registers.
 *  this function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. bias inputs are lsb
 *  in +-16g format.
 *  @param[in]  accel_bias  new biases.
 *  @return     0 if successful.
 */
int mpu_set_accel_bias_6500_reg(const long *accel_bias) {
    unsigned char data[6] = {0, 0, 0, 0, 0, 0};
    long accel_reg_bias[3] = {0, 0, 0};

    if(mpu_read_6500_accel_bias(accel_reg_bias))
        return -1;

    // preserve bit 0 of factory value (for temperature compensation)
    accel_reg_bias[0] -= (accel_bias[0] & ~1);
    accel_reg_bias[1] -= (accel_bias[1] & ~1);
    accel_reg_bias[2] -= (accel_bias[2] & ~1);

    data[0] = (accel_reg_bias[0] >> 8) & 0xff;
    data[1] = (accel_reg_bias[0]) & 0xff;
    data[2] = (accel_reg_bias[1] >> 8) & 0xff;
    data[3] = (accel_reg_bias[1]) & 0xff;
    data[4] = (accel_reg_bias[2] >> 8) & 0xff;
    data[5] = (accel_reg_bias[2]) & 0xff;

    if (i2c_write(st.hw->addr, 0x77, 2, &data[0]))
        return -1;
    if (i2c_write(st.hw->addr, 0x7a, 2, &data[2]))
        return -1;
    if (i2c_write(st.hw->addr, 0x7d, 2, &data[4]))
        return -1;

    return 0;
}


/**
 *  @brief  reset fifo read/write pointers.
 *  @return 0 if successful.
 */
int mpu_reset_fifo(void)
{
    unsigned char data;

    if (!(st.chip_cfg.sensors))
        return -1;

    data = 0;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
        return -1;

    if (st.chip_cfg.dmp_on) {
        data = bit_fifo_rst | bit_dmp_rst;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
            return -1;
        delay_ms(50);
        data = bit_dmp_en | bit_fifo_en;
        if (st.chip_cfg.sensors & inv_xyz_compass)
            data |= bit_aux_if_en;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
            return -1;
        if (st.chip_cfg.int_enable)
            data = bit_dmp_int_en;
        else
            data = 0;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
            return -1;
        data = 0;
        if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
            return -1;
    } else {
        data = bit_fifo_rst;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
            return -1;
        if (st.chip_cfg.bypass_mode || !(st.chip_cfg.sensors & inv_xyz_compass))
            data = bit_fifo_en;
        else
            data = bit_fifo_en | bit_aux_if_en;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
            return -1;
        delay_ms(50);
        if (st.chip_cfg.int_enable)
            data = bit_data_rdy_en;
        else
            data = 0;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
            return -1;
        if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &st.chip_cfg.fifo_enable))
            return -1;
    }
    return 0;
}

/**
 *  @brief      get the gyro full-scale range.
 *  @param[out] fsr current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_gyro_fsr(unsigned short *fsr)
{
    switch (st.chip_cfg.gyro_fsr) {
    case inv_fsr_250dps:
        fsr[0] = 250;
        break;
    case inv_fsr_500dps:
        fsr[0] = 500;
        break;
    case inv_fsr_1000dps:
        fsr[0] = 1000;
        break;
    case inv_fsr_2000dps:
        fsr[0] = 2000;
        break;
    default:
        fsr[0] = 0;
        break;
    }
    return 0;
}

/**
 *  @brief      set the gyro full-scale range.
 *  @param[in]  fsr desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_gyro_fsr(unsigned short fsr)
{
    unsigned char data;

    if (!(st.chip_cfg.sensors))
        return -1;

    switch (fsr) {
    case 250:
        data = inv_fsr_250dps << 3;
        break;
    case 500:
        data = inv_fsr_500dps << 3;
        break;
    case 1000:
        data = inv_fsr_1000dps << 3;
        break;
    case 2000:
        data = inv_fsr_2000dps << 3;
        break;
    default:
        return -1;
    }

    if (st.chip_cfg.gyro_fsr == (data >> 3))
        return 0;
    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &data))
        return -1;
    st.chip_cfg.gyro_fsr = data >> 3;
    return 0;
}

/**
 *  @brief      get the accel full-scale range.
 *  @param[out] fsr current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_accel_fsr(unsigned char *fsr)
{
    switch (st.chip_cfg.accel_fsr) {
    case inv_fsr_2g:
        fsr[0] = 2;
        break;
    case inv_fsr_4g:
        fsr[0] = 4;
        break;
    case inv_fsr_8g:
        fsr[0] = 8;
        break;
    case inv_fsr_16g:
        fsr[0] = 16;
        break;
    default:
        return -1;
    }
    if (st.chip_cfg.accel_half)
        fsr[0] <<= 1;
    return 0;
}

/**
 *  @brief      set the accel full-scale range.
 *  @param[in]  fsr desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_accel_fsr(unsigned char fsr)
{
    unsigned char data;

    if (!(st.chip_cfg.sensors))
        return -1;

    switch (fsr) {
    case 2:
        data = inv_fsr_2g << 3;
        break;
    case 4:
        data = inv_fsr_4g << 3;
        break;
    case 8:
        data = inv_fsr_8g << 3;
        break;
    case 16:
        data = inv_fsr_16g << 3;
        break;
    default:
        return -1;
    }

    if (st.chip_cfg.accel_fsr == (data >> 3))
        return 0;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, &data))
        return -1;
    st.chip_cfg.accel_fsr = data >> 3;
    return 0;
}

/**
 *  @brief      get the current dlpf setting.
 *  @param[out] lpf current lpf setting.
 *  0 if successful.
 */
int mpu_get_lpf(unsigned short *lpf)
{
    switch (st.chip_cfg.lpf) {
    case inv_filter_188hz:
        lpf[0] = 188;
        break;
    case inv_filter_98hz:
        lpf[0] = 98;
        break;
    case inv_filter_42hz:
        lpf[0] = 42;
        break;
    case inv_filter_20hz:
        lpf[0] = 20;
        break;
    case inv_filter_10hz:
        lpf[0] = 10;
        break;
    case inv_filter_5hz:
        lpf[0] = 5;
        break;
    case inv_filter_256hz_nolpf2:
    case inv_filter_2100hz_nolpf:
    default:
        lpf[0] = 0;
        break;
    }
    return 0;
}

/**
 *  @brief      set digital low pass filter.
 *  the following lpf settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf desired lpf setting.
 *  @return     0 if successful.
 */
int mpu_set_lpf(unsigned short lpf)
{
    unsigned char data;

    if (!(st.chip_cfg.sensors))
        return -1;

    if (lpf >= 188)
        data = inv_filter_188hz;
    else if (lpf >= 98)
        data = inv_filter_98hz;
    else if (lpf >= 42)
        data = inv_filter_42hz;
    else if (lpf >= 20)
        data = inv_filter_20hz;
    else if (lpf >= 10)
        data = inv_filter_10hz;
    else
        data = inv_filter_5hz;

    if (st.chip_cfg.lpf == data)
        return 0;
    if (i2c_write(st.hw->addr, st.reg->lpf, 1, &data))
        return -1;
    st.chip_cfg.lpf = data;
    return 0;
}

/**
 *  @brief      get sampling rate.
 *  @param[out] rate    current sampling rate (hz).
 *  @return     0 if successful.
 */
int mpu_get_sample_rate(unsigned short *rate)
{
    if (st.chip_cfg.dmp_on)
        return -1;
    else
        rate[0] = st.chip_cfg.sample_rate;
    return 0;
}

/**
 *  @brief      set sampling rate.
 *  sampling rate must be between 4hz and 1khz.
 *  @param[in]  rate    desired sampling rate (hz).
 *  @return     0 if successful.
 */
int mpu_set_sample_rate(unsigned short rate)
{
    unsigned char data;

    if (!(st.chip_cfg.sensors))
        return -1;

    if (st.chip_cfg.dmp_on)
        return -1;
    else {
        if (st.chip_cfg.lp_accel_mode) {
            if (rate && (rate <= 40)) {
                /* just stay in low-power accel mode. */
                mpu_lp_accel_mode(rate);
                return 0;
            }
            /* requested rate exceeds the allowed frequencies in lp accel mode,
             * switch back to full-power mode.
             */
            mpu_lp_accel_mode(0);
        }
        if (rate < 4)
            rate = 4;
        else if (rate > 1000)
            rate = 1000;

        data = 1000 / rate - 1;
        if (i2c_write(st.hw->addr, st.reg->rate_div, 1, &data))
            return -1;

        st.chip_cfg.sample_rate = 1000 / (1 + data);

#ifdef ak89xx_secondary
        mpu_set_compass_sample_rate(min(st.chip_cfg.compass_sample_rate, max_compass_sample_rate));
#endif

        /* automatically set lpf to 1/2 sampling rate. */
        mpu_set_lpf(st.chip_cfg.sample_rate >> 1);
        return 0;
    }
}

/**
 *  @brief      get compass sampling rate.
 *  @param[out] rate    current compass sampling rate (hz).
 *  @return     0 if successful.
 */
int mpu_get_compass_sample_rate(unsigned short *rate)
{
#ifdef ak89xx_secondary
    rate[0] = st.chip_cfg.compass_sample_rate;
    return 0;
#else
    rate[0] = 0;
    return -1;
#endif
}

/**
 *  @brief      set compass sampling rate.
 *  the compass on the auxiliary i2c bus is read by the mpu hardware at a
 *  maximum of 100hz. the actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n warning: the new rate may be different than what was requested. call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    desired compass sampling rate (hz).
 *  @return     0 if successful.
 */
int mpu_set_compass_sample_rate(unsigned short rate)
{
#ifdef ak89xx_secondary
    unsigned char div;
    if (!rate || rate > st.chip_cfg.sample_rate || rate > max_compass_sample_rate)
        return -1;

    div = st.chip_cfg.sample_rate / rate - 1;
    if (i2c_write(st.hw->addr, st.reg->s4_ctrl, 1, &div))
        return -1;
    st.chip_cfg.compass_sample_rate = st.chip_cfg.sample_rate / (div + 1);
    return 0;
#else
    return -1;
#endif
}

/**
 *  @brief      get gyro sensitivity scale factor.
 *  @param[out] sens    conversion from hardware units to dps.
 *  @return     0 if successful.
 */
int mpu_get_gyro_sens(float *sens)
{
    switch (st.chip_cfg.gyro_fsr) {
    case inv_fsr_250dps:
        sens[0] = 131.f;
        break;
    case inv_fsr_500dps:
        sens[0] = 65.5f;
        break;
    case inv_fsr_1000dps:
        sens[0] = 32.8f;
        break;
    case inv_fsr_2000dps:
        sens[0] = 16.4f;
        break;
    default:
        return -1;
    }
    return 0;
}

/**
 *  @brief      get accel sensitivity scale factor.
 *  @param[out] sens    conversion from hardware units to g's.
 *  @return     0 if successful.
 */
int mpu_get_accel_sens(unsigned short *sens)
{
    switch (st.chip_cfg.accel_fsr) {
    case inv_fsr_2g:
        sens[0] = 16384;
        break;
    case inv_fsr_4g:
        sens[0] = 8192;
        break;
    case inv_fsr_8g:
        sens[0] = 4096;
        break;
    case inv_fsr_16g:
        sens[0] = 2048;
        break;
    default:
        return -1;
    }
    if (st.chip_cfg.accel_half)
        sens[0] >>= 1;
    return 0;
}

/**
 *  @brief      get current fifo configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n inv_x_gyro, inv_y_gyro, inv_z_gyro
 *  \n inv_xyz_gyro
 *  \n inv_xyz_accel
 *  @param[out] sensors mask of sensors in fifo.
 *  @return     0 if successful.
 */
int mpu_get_fifo_config(unsigned char *sensors)
{
    sensors[0] = st.chip_cfg.fifo_enable;
    return 0;
}

/**
 *  @brief      select which sensors are pushed to fifo.
 *  @e sensors can contain a combination of the following flags:
 *  \n inv_x_gyro, inv_y_gyro, inv_z_gyro
 *  \n inv_xyz_gyro
 *  \n inv_xyz_accel
 *  @param[in]  sensors mask of sensors to push to fifo.
 *  @return     0 if successful.
 */
int mpu_configure_fifo(unsigned char sensors)
{
    unsigned char prev;
    int result = 0;

    /* compass data isn't going into the fifo. stop trying. */
    sensors &= ~inv_xyz_compass;

    if (st.chip_cfg.dmp_on)
        return 0;
    else {
        if (!(st.chip_cfg.sensors))
            return -1;
        prev = st.chip_cfg.fifo_enable;
        st.chip_cfg.fifo_enable = sensors & st.chip_cfg.sensors;
        if (st.chip_cfg.fifo_enable != sensors)
            /* you're not getting what you asked for. some sensors are
             * asleep.
             */
            result = -1;
        else
            result = 0;
        if (sensors || st.chip_cfg.lp_accel_mode)
            set_int_enable(1);
        else
            set_int_enable(0);
        if (sensors) {
            if (mpu_reset_fifo()) {
                st.chip_cfg.fifo_enable = prev;
                return -1;
            }
        }
    }

    return result;
}

/**
 *  @brief      get current power state.
 *  @param[in]  power_on    1 if turned on, 0 if suspended.
 *  @return     0 if successful.
 */
int mpu_get_power_state(unsigned char *power_on)
{
    if (st.chip_cfg.sensors)
        power_on[0] = 1;
    else
        power_on[0] = 0;
    return 0;
}

/**
 *  @brief      turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n inv_x_gyro, inv_y_gyro, inv_z_gyro
 *  \n inv_xyz_gyro
 *  \n inv_xyz_accel
 *  \n inv_xyz_compass
 *  @param[in]  sensors    mask of sensors to wake.
 *  @return     0 if successful.
 */
int mpu_set_sensors(unsigned char sensors)
{
    unsigned char data;
#ifdef ak89xx_secondary
    unsigned char user_ctrl;
#endif

    if (sensors & inv_xyz_gyro)
        data = inv_clk_pll;
    else if (sensors)
        data = 0;
    else
        data = bit_sleep;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &data)) {
        st.chip_cfg.sensors = 0;
        return -1;
    }
    st.chip_cfg.clk_src = data & ~bit_sleep;

    data = 0;
    if (!(sensors & inv_x_gyro))
        data |= bit_stby_xg;
    if (!(sensors & inv_y_gyro))
        data |= bit_stby_yg;
    if (!(sensors & inv_z_gyro))
        data |= bit_stby_zg;
    if (!(sensors & inv_xyz_accel))
        data |= bit_stby_xyza;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_2, 1, &data)) {
        st.chip_cfg.sensors = 0;
        return -1;
    }

    if (sensors && (sensors != inv_xyz_accel))
        /* latched interrupts only used in lp accel mode. */
        mpu_set_int_latched(0);

#ifdef ak89xx_secondary
#ifdef ak89xx_bypass
    if (sensors & inv_xyz_compass)
        mpu_set_bypass(1);
    else
        mpu_set_bypass(0);
#else
    if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &user_ctrl))
        return -1;
    /* handle akm power management. */
    if (sensors & inv_xyz_compass) {
        data = akm_single_measurement;
        user_ctrl |= bit_aux_if_en;
    } else {
        data = akm_power_down;
        user_ctrl &= ~bit_aux_if_en;
    }
    if (st.chip_cfg.dmp_on)
        user_ctrl |= bit_dmp_en;
    else
        user_ctrl &= ~bit_dmp_en;
    if (i2c_write(st.hw->addr, st.reg->s1_do, 1, &data))
        return -1;
    /* enable/disable i2c master mode. */
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &user_ctrl))
        return -1;
#endif
#endif

    st.chip_cfg.sensors = sensors;
    st.chip_cfg.lp_accel_mode = 0;
    delay_ms(50);
    return 0;
}

/**
 *  @brief      read the mpu interrupt status registers.
 *  @param[out] status  mask of interrupt bits.
 *  @return     0 if successful.
 */
int mpu_get_int_status(short *status)
{
    unsigned char tmp[2];
    if (!st.chip_cfg.sensors)
        return -1;
    if (i2c_read(st.hw->addr, st.reg->dmp_int_status, 2, tmp))
        return -1;
    status[0] = (tmp[0] << 8) | tmp[1];
    return 0;
}

/**
 *  @brief      get one packet from the fifo.
 *  if @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n inv_x_gyro, inv_y_gyro, inv_z_gyro
 *  \n inv_xyz_gyro
 *  \n inv_xyz_accel
 *  \n if the fifo has no new data, @e sensors will be zero.
 *  \n if the fifo is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        gyro data in hardware units.
 *  @param[out] accel       accel data in hardware units.
 *  @param[out] timestamp   timestamp in milliseconds.
 *  @param[out] sensors     mask of sensors read from fifo.
 *  @param[out] more        number of remaining packets.
 *  @return     0 if successful.
 */
int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
        unsigned char *sensors, unsigned char *more)
{
    /* assumes maximum packet size is gyro (6) + accel (6). */
    unsigned char data[max_packet_length];
    unsigned char packet_size = 0;
    unsigned short fifo_count, index = 0;

    if (st.chip_cfg.dmp_on)
        return -1;

    sensors[0] = 0;
    if (!st.chip_cfg.sensors)
        return -1;
    if (!st.chip_cfg.fifo_enable)
        return -1;

    if (st.chip_cfg.fifo_enable & inv_x_gyro)
        packet_size += 2;
    if (st.chip_cfg.fifo_enable & inv_y_gyro)
        packet_size += 2;
    if (st.chip_cfg.fifo_enable & inv_z_gyro)
        packet_size += 2;
    if (st.chip_cfg.fifo_enable & inv_xyz_accel)
        packet_size += 6;

    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
        return -1;
    fifo_count = (data[0] << 8) | data[1];
    if (fifo_count < packet_size)
        return 0;
//    log_i("fifo count: %hd\n", fifo_count);
    if (fifo_count > (st.hw->max_fifo >> 1)) {
        /* fifo is 50% full, better check overflow bit. */
        if (i2c_read(st.hw->addr, st.reg->int_status, 1, data))
            return -1;
        if (data[0] & bit_fifo_overflow) {
            mpu_reset_fifo();
            return -2;
        }
    }
    get_ms((unsigned long*)timestamp);

    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, packet_size, data))
        return -1;
    more[0] = fifo_count / packet_size - 1;
    sensors[0] = 0;

    if ((index != packet_size) && st.chip_cfg.fifo_enable & inv_xyz_accel) {
        accel[0] = (data[index+0] << 8) | data[index+1];
        accel[1] = (data[index+2] << 8) | data[index+3];
        accel[2] = (data[index+4] << 8) | data[index+5];
        sensors[0] |= inv_xyz_accel;
        index += 6;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & inv_x_gyro) {
        gyro[0] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= inv_x_gyro;
        index += 2;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & inv_y_gyro) {
        gyro[1] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= inv_y_gyro;
        index += 2;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & inv_z_gyro) {
        gyro[2] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= inv_z_gyro;
        index += 2;
    }

    return 0;
}

/**
 *  @brief      get one unparsed packet from the fifo.
 *  this function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  length of one fifo packet.
 *  @param[in]  data    fifo packet.
 *  @param[in]  more    number of remaining packets.
 */
int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
    unsigned char *more)
{
    unsigned char tmp[2];
    unsigned short fifo_count;
    if (!st.chip_cfg.dmp_on)
        return -1;
    if (!st.chip_cfg.sensors)
        return -1;

    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, tmp))
        return -1;
    fifo_count = (tmp[0] << 8) | tmp[1];
    if (fifo_count < length) {
        more[0] = 0;
        return -1;
    }
    if (fifo_count > (st.hw->max_fifo >> 1)) {
        /* fifo is 50% full, better check overflow bit. */
        if (i2c_read(st.hw->addr, st.reg->int_status, 1, tmp))
            return -1;
        if (tmp[0] & bit_fifo_overflow) {
            mpu_reset_fifo();
            return -2;
        }
    }

    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, length, data))
        return -1;
    more[0] = fifo_count / length - 1;
    return 0;
}

/**
 *  @brief      set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
int mpu_set_bypass(unsigned char bypass_on)
{
    unsigned char tmp;

    if (st.chip_cfg.bypass_mode == bypass_on)
        return 0;

    if (bypass_on) {
        if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        tmp &= ~bit_aux_if_en;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        delay_ms(3);
        tmp = bit_bypass_en;
        if (st.chip_cfg.active_low_int)
            tmp |= bit_actl;
        if (st.chip_cfg.latched_int)
            tmp |= bit_latch_en | bit_any_rd_clr;
        if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
            return -1;
    } else {
        /* enable i2c master mode if compass is being used. */
        if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        if (st.chip_cfg.sensors & inv_xyz_compass)
            tmp |= bit_aux_if_en;
        else
            tmp &= ~bit_aux_if_en;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        delay_ms(3);
        if (st.chip_cfg.active_low_int)
            tmp = bit_actl;
        else
            tmp = 0;
        if (st.chip_cfg.latched_int)
            tmp |= bit_latch_en | bit_any_rd_clr;
        if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
            return -1;
    }
    st.chip_cfg.bypass_mode = bypass_on;
    return 0;
}

/**
 *  @brief      set interrupt level.
 *  @param[in]  active_low  1 for active low, 0 for active high.
 *  @return     0 if successful.
 */
int mpu_set_int_level(unsigned char active_low)
{
    st.chip_cfg.active_low_int = active_low;
    return 0;
}

/**
 *  @brief      enable latched interrupts.
 *  any mpu register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int mpu_set_int_latched(unsigned char enable)
{
    unsigned char tmp;
    if (st.chip_cfg.latched_int == enable)
        return 0;

    if (enable)
        tmp = bit_latch_en | bit_any_rd_clr;
    else
        tmp = 0;
    if (st.chip_cfg.bypass_mode)
        tmp |= bit_bypass_en;
    if (st.chip_cfg.active_low_int)
        tmp |= bit_actl;
    if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
        return -1;
    st.chip_cfg.latched_int = enable;
    return 0;
}

#ifdef mpu6050
static int get_accel_prod_shift(float *st_shift)
{
    unsigned char tmp[4], shift_code[3], ii;

    if (i2c_read(st.hw->addr, 0x0d, 4, tmp))
        return 0x07;

    shift_code[0] = ((tmp[0] & 0xe0) >> 3) | ((tmp[3] & 0x30) >> 4);
    shift_code[1] = ((tmp[1] & 0xe0) >> 3) | ((tmp[3] & 0x0c) >> 2);
    shift_code[2] = ((tmp[2] & 0xe0) >> 3) | (tmp[3] & 0x03);
    for (ii = 0; ii < 3; ii++) {
        if (!shift_code[ii]) {
            st_shift[ii] = 0.f;
            continue;
        }
        /* equivalent to..
         * st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
         */
        st_shift[ii] = 0.34f;
        while (--shift_code[ii])
            st_shift[ii] *= 1.034f;
    }
    return 0;
}

static int accel_self_test(long *bias_regular, long *bias_st)
{
    int jj, result = 0;
    float st_shift[3], st_shift_cust, st_shift_var;

    get_accel_prod_shift(st_shift);
    for(jj = 0; jj < 3; jj++) {
        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
        if (st_shift[jj]) {
            st_shift_var = st_shift_cust / st_shift[jj] - 1.f;
            if (fabs(st_shift_var) > test.max_accel_var)
                result |= 1 << jj;
        } else if ((st_shift_cust < test.min_g) ||
            (st_shift_cust > test.max_g))
            result |= 1 << jj;
    }

    return result;
}

static int gyro_self_test(long *bias_regular, long *bias_st)
{
    int jj, result = 0;
    unsigned char tmp[3];
    float st_shift, st_shift_cust, st_shift_var;

    if (i2c_read(st.hw->addr, 0x0d, 3, tmp))
        return 0x07;

    tmp[0] &= 0x1f;
    tmp[1] &= 0x1f;
    tmp[2] &= 0x1f;

    for (jj = 0; jj < 3; jj++) {
        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
        if (tmp[jj]) {
            st_shift = 3275.f / test.gyro_sens;
            while (--tmp[jj])
                st_shift *= 1.046f;
            st_shift_var = st_shift_cust / st_shift - 1.f;
            if (fabs(st_shift_var) > test.max_gyro_var)
                result |= 1 << jj;
        } else if ((st_shift_cust < test.min_dps) ||
            (st_shift_cust > test.max_dps))
            result |= 1 << jj;
    }
    return result;
}

#endif
#ifdef ak89xx_secondary
static int compass_self_test(void)
{
    unsigned char tmp[6];
    unsigned char tries = 10;
    int result = 0x07;
    short data;

    mpu_set_bypass(1);

    tmp[0] = akm_power_down;
    if (i2c_write(st.chip_cfg.compass_addr, akm_reg_cntl, 1, tmp))
        return 0x07;
    tmp[0] = akm_bit_self_test;
    if (i2c_write(st.chip_cfg.compass_addr, akm_reg_astc, 1, tmp))
        goto akm_restore;
    tmp[0] = akm_mode_self_test;
    if (i2c_write(st.chip_cfg.compass_addr, akm_reg_cntl, 1, tmp))
        goto akm_restore;

    do {
        delay_ms(10);
        if (i2c_read(st.chip_cfg.compass_addr, akm_reg_st1, 1, tmp))
            goto akm_restore;
        if (tmp[0] & akm_data_ready)
            break;
    } while (tries--);
    if (!(tmp[0] & akm_data_ready))
        goto akm_restore;

    if (i2c_read(st.chip_cfg.compass_addr, akm_reg_hxl, 6, tmp))
        goto akm_restore;

    result = 0;
#if defined mpu9150
    data = (short)(tmp[1] << 8) | tmp[0];
    if ((data > 100) || (data < -100))
        result |= 0x01;
    data = (short)(tmp[3] << 8) | tmp[2];
    if ((data > 100) || (data < -100))
        result |= 0x02;
    data = (short)(tmp[5] << 8) | tmp[4];
    if ((data > -300) || (data < -1000))
        result |= 0x04;
#elif defined mpu9250
    data = (short)(tmp[1] << 8) | tmp[0];
    if ((data > 200) || (data < -200))
        result |= 0x01;
    data = (short)(tmp[3] << 8) | tmp[2];
    if ((data > 200) || (data < -200))
        result |= 0x02;
    data = (short)(tmp[5] << 8) | tmp[4];
    if ((data > -800) || (data < -3200))
        result |= 0x04;
#endif
akm_restore:
    tmp[0] = 0 | supports_ak89xx_high_sens;
    i2c_write(st.chip_cfg.compass_addr, akm_reg_astc, 1, tmp);
    tmp[0] = supports_ak89xx_high_sens;
    i2c_write(st.chip_cfg.compass_addr, akm_reg_cntl, 1, tmp);
    mpu_set_bypass(0);
    return result;
}
#endif

static int get_st_biases(long *gyro, long *accel, unsigned char hw_test)
{
    unsigned char data[max_packet_length];
    unsigned char packet_count, ii;
    unsigned short fifo_count;

    data[0] = 0x01;
    data[1] = 0;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
        return -1;
    delay_ms(200);
    data[0] = 0;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;
    data[0] = bit_fifo_rst | bit_dmp_rst;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;
    delay_ms(15);
    data[0] = st.test->reg_lpf;
    if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
        return -1;
    data[0] = st.test->reg_rate_div;
    if (i2c_write(st.hw->addr, st.reg->rate_div, 1, data))
        return -1;
    if (hw_test)
        data[0] = st.test->reg_gyro_fsr | 0xe0;
    else
        data[0] = st.test->reg_gyro_fsr;
    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, data))
        return -1;

    if (hw_test)
        data[0] = st.test->reg_accel_fsr | 0xe0;
    else
        data[0] = test.reg_accel_fsr;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
        return -1;
    if (hw_test)
        delay_ms(200);

    /* fill fifo for test.wait_ms milliseconds. */
    data[0] = bit_fifo_en;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;

    data[0] = inv_xyz_gyro | inv_xyz_accel;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;
    delay_ms(test.wait_ms);
    data[0] = 0;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;

    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
        return -1;

    fifo_count = (data[0] << 8) | data[1];
    packet_count = fifo_count / max_packet_length;
    gyro[0] = gyro[1] = gyro[2] = 0;
    accel[0] = accel[1] = accel[2] = 0;

    for (ii = 0; ii < packet_count; ii++) {
        short accel_cur[3], gyro_cur[3];
        if (i2c_read(st.hw->addr, st.reg->fifo_r_w, max_packet_length, data))
            return -1;
        accel_cur[0] = ((short)data[0] << 8) | data[1];
        accel_cur[1] = ((short)data[2] << 8) | data[3];
        accel_cur[2] = ((short)data[4] << 8) | data[5];
        accel[0] += (long)accel_cur[0];
        accel[1] += (long)accel_cur[1];
        accel[2] += (long)accel_cur[2];
        gyro_cur[0] = (((short)data[6] << 8) | data[7]);
        gyro_cur[1] = (((short)data[8] << 8) | data[9]);
        gyro_cur[2] = (((short)data[10] << 8) | data[11]);
        gyro[0] += (long)gyro_cur[0];
        gyro[1] += (long)gyro_cur[1];
        gyro[2] += (long)gyro_cur[2];
    }
#ifdef empl_no_64bit
    gyro[0] = (long)(((float)gyro[0]*65536.f) / test.gyro_sens / packet_count);
    gyro[1] = (long)(((float)gyro[1]*65536.f) / test.gyro_sens / packet_count);
    gyro[2] = (long)(((float)gyro[2]*65536.f) / test.gyro_sens / packet_count);
    if (has_accel) {
        accel[0] = (long)(((float)accel[0]*65536.f) / test.accel_sens /
            packet_count);
        accel[1] = (long)(((float)accel[1]*65536.f) / test.accel_sens /
            packet_count);
        accel[2] = (long)(((float)accel[2]*65536.f) / test.accel_sens /
            packet_count);
        /* don't remove gravity! */
        accel[2] -= 65536l;
    }
#else
    gyro[0] = (long)(((long long)gyro[0]<<16) / test.gyro_sens / packet_count);
    gyro[1] = (long)(((long long)gyro[1]<<16) / test.gyro_sens / packet_count);
    gyro[2] = (long)(((long long)gyro[2]<<16) / test.gyro_sens / packet_count);
    accel[0] = (long)(((long long)accel[0]<<16) / test.accel_sens /
        packet_count);
    accel[1] = (long)(((long long)accel[1]<<16) / test.accel_sens /
        packet_count);
    accel[2] = (long)(((long long)accel[2]<<16) / test.accel_sens /
        packet_count);
    /* don't remove gravity! */
    if (accel[2] > 0l)
        accel[2] -= 65536l;
    else
        accel[2] += 65536l;
#endif

    return 0;
}

#ifdef mpu6500
#define reg_6500_xg_st_data     0x0
#define reg_6500_xa_st_data     0xd
static const unsigned short mpu_6500_st_tb[256] = {
	2620,2646,2672,2699,2726,2753,2781,2808, //7
	2837,2865,2894,2923,2952,2981,3011,3041, //15
	3072,3102,3133,3165,3196,3228,3261,3293, //23
	3326,3359,3393,3427,3461,3496,3531,3566, //31
	3602,3638,3674,3711,3748,3786,3823,3862, //39
	3900,3939,3979,4019,4059,4099,4140,4182, //47
	4224,4266,4308,4352,4395,4439,4483,4528, //55
	4574,4619,4665,4712,4759,4807,4855,4903, //63
	4953,5002,5052,5103,5154,5205,5257,5310, //71
	5363,5417,5471,5525,5581,5636,5693,5750, //79
	5807,5865,5924,5983,6043,6104,6165,6226, //87
	6289,6351,6415,6479,6544,6609,6675,6742, //95
	6810,6878,6946,7016,7086,7157,7229,7301, //103
	7374,7448,7522,7597,7673,7750,7828,7906, //111
	7985,8065,8145,8227,8309,8392,8476,8561, //119
	8647,8733,8820,8909,8998,9088,9178,9270,
	9363,9457,9551,9647,9743,9841,9939,10038,
	10139,10240,10343,10446,10550,10656,10763,10870,
	10979,11089,11200,11312,11425,11539,11654,11771,
	11889,12008,12128,12249,12371,12495,12620,12746,
	12874,13002,13132,13264,13396,13530,13666,13802,
	13940,14080,14221,14363,14506,14652,14798,14946,
	15096,15247,15399,15553,15709,15866,16024,16184,
	16346,16510,16675,16842,17010,17180,17352,17526,
	17701,17878,18057,18237,18420,18604,18790,18978,
	19167,19359,19553,19748,19946,20145,20347,20550,
	20756,20963,21173,21385,21598,21814,22033,22253,
	22475,22700,22927,23156,23388,23622,23858,24097,
	24338,24581,24827,25075,25326,25579,25835,26093,
	26354,26618,26884,27153,27424,27699,27976,28255,
	28538,28823,29112,29403,29697,29994,30294,30597,
	30903,31212,31524,31839,32157,32479,32804,33132
};
static int accel_6500_self_test(long *bias_regular, long *bias_st, int debug)
{
    int i, result = 0, otp_value_zero = 0;
    float accel_st_al_min, accel_st_al_max;
    float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], accel_offset_max;
    unsigned char regs[3];
    if (i2c_read(st.hw->addr, reg_6500_xa_st_data, 3, regs)) {
    	if(debug)
    		log_i("reading otp register error.\n");
    	return 0x07;
    }
    if(debug)
    	log_i("accel otp:%d, %d, %d\n", regs[0], regs[1], regs[2]);
	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			ct_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
			ct_shift_prod[i] *= 65536.f;
			ct_shift_prod[i] /= test.accel_sens;
		}
		else {
			ct_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	if(otp_value_zero == 0) {
		if(debug)
			log_i("accel:criteria a\n");
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];
			if(debug) {
				log_i("bias_shift=%7.4f, bias_reg=%7.4f, bias_hwst=%7.4f\r\n",
						st_shift_cust[i]/1.f, bias_regular[i]/1.f,
						bias_st[i]/1.f);
				log_i("otp value: %7.4f\r\n", ct_shift_prod[i]/1.f);
			}

			st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i] - 1.f;

			if(debug)
				log_i("ratio=%7.4f, threshold=%7.4f\r\n", st_shift_ratio[i]/1.f,
							test.max_accel_var/1.f);

			if (fabs(st_shift_ratio[i]) > test.max_accel_var) {
				if(debug)
					log_i("accel fail axis = %d\n", i);
				result |= 1 << i;	//error condition
			}
		}
	}
	else {
		/* self test pass/fail criteria b */
		accel_st_al_min = test.min_g * 65536.f;
		accel_st_al_max = test.max_g * 65536.f;

		if(debug) {
			log_i("accel:criteria b\r\n");
			log_i("min mg: %7.4f\r\n", accel_st_al_min/1.f);
			log_i("max mg: %7.4f\r\n", accel_st_al_max/1.f);
		}

		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];

			if(debug)
				log_i("bias_shift=%7.4f, st=%7.4f, reg=%7.4f\n", st_shift_cust[i]/1.f, bias_st[i]/1.f, bias_regular[i]/1.f);
			if(st_shift_cust[i] < accel_st_al_min || st_shift_cust[i] > accel_st_al_max) {
				if(debug)
					log_i("accel fail axis:%d <= 225mg or >= 675mg\n", i);
				result |= 1 << i;	//error condition
			}
		}
	}

	if(result == 0) {
	/* self test pass/fail criteria c */
		accel_offset_max = test.max_g_offset * 65536.f;
		if(debug)
			log_i("accel:criteria c: bias less than %7.4f\n", accel_offset_max/1.f);
		for (i = 0; i < 3; i++) {
			if(fabs(bias_regular[i]) > accel_offset_max) {
				if(debug)
					log_i("failed: accel axis:%d = %ld > 500mg\n", i, bias_regular[i]);
				result |= 1 << i;	//error condition
			}
		}
	}

    return result;
}

static int gyro_6500_self_test(long *bias_regular, long *bias_st, int debug)
{
    int i, result = 0, otp_value_zero = 0;
    float gyro_st_al_max;
    float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], gyro_offset_max;
    unsigned char regs[3];

    if (i2c_read(st.hw->addr, reg_6500_xg_st_data, 3, regs)) {
    	if(debug)
    		log_i("reading otp register error.\n");
        return 0x07;
    }

    if(debug)
    	log_i("gyro otp:%d, %d, %d\r\n", regs[0], regs[1], regs[2]);

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			ct_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
			ct_shift_prod[i] *= 65536.f;
			ct_shift_prod[i] /= test.gyro_sens;
		}
		else {
			ct_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}

	if(otp_value_zero == 0) {
		if(debug)
			log_i("gyro:criteria a\n");
		/* self test pass/fail criteria a */
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];

			if(debug) {
				log_i("bias_shift=%7.4f, bias_reg=%7.4f, bias_hwst=%7.4f\r\n",
						st_shift_cust[i]/1.f, bias_regular[i]/1.f,
						bias_st[i]/1.f);
				log_i("otp value: %7.4f\r\n", ct_shift_prod[i]/1.f);
			}

			st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i];

			if(debug)
				log_i("ratio=%7.4f, threshold=%7.4f\r\n", st_shift_ratio[i]/1.f,
							test.max_gyro_var/1.f);

			if (fabs(st_shift_ratio[i]) < test.max_gyro_var) {
				if(debug)
					log_i("gyro fail axis = %d\n", i);
				result |= 1 << i;	//error condition
			}
		}
	}
	else {
		/* self test pass/fail criteria b */
		gyro_st_al_max = test.max_dps * 65536.f;

		if(debug) {
			log_i("gyro:criteria b\r\n");
			log_i("max dps: %7.4f\r\n", gyro_st_al_max/1.f);
		}

		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];

			if(debug)
				log_i("bias_shift=%7.4f, st=%7.4f, reg=%7.4f\n", st_shift_cust[i]/1.f, bias_st[i]/1.f, bias_regular[i]/1.f);
			if(st_shift_cust[i] < gyro_st_al_max) {
				if(debug)
					log_i("gyro fail axis:%d greater than 60dps\n", i);
				result |= 1 << i;	//error condition
			}
		}
	}

	if(result == 0) {
	/* self test pass/fail criteria c */
		gyro_offset_max = test.min_dps * 65536.f;
		if(debug)
			log_i("gyro:criteria c: bias less than %7.4f\n", gyro_offset_max/1.f);
		for (i = 0; i < 3; i++) {
			if(fabs(bias_regular[i]) > gyro_offset_max) {
				if(debug)
					log_i("failed: gyro axis:%d = %ld > 20dps\n", i, bias_regular[i]);
				result |= 1 << i;	//error condition
			}
		}
	}
    return result;
}

static int get_st_6500_biases(long *gyro, long *accel, unsigned char hw_test, int debug)
{
    unsigned char data[hwst_max_packet_length];
    unsigned char packet_count, ii;
    unsigned short fifo_count;
    int s = 0, read_size = 0, ind;

    data[0] = 0x01;
    data[1] = 0;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
        return -1;
    delay_ms(200);
    data[0] = 0;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;
    data[0] = bit_fifo_rst | bit_dmp_rst;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;
    delay_ms(15);
    data[0] = st.test->reg_lpf;
    if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
        return -1;
    data[0] = st.test->reg_rate_div;
    if (i2c_write(st.hw->addr, st.reg->rate_div, 1, data))
        return -1;
    if (hw_test)
        data[0] = st.test->reg_gyro_fsr | 0xe0;
    else
        data[0] = st.test->reg_gyro_fsr;
    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, data))
        return -1;

    if (hw_test)
        data[0] = st.test->reg_accel_fsr | 0xe0;
    else
        data[0] = test.reg_accel_fsr;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
        return -1;

    delay_ms(test.wait_ms);  //wait 200ms for sensors to stabilize

    /* enable fifo */
    data[0] = bit_fifo_en;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;
    data[0] = inv_xyz_gyro | inv_xyz_accel;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;

    //initialize the bias return values
    gyro[0] = gyro[1] = gyro[2] = 0;
    accel[0] = accel[1] = accel[2] = 0;

    if(debug)
    	log_i("starting bias loop reads\n");

    //start reading samples
    while (s < test.packet_thresh) {
    	delay_ms(test.sample_wait_ms); //wait 10ms to fill fifo
		if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
			return -1;
		fifo_count = (data[0] << 8) | data[1];
		packet_count = fifo_count / max_packet_length;
		if ((test.packet_thresh - s) < packet_count)
		            packet_count = test.packet_thresh - s;
		read_size = packet_count * max_packet_length;

		//burst read from fifo
		if (i2c_read(st.hw->addr, st.reg->fifo_r_w, read_size, data))
						return -1;
		ind = 0;
		for (ii = 0; ii < packet_count; ii++) {
			short accel_cur[3], gyro_cur[3];
			accel_cur[0] = ((short)data[ind + 0] << 8) | data[ind + 1];
			accel_cur[1] = ((short)data[ind + 2] << 8) | data[ind + 3];
			accel_cur[2] = ((short)data[ind + 4] << 8) | data[ind + 5];
			accel[0] += (long)accel_cur[0];
			accel[1] += (long)accel_cur[1];
			accel[2] += (long)accel_cur[2];
			gyro_cur[0] = (((short)data[ind + 6] << 8) | data[ind + 7]);
			gyro_cur[1] = (((short)data[ind + 8] << 8) | data[ind + 9]);
			gyro_cur[2] = (((short)data[ind + 10] << 8) | data[ind + 11]);
			gyro[0] += (long)gyro_cur[0];
			gyro[1] += (long)gyro_cur[1];
			gyro[2] += (long)gyro_cur[2];
			ind += max_packet_length;
		}
		s += packet_count;
    }

    if(debug)
    	log_i("samples: %d\n", s);

    //stop fifo
    data[0] = 0;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;

    gyro[0] = (long)(((long long)gyro[0]<<16) / test.gyro_sens / s);
    gyro[1] = (long)(((long long)gyro[1]<<16) / test.gyro_sens / s);
    gyro[2] = (long)(((long long)gyro[2]<<16) / test.gyro_sens / s);
    accel[0] = (long)(((long long)accel[0]<<16) / test.accel_sens / s);
    accel[1] = (long)(((long long)accel[1]<<16) / test.accel_sens / s);
    accel[2] = (long)(((long long)accel[2]<<16) / test.accel_sens / s);
    /* remove gravity from bias calculation */
    if (accel[2] > 0l)
        accel[2] -= 65536l;
    else
        accel[2] += 65536l;


    if(debug) {
    	log_i("accel offset data hwst bit=%d: %7.4f %7.4f %7.4f\r\n", hw_test, accel[0]/65536.f, accel[1]/65536.f, accel[2]/65536.f);
    	log_i("gyro offset data hwst bit=%d: %7.4f %7.4f %7.4f\r\n", hw_test, gyro[0]/65536.f, gyro[1]/65536.f, gyro[2]/65536.f);
    }

    return 0;
}
/**
 *  @brief      trigger gyro/accel/compass self-test for mpu6500/mpu9250
 *  on success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. for each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n the mask is defined as follows:
 *  \n bit 0:   gyro.
 *  \n bit 1:   accel.
 *  \n bit 2:   compass.
 *
 *  @param[out] gyro        gyro biases in q16 format.
 *  @param[out] accel       accel biases (if applicable) in q16 format.
 *  @param[in]  debug       debug flag used to print out more detailed logs. must first set up logging in motion driver.
 *  @return     result mask (see above).
 */
int mpu_run_6500_self_test(long *gyro, long *accel, unsigned char debug)
{
    const unsigned char tries = 2;
    long gyro_st[3], accel_st[3];
    unsigned char accel_result, gyro_result;
#ifdef ak89xx_secondary
    unsigned char compass_result;
#endif
    int ii;

    int result;
    unsigned char accel_fsr, fifo_sensors, sensors_on;
    unsigned short gyro_fsr, sample_rate, lpf;
    unsigned char dmp_was_on;



    if(debug)
    	log_i("starting mpu6500 hwst!\r\n");

    if (st.chip_cfg.dmp_on) {
        mpu_set_dmp_state(0);
        dmp_was_on = 1;
    } else
        dmp_was_on = 0;

    /* get initial settings. */
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_get_lpf(&lpf);
    mpu_get_sample_rate(&sample_rate);
    sensors_on = st.chip_cfg.sensors;
    mpu_get_fifo_config(&fifo_sensors);

    if(debug)
    	log_i("retrieving biases\r\n");

    for (ii = 0; ii < tries; ii++)
        if (!get_st_6500_biases(gyro, accel, 0, debug))
            break;
    if (ii == tries) {
        /* if we reach this point, we most likely encountered an i2c error.
         * we'll just report an error for all three sensors.
         */
        if(debug)
        	log_i("retrieving biases error - possible i2c error\n");

        result = 0;
        goto restore;
    }

    if(debug)
    	log_i("retrieving st biases\n");

    for (ii = 0; ii < tries; ii++)
        if (!get_st_6500_biases(gyro_st, accel_st, 1, debug))
            break;
    if (ii == tries) {

        if(debug)
        	log_i("retrieving st biases error - possible i2c error\n");

        /* again, probably an i2c error. */
        result = 0;
        goto restore;
    }

    accel_result = accel_6500_self_test(accel, accel_st, debug);
    if(debug)
    	log_i("accel self test results: %d\n", accel_result);

    gyro_result = gyro_6500_self_test(gyro, gyro_st, debug);
    if(debug)
    	log_i("gyro self test results: %d\n", gyro_result);

    result = 0;
    if (!gyro_result)
        result |= 0x01;
    if (!accel_result)
        result |= 0x02;

#ifdef ak89xx_secondary
    compass_result = compass_self_test();
    if(debug)
    	log_i("compass self test results: %d\n", compass_result);
    if (!compass_result)
        result |= 0x04;
#else
    result |= 0x04;
#endif
restore:
	if(debug)
		log_i("exiting hwst\n");
	/* set to invalid values to ensure no i2c writes are skipped. */
	st.chip_cfg.gyro_fsr = 0xff;
	st.chip_cfg.accel_fsr = 0xff;
	st.chip_cfg.lpf = 0xff;
	st.chip_cfg.sample_rate = 0xffff;
	st.chip_cfg.sensors = 0xff;
	st.chip_cfg.fifo_enable = 0xff;
	st.chip_cfg.clk_src = inv_clk_pll;
	mpu_set_gyro_fsr(gyro_fsr);
	mpu_set_accel_fsr(accel_fsr);
	mpu_set_lpf(lpf);
	mpu_set_sample_rate(sample_rate);
	mpu_set_sensors(sensors_on);
	mpu_configure_fifo(fifo_sensors);

	if (dmp_was_on)
		mpu_set_dmp_state(1);

	return result;
}
#endif
 /*
 *  \n this function must be called with the device either face-up or face-down
 *  (z-axis is parallel to gravity).
 *  @param[out] gyro        gyro biases in q16 format.
 *  @param[out] accel       accel biases (if applicable) in q16 format.
 *  @return     result mask (see above).
 */
int mpu_run_self_test(long *gyro, long *accel)
{
#ifdef mpu6050
    const unsigned char tries = 2;
    long gyro_st[3], accel_st[3];
    unsigned char accel_result, gyro_result;
#ifdef ak89xx_secondary
    unsigned char compass_result;
#endif
    int ii;
#endif
    int result;
    unsigned char accel_fsr, fifo_sensors, sensors_on;
    unsigned short gyro_fsr, sample_rate, lpf;
    unsigned char dmp_was_on;

    if (st.chip_cfg.dmp_on) {
        mpu_set_dmp_state(0);
        dmp_was_on = 1;
    } else
        dmp_was_on = 0;

    /* get initial settings. */
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_get_lpf(&lpf);
    mpu_get_sample_rate(&sample_rate);
    sensors_on = st.chip_cfg.sensors;
    mpu_get_fifo_config(&fifo_sensors);

    /* for older chips, the self-test will be different. */
#if defined mpu6050
    for (ii = 0; ii < tries; ii++)
        if (!get_st_biases(gyro, accel, 0))
            break;
    if (ii == tries) {
        /* if we reach this point, we most likely encountered an i2c error.
         * we'll just report an error for all three sensors.
         */
        result = 0;
        goto restore;
    }
    for (ii = 0; ii < tries; ii++)
        if (!get_st_biases(gyro_st, accel_st, 1))
            break;
    if (ii == tries) {
        /* again, probably an i2c error. */
        result = 0;
        goto restore;
    }
    accel_result = accel_self_test(accel, accel_st);
    gyro_result = gyro_self_test(gyro, gyro_st);

    result = 0;
    if (!gyro_result)
        result |= 0x01;
    if (!accel_result)
        result |= 0x02;

#ifdef ak89xx_secondary
    compass_result = compass_self_test();
    if (!compass_result)
        result |= 0x04;
#else
        result |= 0x04;
#endif
restore:
#elif defined mpu6500
    /* for now, this function will return a "pass" result for all three sensors
     * for compatibility with current test applications.
     */
    get_st_biases(gyro, accel, 0);
    result = 0x7;
#endif
    /* set to invalid values to ensure no i2c writes are skipped. */
    st.chip_cfg.gyro_fsr = 0xff;
    st.chip_cfg.accel_fsr = 0xff;
    st.chip_cfg.lpf = 0xff;
    st.chip_cfg.sample_rate = 0xffff;
    st.chip_cfg.sensors = 0xff;
    st.chip_cfg.fifo_enable = 0xff;
    st.chip_cfg.clk_src = inv_clk_pll;
    mpu_set_gyro_fsr(gyro_fsr);
    mpu_set_accel_fsr(accel_fsr);
    mpu_set_lpf(lpf);
    mpu_set_sample_rate(sample_rate);
    mpu_set_sensors(sensors_on);
    mpu_configure_fifo(fifo_sensors);

    if (dmp_was_on)
        mpu_set_dmp_state(1);

    return result;
}

/**
 *  @brief      write to the dmp memory.
 *  this function prevents i2c writes past the bank boundaries. the dmp memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    memory location (bank << 8 | start address)
 *  @param[in]  length      number of bytes to write.
 *  @param[in]  data        bytes to write to memory.
 *  @return     0 if successful.
 */
int mpu_write_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data)
{
    unsigned char tmp[2];

    if (!data)
        return -1;
    if (!st.chip_cfg.sensors)
        return -1;

    tmp[0] = (unsigned char)(mem_addr >> 8);
    tmp[1] = (unsigned char)(mem_addr & 0xff);

    /* check bank boundaries. */
    if (tmp[1] + length > st.hw->bank_size)
        return -1;

    if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->mem_r_w, length, data))
        return -1;
    return 0;
}

/**
 *  @brief      read from the dmp memory.
 *  this function prevents i2c reads past the bank boundaries. the dmp memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    memory location (bank << 8 | start address)
 *  @param[in]  length      number of bytes to read.
 *  @param[out] data        bytes read from memory.
 *  @return     0 if successful.
 */
int mpu_read_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data)
{
    unsigned char tmp[2];

    if (!data)
        return -1;
    if (!st.chip_cfg.sensors)
        return -1;

    tmp[0] = (unsigned char)(mem_addr >> 8);
    tmp[1] = (unsigned char)(mem_addr & 0xff);

    /* check bank boundaries. */
    if (tmp[1] + length > st.hw->bank_size)
        return -1;

    if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
        return -1;
    if (i2c_read(st.hw->addr, st.reg->mem_r_w, length, data))
        return -1;
    return 0;
}

/**
 *  @brief      load and verify dmp image.
 *  @param[in]  length      length of dmp image.
 *  @param[in]  firmware    dmp code.
 *  @param[in]  start_addr  starting address of dmp code memory.
 *  @param[in]  sample_rate fixed sampling rate used when dmp is enabled.
 *  @return     0 if successful.
 */
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate)
{
    unsigned short ii;
    unsigned short this_write;
    /* must divide evenly into st.hw->bank_size to avoid bank crossings. */
#define load_chunk  (16)
    unsigned char cur[load_chunk], tmp[2];

    if (st.chip_cfg.dmp_loaded)
        /* dmp should only be loaded once. */
        return -1;

    if (!firmware)
        return -1;
    for (ii = 0; ii < length; ii += this_write) {
        this_write = min(load_chunk, length - ii);
        if (mpu_write_mem(ii, this_write, (unsigned char*)&firmware[ii]))
            return -1;
        if (mpu_read_mem(ii, this_write, cur))
            return -1;
        if (memcmp(firmware+ii, cur, this_write))
            return -2;
    }

    /* set program start address. */
    tmp[0] = start_addr >> 8;
    tmp[1] = start_addr & 0xff;
    if (i2c_write(st.hw->addr, st.reg->prgm_start_h, 2, tmp))
        return -1;

    st.chip_cfg.dmp_loaded = 1;
    st.chip_cfg.dmp_sample_rate = sample_rate;
    return 0;
}

/**
 *  @brief      enable/disable dmp support.
 *  @param[in]  enable  1 to turn on the dmp.
 *  @return     0 if successful.
 */
int mpu_set_dmp_state(unsigned char enable)
{
    unsigned char tmp;
    if (st.chip_cfg.dmp_on == enable)
        return 0;

    if (enable) {
        if (!st.chip_cfg.dmp_loaded)
            return -1;
        /* disable data ready interrupt. */
        set_int_enable(0);
        /* disable bypass mode. */
        mpu_set_bypass(0);
        /* keep constant sample rate, fifo rate controlled by dmp. */
        mpu_set_sample_rate(st.chip_cfg.dmp_sample_rate);
        /* remove fifo elements. */
        tmp = 0;
        i2c_write(st.hw->addr, 0x23, 1, &tmp);
        st.chip_cfg.dmp_on = 1;
        /* enable dmp interrupt. */
        set_int_enable(1);
        mpu_reset_fifo();
    } else {
        /* disable dmp interrupt. */
        set_int_enable(0);
        /* restore fifo settings. */
        tmp = st.chip_cfg.fifo_enable;
        i2c_write(st.hw->addr, 0x23, 1, &tmp);
        st.chip_cfg.dmp_on = 0;
        mpu_reset_fifo();
    }
    return 0;
}

/**
 *  @brief      get dmp state.
 *  @param[out] enabled 1 if enabled.
 *  @return     0 if successful.
 */
int mpu_get_dmp_state(unsigned char *enabled)
{
    enabled[0] = st.chip_cfg.dmp_on;
    return 0;
}

#ifdef ak89xx_secondary
/* this initialization is similar to the one in ak8975.c. */
static int setup_compass(void)
{
    unsigned char data[4], akm_addr;

    mpu_set_bypass(1);

    /* find compass. possible addresses range from 0x0c to 0x0f. */
    for (akm_addr = 0x0c; akm_addr <= 0x0f; akm_addr++) {
        int result;
        result = i2c_read(akm_addr, akm_reg_whoami, 1, data);
        if (!result && (data[0] == akm_whoami))
            break;
    }

    if (akm_addr > 0x0f) {
        /* todo: handle this case in all compass-related functions. */
        log_e("compass not found.\n");
        return -1;
    }

    st.chip_cfg.compass_addr = akm_addr;

    data[0] = akm_power_down;
    if (i2c_write(st.chip_cfg.compass_addr, akm_reg_cntl, 1, data))
        return -1;
    delay_ms(1);

    data[0] = akm_fuse_rom_access;
    if (i2c_write(st.chip_cfg.compass_addr, akm_reg_cntl, 1, data))
        return -1;
    delay_ms(1);

    /* get sensitivity adjustment data from fuse rom. */
    if (i2c_read(st.chip_cfg.compass_addr, akm_reg_asax, 3, data))
        return -1;
    st.chip_cfg.mag_sens_adj[0] = (long)data[0] + 128;
    st.chip_cfg.mag_sens_adj[1] = (long)data[1] + 128;
    st.chip_cfg.mag_sens_adj[2] = (long)data[2] + 128;

    data[0] = akm_power_down;
    if (i2c_write(st.chip_cfg.compass_addr, akm_reg_cntl, 1, data))
        return -1;
    delay_ms(1);

    mpu_set_bypass(0);

    /* set up master mode, master clock, and es bit. */
    data[0] = 0x40;
    if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
        return -1;

    /* slave 0 reads from akm data registers. */
    data[0] = bit_i2c_read | st.chip_cfg.compass_addr;
    if (i2c_write(st.hw->addr, st.reg->s0_addr, 1, data))
        return -1;

    /* compass reads start at this register. */
    data[0] = akm_reg_st1;
    if (i2c_write(st.hw->addr, st.reg->s0_reg, 1, data))
        return -1;

    /* enable slave 0, 8-byte reads. */
    data[0] = bit_slave_en | 8;
    if (i2c_write(st.hw->addr, st.reg->s0_ctrl, 1, data))
        return -1;

    /* slave 1 changes akm measurement mode. */
    data[0] = st.chip_cfg.compass_addr;
    if (i2c_write(st.hw->addr, st.reg->s1_addr, 1, data))
        return -1;

    /* akm measurement mode register. */
    data[0] = akm_reg_cntl;
    if (i2c_write(st.hw->addr, st.reg->s1_reg, 1, data))
        return -1;

    /* enable slave 1, 1-byte writes. */
    data[0] = bit_slave_en | 1;
    if (i2c_write(st.hw->addr, st.reg->s1_ctrl, 1, data))
        return -1;

    /* set slave 1 data. */
    data[0] = akm_single_measurement;
    if (i2c_write(st.hw->addr, st.reg->s1_do, 1, data))
        return -1;

    /* trigger slave 0 and slave 1 actions at each sample. */
    data[0] = 0x03;
    if (i2c_write(st.hw->addr, st.reg->i2c_delay_ctrl, 1, data))
        return -1;

#ifdef mpu9150
    /* for the mpu9150, the auxiliary i2c bus needs to be set to vdd. */
    data[0] = bit_i2c_mst_vddio;
    if (i2c_write(st.hw->addr, st.reg->yg_offs_tc, 1, data))
        return -1;
#endif

    return 0;
}
#endif

/**
 *  @brief      read raw compass data.
 *  @param[out] data        raw data in hardware units.
 *  @param[out] timestamp   timestamp in milliseconds. null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_compass_reg(short *data, unsigned long *timestamp)
{
#ifdef ak89xx_secondary
    unsigned char tmp[9];

    if (!(st.chip_cfg.sensors & inv_xyz_compass))
        return -1;

#ifdef ak89xx_bypass
    if (i2c_read(st.chip_cfg.compass_addr, akm_reg_st1, 8, tmp))
        return -1;
    tmp[8] = akm_single_measurement;
    if (i2c_write(st.chip_cfg.compass_addr, akm_reg_cntl, 1, tmp+8))
        return -1;
#else
    if (i2c_read(st.hw->addr, st.reg->raw_compass, 8, tmp))
        return -1;
#endif

#if defined ak8975_secondary
    /* ak8975 doesn't have the overrun error bit. */
    if (!(tmp[0] & akm_data_ready))
        return -2;
    if ((tmp[7] & akm_overflow) || (tmp[7] & akm_data_error))
        return -3;
#elif defined ak8963_secondary
    /* ak8963 doesn't have the data read error bit. */
    if (!(tmp[0] & akm_data_ready) || (tmp[0] & akm_data_overrun))
        return -2;
    if (tmp[7] & akm_overflow)
        return -3;
#endif
    data[0] = (tmp[2] << 8) | tmp[1];
    data[1] = (tmp[4] << 8) | tmp[3];
    data[2] = (tmp[6] << 8) | tmp[5];

    data[0] = ((long)data[0] * st.chip_cfg.mag_sens_adj[0]) >> 8;
    data[1] = ((long)data[1] * st.chip_cfg.mag_sens_adj[1]) >> 8;
    data[2] = ((long)data[2] * st.chip_cfg.mag_sens_adj[2]) >> 8;

    if (timestamp)
        get_ms(timestamp);
    return 0;
#else
    return -1;
#endif
}

/**
 *  @brief      get the compass full-scale range.
 *  @param[out] fsr current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_compass_fsr(unsigned short *fsr)
{
#ifdef ak89xx_secondary
    fsr[0] = st.hw->compass_fsr;
    return 0;
#else
    return -1;
#endif
}

/**
 *  @brief      enters lp accel motion interrupt mode.
 *  the behaviour of this feature is very different between the mpu6050 and the
 *  mpu6500. each chip's version of this feature is explained below.
 *
 *  \n the hardware motion threshold can be between 32mg and 8160mg in 32mg
 *  increments.
 *
 *  \n low-power accel mode supports the following frequencies:
 *  \n 1.25hz, 5hz, 20hz, 40hz
 *
 *  \n mpu6500:
 *  \n unlike the mpu6050 version, the hardware does not "lock in" a reference
 *  sample. the hardware monitors the accel data and detects any large change
 *  over a short period of time.
 *
 *  \n the hardware motion threshold can be between 4mg and 1020mg in 4mg
 *  increments.
 *
 *  \n mpu6500 low-power accel mode supports the following frequencies:
 *  \n 1.25hz, 2.5hz, 5hz, 10hz, 20hz, 40hz, 80hz, 160hz, 320hz, 640hz
 *
 *  \n\n notes:
 *  \n the driver will round down @e thresh to the nearest supported value if
 *  an unsupported threshold is selected.
 *  \n to select a fractional wake-up frequency, round down the value passed to
 *  @e lpa_freq.
 *  \n the mpu6500 does not support a delay parameter. if this function is used
 *  for the mpu6500, the value passed to @e time will be ignored.
 *  \n to disable this mode, set @e lpa_freq to zero. the driver will restore
 *  the previous configuration.
 *
 *  @param[in]  thresh      motion threshold in mg.
 *  @param[in]  time        duration in milliseconds that the accel data must
 *                          exceed @e thresh before motion is reported.
 *  @param[in]  lpa_freq    minimum sampling rate, or zero to disable.
 *  @return     0 if successful.
 */
int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,
    unsigned short lpa_freq)
{

#if defined mpu6500
    unsigned char data[3];
#endif
    if (lpa_freq) {
#if defined mpu6500
    	unsigned char thresh_hw;

        /* 1lsb = 4mg. */
        if (thresh > 1020)
            thresh_hw = 255;
        else if (thresh < 4)
            thresh_hw = 1;
        else
            thresh_hw = thresh >> 2;
#endif

        if (!time)
            /* minimum duration must be 1ms. */
            time = 1;

#if defined mpu6500
        if (lpa_freq > 640)
            /* at this point, the chip has not been re-configured, so the
             * function can safely exit.
             */
            return -1;
#endif

        if (!st.chip_cfg.int_motion_only) {
            /* store current settings for later. */
            if (st.chip_cfg.dmp_on) {
                mpu_set_dmp_state(0);
                st.chip_cfg.cache.dmp_on = 1;
            } else
                st.chip_cfg.cache.dmp_on = 0;
            mpu_get_gyro_fsr(&st.chip_cfg.cache.gyro_fsr);
            mpu_get_accel_fsr(&st.chip_cfg.cache.accel_fsr);
            mpu_get_lpf(&st.chip_cfg.cache.lpf);
            mpu_get_sample_rate(&st.chip_cfg.cache.sample_rate);
            st.chip_cfg.cache.sensors_on = st.chip_cfg.sensors;
            mpu_get_fifo_config(&st.chip_cfg.cache.fifo_sensors);
        }

#if defined mpu6500
        /* disable hardware interrupts. */
        set_int_enable(0);

        /* enter full-power accel-only mode, no fifo/dmp. */
        data[0] = 0;
        data[1] = 0;
        data[2] = bit_stby_xyzg;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 3, data))
            goto lp_int_restore;

        /* set motion threshold. */
        data[0] = thresh_hw;
        if (i2c_write(st.hw->addr, st.reg->motion_thr, 1, data))
            goto lp_int_restore;

        /* set wake frequency. */
        if (lpa_freq == 1)
            data[0] = inv_lpa_1_25hz;
        else if (lpa_freq == 2)
            data[0] = inv_lpa_2_5hz;
        else if (lpa_freq <= 5)
            data[0] = inv_lpa_5hz;
        else if (lpa_freq <= 10)
            data[0] = inv_lpa_10hz;
        else if (lpa_freq <= 20)
            data[0] = inv_lpa_20hz;
        else if (lpa_freq <= 40)
            data[0] = inv_lpa_40hz;
        else if (lpa_freq <= 80)
            data[0] = inv_lpa_80hz;
        else if (lpa_freq <= 160)
            data[0] = inv_lpa_160hz;
        else if (lpa_freq <= 320)
            data[0] = inv_lpa_320hz;
        else
            data[0] = inv_lpa_640hz;
        if (i2c_write(st.hw->addr, st.reg->lp_accel_odr, 1, data))
            goto lp_int_restore;

        /* enable motion interrupt (mpu6500 version). */
        data[0] = bits_wom_en;
        if (i2c_write(st.hw->addr, st.reg->accel_intel, 1, data))
            goto lp_int_restore;

        /* enable cycle mode. */
        data[0] = bit_lpa_cycle;
        if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
            goto lp_int_restore;

        /* enable interrupt. */
        data[0] = bit_mot_int_en;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
            goto lp_int_restore;

        st.chip_cfg.int_motion_only = 1;
        return 0;
#endif
    } else {
        /* don't "restore" the previous state if no state has been saved. */
        unsigned int ii;
        char *cache_ptr = (char*)&st.chip_cfg.cache;
        for (ii = 0; ii < sizeof(st.chip_cfg.cache); ii++) {
            if (cache_ptr[ii] != 0)
                goto lp_int_restore;
        }
        /* if we reach this point, motion interrupt mode hasn't been used yet. */
        return -1;
    }
lp_int_restore:
    /* set to invalid values to ensure no i2c writes are skipped. */
    st.chip_cfg.gyro_fsr = 0xff;
    st.chip_cfg.accel_fsr = 0xff;
    st.chip_cfg.lpf = 0xff;
    st.chip_cfg.sample_rate = 0xffff;
    st.chip_cfg.sensors = 0xff;
    st.chip_cfg.fifo_enable = 0xff;
    st.chip_cfg.clk_src = inv_clk_pll;
    mpu_set_sensors(st.chip_cfg.cache.sensors_on);
    mpu_set_gyro_fsr(st.chip_cfg.cache.gyro_fsr);
    mpu_set_accel_fsr(st.chip_cfg.cache.accel_fsr);
    mpu_set_lpf(st.chip_cfg.cache.lpf);
    mpu_set_sample_rate(st.chip_cfg.cache.sample_rate);
    mpu_configure_fifo(st.chip_cfg.cache.fifo_sensors);

    if (st.chip_cfg.cache.dmp_on)
        mpu_set_dmp_state(1);

#ifdef mpu6500
    /* disable motion interrupt (mpu6500 version). */
    data[0] = 0;
    if (i2c_write(st.hw->addr, st.reg->accel_intel, 1, data))
        goto lp_int_restore;
#endif

    st.chip_cfg.int_motion_only = 0;
    return 0;
}

/**
 *  @}
 */

