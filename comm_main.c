/* Standard includes. */
#include <stdio.h>
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "mmio.h"
#include "delay.h"

/* cvitek includes. */
#include "printf.h"
#include "rtos_cmdqu.h"
#include "cvi_mailbox.h"
#include "intr_conf.h"
#include "top_reg.h"
#include "memmap.h"
#include "comm.h"
#include "cvi_spinlock.h"
#include "hal_pwm.h"

/*
 * hal_pwm.h defines PWM_CHANNEL_0..3 as macros, while driver pwm.h defines
 * PWM_CHANNEL_0.. as enum identifiers. Undefine the hal macros before
 * including pwm.h to avoid macro substitution breaking the enum.
 */
#ifdef PWM_CHANNEL_0
#undef PWM_CHANNEL_0
#endif
#ifdef PWM_CHANNEL_1
#undef PWM_CHANNEL_1
#endif
#ifdef PWM_CHANNEL_2
#undef PWM_CHANNEL_2
#endif
#ifdef PWM_CHANNEL_3
#undef PWM_CHANNEL_3
#endif

#include "pwm.h"

/* Milk-V Duo */
#include "milkv_duo_io.h"
#include "arch_helpers.h"


#define AMP_ENABLE_IBUS_PROBE 1

#ifndef UART3_BASE
#define UART3_BASE 0x04170000UL
#endif

#define UART_RBR   0x00
#define UART_LSR   0x14
#define UART_LSR_DR (1u << 0)

typedef enum {
	UART0 = 0,
	UART1,
	UART2,
	UART3,
} device_uart;

void hal_uart_init(device_uart dev_uart, int baudrate, int uart_clock);


#define IBUS_FRAME_SIZE 32
#define IBUS_CMD 0x40
#define IBUS_CHANNELS 14


// #define __DEBUG__
#ifdef __DEBUG__
#define debug_printf printf
#else
#define debug_printf(...)
#endif

typedef struct _TASK_CTX_S {
	char        name[32];
	u16         stack_size;
	UBaseType_t priority;
	void (*runTask)(void *pvParameters);
	u8            queLength;
	QueueHandle_t queHandle;
} TASK_CTX_S;

/****************************************************************************
 * Function prototypes
 ****************************************************************************/
int prvQueueISR(int irq, void *dev_id);
void prvCmdQuRunTask(void *pvParameters);
static void mailbox_hw_init(void);

#if AMP_ENABLE_IBUS_PROBE
static void ibus_probe_poll_once(void);
#endif


/****************************************************************************
 * Global parameters
 ****************************************************************************/
TASK_CTX_S gTaskCtx[1] = {
	{
		.name = "CMDQU",
		.stack_size = configMINIMAL_STACK_SIZE*4,
		.priority = tskIDLE_PRIORITY + 5,
		.runTask = prvCmdQuRunTask,
		.queLength = 30,
		.queHandle = NULL,
	},
};

/* mailbox parameters */
volatile struct mailbox_set_register *mbox_reg;
volatile struct mailbox_done_register *mbox_done_reg;
volatile unsigned long *mailbox_context; // mailbox buffer context is 64 Bytess

#define AMP_CMD_REGION_PHYS 0x83F00000UL
#define AMP_SHARED_REGION_MAGIC 0x4643414DU
#define AMP_SHARED_REGION_VERSION 1U
#define AMP_CMD_INIT_SHM 0x20

static volatile unsigned int g_amp_shm_phys = 0;
static volatile unsigned int g_amp_shm_ready = 0;

#define AMP_DBG_MAGIC   0x414D5044U  /* "AMPD" */
#define AMP_DBG_VERSION 1U
#define AMP_DBG_SIZE    4096U
/*
 * CVIMMAP_FREERTOS_ADDR/SIZE are visible in cvirtos.map but not exported
 * as C macros in this build. The map confirms:
 *   CVIMMAP_FREERTOS_ADDR = 0x8fe00000
 *   CVIMMAP_FREERTOS_SIZE = 0x00200000
 * Therefore the last 4KB of RTOS reserved memory is 0x8FFFF000.
 */
#define AMP_DBG_SHM_ADDR 0x8FFFF000UL

typedef struct {
	volatile uint32_t magic;
	volatile uint32_t version;
	volatile uint32_t seq;
	volatile uint32_t tick;
	volatile uint32_t alive;

	volatile uint32_t mailbox_isr_count;
	volatile uint32_t mailbox_task_count;
	volatile uint32_t last_cmd_id;
	volatile uint32_t last_param_ptr;

	volatile uint32_t ibus_rx_bytes;
	volatile uint32_t ibus_good_frames;
	volatile uint32_t ibus_bad_frames;
	volatile uint32_t ibus_header20;
	volatile uint32_t ibus_header2040;
	volatile uint32_t ibus_failsafe;
	volatile uint32_t uart3_lsr;
	volatile uint32_t uart3_last_byte;

	volatile uint32_t shm_phys;
	volatile uint32_t shm_ready;

	volatile uint32_t i2c_probe_count;
	volatile uint32_t i2c_last_err;

	volatile uint32_t hmc_addr;
	volatile uint32_t hmc_id_a;
	volatile uint32_t hmc_id_b;
	volatile uint32_t hmc_id_c;
	volatile uint32_t hmc_ok;
	volatile uint32_t hmc_bus;
	volatile uint32_t hmc_scan_mask;
	volatile uint32_t hmc_data_ok;
	volatile int32_t hmc_x;
	volatile int32_t hmc_y;
	volatile int32_t hmc_z;

	volatile uint32_t mpu_addr;
	volatile uint32_t mpu_whoami;
	volatile uint32_t mpu_ok;
	volatile uint32_t mpu_data_ok;
	volatile int32_t mpu_ax;
	volatile int32_t mpu_ay;
	volatile int32_t mpu_az;
	volatile int32_t mpu_gx;
	volatile int32_t mpu_gy;
	volatile int32_t mpu_gz;

		volatile uint32_t mpu_cal_count;
	volatile uint32_t mpu_cal_done;

	volatile int32_t mpu_acc_bias_x;
	volatile int32_t mpu_acc_bias_y;
	volatile int32_t mpu_acc_bias_z;

	volatile int32_t mpu_gyro_bias_x;
	volatile int32_t mpu_gyro_bias_y;
	volatile int32_t mpu_gyro_bias_z;

	volatile uint32_t sensor_ready;

	volatile int32_t mpu_ax_corr;
	volatile int32_t mpu_ay_corr;
	volatile int32_t mpu_az_corr;

	volatile int32_t mpu_gx_corr;
	volatile int32_t mpu_gy_corr;
	volatile int32_t mpu_gz_corr;

	volatile uint32_t att_ok;
	volatile int32_t att_roll_cd;
	volatile int32_t att_pitch_cd;
	volatile int32_t att_yaw_cd;

	volatile int32_t rate_roll_raw;
	volatile int32_t rate_pitch_raw;
	volatile int32_t rate_yaw_raw;

	volatile uint32_t level_trim_done;
	volatile int32_t level_roll_cd;
	volatile int32_t level_pitch_cd;

	volatile int32_t body_roll_cd;
	volatile int32_t body_pitch_cd;
	volatile int32_t body_yaw_cd;

	volatile int32_t body_rate_roll_raw;
	volatile int32_t body_rate_pitch_raw;
	volatile int32_t body_rate_yaw_raw;

	volatile int32_t sp_roll_rate_raw;
	volatile int32_t sp_pitch_rate_raw;

	volatile int32_t sp_roll_cd;
	volatile int32_t sp_pitch_cd;
	volatile int32_t sp_yaw_rate_raw;
	volatile uint32_t sp_throttle_us;

	volatile int32_t pid_roll;
	volatile int32_t pid_pitch;
	volatile int32_t pid_yaw;

	volatile int32_t mix_m0;
	volatile int32_t mix_m1;
	volatile int32_t mix_m2;
	volatile int32_t mix_m3;

	volatile uint32_t motor_armed;
	volatile uint32_t motor_output_enabled;
	volatile uint32_t motor_safety_reason;
	volatile uint32_t motor_pwm0;
	volatile uint32_t motor_pwm1;
	volatile uint32_t motor_pwm2;
	volatile uint32_t motor_pwm3;

	volatile int32_t motor_trim0;
	volatile int32_t motor_trim1;
	volatile int32_t motor_trim2;
	volatile int32_t motor_trim3;

	volatile uint32_t pwm_backend_ready;
	volatile uint32_t pwm_backend_err;
	volatile uint32_t pwm_real_enabled;

	volatile uint32_t pwm_period_us_dbg;
	volatile uint32_t pwm_active_high_dbg;
	volatile uint32_t pwm_duty0_us_dbg;
	volatile uint32_t pwm_duty1_us_dbg;
	volatile uint32_t pwm_duty2_us_dbg;
	volatile uint32_t pwm_duty3_us_dbg;
	volatile uint32_t pwm_period0_clk_dbg;
	volatile uint32_t pwm_period1_clk_dbg;
	volatile uint32_t pwm_period2_clk_dbg;
	volatile uint32_t pwm_period3_clk_dbg;
	volatile uint32_t pwm_duty0_clk_dbg;
	volatile uint32_t pwm_duty1_clk_dbg;
	volatile uint32_t pwm_duty2_clk_dbg;
	volatile uint32_t pwm_duty3_clk_dbg;
	volatile uint32_t pwm_start0_dbg;
	volatile uint32_t pwm_start1_dbg;
	volatile uint32_t pwm_start2_dbg;
	volatile uint32_t pwm_start3_dbg;
	volatile uint32_t pwm_oe0_dbg;
	volatile uint32_t pwm_oe1_dbg;
	volatile uint32_t pwm_oe2_dbg;
	volatile uint32_t pwm_oe3_dbg;

	volatile uint32_t esc_arm_delay_done;
	volatile uint32_t esc_arm_delay_ms;
	volatile uint32_t esc_test_max_us;

	volatile uint32_t rc_valid;
	volatile uint32_t rc_throttle_us;
	volatile uint32_t rc_arm_switch;
	volatile uint32_t rc_arm_allowed;
	volatile uint32_t rc_frame_age_ms;
	volatile uint32_t rc_seen_arm_low;
	volatile uint32_t rc_arm_latched;

	volatile uint32_t motor_min_spin0;
	volatile uint32_t motor_min_spin1;
	volatile uint32_t motor_min_spin2;
	volatile uint32_t motor_min_spin3;

	volatile int32_t angle_corr_roll;
	volatile int32_t angle_corr_pitch;
	volatile int32_t rate_corr_roll;
	volatile int32_t rate_corr_pitch;
	volatile int32_t rate_corr_yaw;
	volatile uint32_t sensor_state_error;

	volatile uint32_t control_stage;
	volatile uint32_t control_min_active_throttle_us;
	volatile uint32_t angle_assist_enable_dbg;
	volatile uint32_t rate_pid_max_corr_us_dbg;
	volatile uint32_t angle_assist_max_corr_us_dbg;

	volatile uint32_t amp_shm_poll_count;
	volatile uint32_t amp_emergency_latched;
	volatile uint32_t amp_config_seq;
	volatile uint32_t amp_config_update_count;
	volatile uint32_t amp_rt_report_seq;
	volatile uint32_t amp_status_flags;
	volatile uint32_t amp_config_deadzone;
	volatile uint32_t amp_config_rate_div;
	volatile uint32_t amp_config_angle_div;

	volatile uint32_t stage3_step_count;
	volatile uint32_t stage3_actuator_map_ok;
	volatile uint32_t stage3_external_enabled;

} amp_dbg_shm_t;

static volatile amp_dbg_shm_t *g_amp_dbg = (volatile amp_dbg_shm_t *)AMP_DBG_SHM_ADDR;

static volatile uint32_t g_mailbox_isr_count = 0;
static volatile uint32_t g_mailbox_task_count = 0;
static volatile uint32_t g_last_cmd_id = 0;
static volatile uint32_t g_last_param_ptr = 0;

static volatile uint32_t g_uart3_last_lsr = 0;
static volatile uint32_t g_uart3_last_byte = 0;




#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(ms) ((TickType_t)(((ms) * configTICK_RATE_HZ) / 1000U))
#endif

typedef struct __attribute__((packed)) {
	float throttle;
	float roll_deg;
	float pitch_deg;
	float yaw_rate_dps;
	uint32_t seq;
} amp_command_setpoint_t;

typedef struct __attribute__((packed)) {
	uint8_t state;
	uint8_t reserved0;
	uint8_t reserved1;
	uint8_t reserved2;
} amp_command_arm_t;

typedef struct __attribute__((packed)) {
	uint64_t monotonic_us;
	uint32_t seq;
} amp_heartbeat_t;

typedef struct __attribute__((packed)) {
	uint32_t reason;
} amp_command_emergency_stop_t;

typedef struct __attribute__((packed)) {
	float angle_p;
	float rate_p;
	float rate_i;
	float rate_d;
	float throttle_expo;
	uint16_t deadzone_us;
	uint16_t reserved0;
	uint32_t update_seq;
} amp_flight_config_t;

typedef struct __attribute__((packed)) {
	uint64_t monotonic_us;
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	float roll_rate_dps;
	float pitch_rate_dps;
	float yaw_rate_dps;
	uint16_t motor_pwm_us[4];
	uint32_t status_flags;
	uint32_t seq;
} amp_rt_report_t;

#define AMP_SHARED_SLOT(type_name) \
	struct __attribute__((packed)) { \
		uint32_t seq; \
		type_name payload; \
		uint32_t payload_crc; \
	}

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint32_t version;
	AMP_SHARED_SLOT(amp_command_setpoint_t) setpoint;
	AMP_SHARED_SLOT(amp_command_arm_t) arm;
	AMP_SHARED_SLOT(amp_heartbeat_t) heartbeat;
	AMP_SHARED_SLOT(amp_command_emergency_stop_t) emergency_stop;
	AMP_SHARED_SLOT(amp_flight_config_t) config;
	AMP_SHARED_SLOT(amp_rt_report_t) rt_report;
} amp_shared_command_region_t;


static void amp_dbg_flush(void)
{
	flush_dcache_range((uintptr_t)g_amp_dbg, sizeof(*g_amp_dbg));
}

/****************************************************************************
 * Function definitions
 ****************************************************************************/
DEFINE_CVI_SPINLOCK(mailbox_lock, SPIN_MBOX);

static void mailbox_hw_init(void)
{
	unsigned int reg_base = MAILBOX_REG_BASE;

	mbox_reg = (struct mailbox_set_register *)reg_base;
	mbox_done_reg = (struct mailbox_done_register *)(reg_base + 2);
	mailbox_context = (unsigned long *)(MAILBOX_REG_BUFF);

	cvi_spinlock_init();
}


#if AMP_ENABLE_IBUS_PROBE

static int uart3_read_byte(uint8_t *out)
{
	volatile uint32_t *lsr = (volatile uint32_t *)(UART3_BASE + UART_LSR);
	volatile uint32_t *rbr = (volatile uint32_t *)(UART3_BASE + UART_RBR);
	uint32_t v_lsr;

	v_lsr = *lsr;
	g_uart3_last_lsr = v_lsr;

	if ((v_lsr & UART_LSR_DR) == 0) {
		return 0;
	}

	g_uart3_last_byte = (*rbr & 0xffU);
	*out = (uint8_t)g_uart3_last_byte;
	return 1;
}

static int ibus_parse_byte(uint8_t b, uint16_t ch[IBUS_CHANNELS])
{
	static uint8_t frame[IBUS_FRAME_SIZE];
	static unsigned int index = 0;

	unsigned int i;
	uint16_t checksum;
	uint16_t expected;

	if (index == 0 && b != 0x20) {
		return 0;
	}

	frame[index++] = b;

	if (index < IBUS_FRAME_SIZE) {
		return 0;
	}

	index = 0;

	if (frame[0] != 0x20 || frame[1] != IBUS_CMD) {
		return -1;
	}

	checksum = 0xffff;
	for (i = 0; i < 30; i++) {
		checksum -= frame[i];
	}

	expected = (uint16_t)frame[30] | ((uint16_t)frame[31] << 8);
	if (checksum != expected) {
		return -1;
	}

	for (i = 0; i < IBUS_CHANNELS; i++) {
		ch[i] = (uint16_t)frame[2 + i * 2] |
		        ((uint16_t)frame[3 + i * 2] << 8);
	}

	return 1;
}


#define HMC5883L_ADDR       0x1EU
#define HMC5883L_ID_A_REG   0x0AU
#define HMC5883L_ID_B_REG   0x0BU
#define HMC5883L_ID_C_REG   0x0CU
#define HMC5883L_CFG_A_REG  0x00U
#define HMC5883L_CFG_B_REG  0x01U
#define HMC5883L_MODE_REG   0x02U
#define HMC5883L_DATA_REG   0x03U

#define MPU6050_ADDR0        0x68U
#define MPU6050_ADDR1        0x69U
#define MPU6050_WHOAMI_REG   0x75U

#define MPU6050_PWR_MGMT_1   0x6BU
#define MPU6050_SMPLRT_DIV   0x19U
#define MPU6050_CONFIG       0x1AU
#define MPU6050_GYRO_CONFIG  0x1BU
#define MPU6050_ACCEL_CONFIG 0x1CU

#define MPU6050_ACCEL_XOUT_H 0x3BU
#define MPU6050_GYRO_XOUT_H  0x43U

/*
 * Duo256M:
 * GP0 = XGPIOA[28]
 * GP1 = XGPIOA[29]
 * GPIOA controller base = 0x03020000
 *
 * Bit-bang I2C mapping:
 *   GP0 -> SCL
 *   GP1 -> SDA
 *
 * Open-drain emulation:
 *   drive low  : output 0
 *   release hi : input, rely on external pull-up
 */
#define GPIOA_BASE          0x03020000UL
#ifndef GPIO_SWPORTA_DR
#define GPIO_SWPORTA_DR     0x00UL
#endif

#ifndef GPIO_SWPORTA_DDR
#define GPIO_SWPORTA_DDR    0x04UL
#endif

#ifndef GPIO_EXT_PORTA
#define GPIO_EXT_PORTA      0x50UL
#endif

#define BB_SCL_BIT          28U
#define BB_SDA_BIT          29U
#define BB_SCL_MASK         (1U << BB_SCL_BIT)
#define BB_SDA_MASK         (1U << BB_SDA_BIT)

#define BB_OK_BUS_MARK      100U

static uint32_t g_i2c_probe_count = 0;
static uint32_t g_i2c_last_err = 0;

static uint32_t g_hmc_bus = 0xff;
static uint32_t g_hmc_scan_mask = 0;
static uint32_t g_hmc_id_a = 0;
static uint32_t g_hmc_id_b = 0;
static uint32_t g_hmc_id_c = 0;
static uint32_t g_hmc_ok = 0;
static uint32_t g_hmc_data_ok = 0;
static int32_t g_hmc_x = 0;
static int32_t g_hmc_y = 0;
static int32_t g_hmc_z = 0;
static uint32_t g_hmc_cfg_done = 0;


static uint32_t g_mpu_whoami = 0;
static uint32_t g_mpu_ok = 0;
static uint32_t g_mpu_addr_found = 0;
static uint32_t g_mpu_data_ok = 0;
static int32_t g_mpu_ax = 0;
static int32_t g_mpu_ay = 0;
static int32_t g_mpu_az = 0;
static int32_t g_mpu_gx = 0;
static int32_t g_mpu_gy = 0;
static int32_t g_mpu_gz = 0;
static uint32_t g_mpu_cfg_done = 0;

#define MPU_CAL_TARGET_COUNT 50U

static uint32_t g_mpu_cal_count = 0;
static uint32_t g_mpu_cal_done = 0;

static int64_t g_mpu_acc_sum_x = 0;
static int64_t g_mpu_acc_sum_y = 0;
static int64_t g_mpu_acc_sum_z = 0;

static int64_t g_mpu_gyro_sum_x = 0;
static int64_t g_mpu_gyro_sum_y = 0;
static int64_t g_mpu_gyro_sum_z = 0;

static int32_t g_mpu_acc_bias_x = 0;
static int32_t g_mpu_acc_bias_y = 0;
static int32_t g_mpu_acc_bias_z = 0;

static int32_t g_mpu_gyro_bias_x = 0;
static int32_t g_mpu_gyro_bias_y = 0;
static int32_t g_mpu_gyro_bias_z = 0;
static uint32_t g_sensor_ready = 0;

static int32_t g_mpu_gx_corr = 0;
static int32_t g_mpu_gy_corr = 0;
static int32_t g_mpu_gz_corr = 0;

static int32_t g_mpu_ax_corr = 0;
static int32_t g_mpu_ay_corr = 0;
static int32_t g_mpu_az_corr = 0;

static uint32_t g_att_ok = 0;

static int32_t g_att_roll_cd = 0;
static int32_t g_att_pitch_cd = 0;
static int32_t g_att_yaw_cd = 0;

static int32_t g_rate_roll_raw = 0;
static int32_t g_rate_pitch_raw = 0;
static int32_t g_rate_yaw_raw = 0;

static uint32_t g_level_trim_done = 0;
static int32_t g_level_roll_cd = 0;
static int32_t g_level_pitch_cd = 0;

static int32_t g_body_roll_cd = 0;
static int32_t g_body_pitch_cd = 0;
static int32_t g_body_yaw_cd = 0;

static int32_t g_body_rate_roll_raw = 0;
static int32_t g_body_rate_pitch_raw = 0;
static int32_t g_body_rate_yaw_raw = 0;

static int32_t g_angle_corr_roll = 0;
static int32_t g_angle_corr_pitch = 0;
static int32_t g_rate_corr_roll = 0;
static int32_t g_rate_corr_pitch = 0;
static int32_t g_rate_corr_yaw = 0;
static uint32_t g_sensor_state_error = 0;

static int32_t g_sp_roll_rate_raw = 0;
static int32_t g_sp_pitch_rate_raw = 0;
static int32_t g_sp_roll_cd = 0;
static int32_t g_sp_pitch_cd = 0;
static int32_t g_sp_yaw_rate_raw = 0;

/*
 * RC throttle is collective thrust / base PWM intent for the mixer.
 * It is not a direct per-motor speed command.
 */
static uint32_t g_sp_throttle_us = 1000;

static int32_t g_pid_roll = 0;
static int32_t g_pid_pitch = 0;
static int32_t g_pid_yaw = 0;

static int32_t g_mix_m0 = 1000;
static int32_t g_mix_m1 = 1000;
static int32_t g_mix_m2 = 1000;
static int32_t g_mix_m3 = 1000;

static uint32_t g_motor_pwm_us[4] = {1000, 1000, 1000, 1000};
static uint32_t g_motor_min_spin_us[4] = {
	1220,  /* motor0 LF GP2 PWM_7 */
	1235,  /* motor1 RF GP4 PWM_5 */
	1220,  /* motor2 RR GP9 PWM_4 */
	1220   /* motor3 LB GP3 PWM_6 */
};
static int32_t g_motor_trim_us[4] = {
	10,  /* motor0 LF GP2 PWM_7 */
	0,   /* motor1 RF GP4 PWM_5 */
	0,   /* motor2 RR GP9 PWM_4 */
	10   /* motor3 LB GP3 PWM_6 */
};
static uint32_t g_motor_armed = 0;
static uint32_t g_motor_output_enabled = 0;
static uint32_t g_motor_safety_reason = 0;
static uint32_t g_rc_ch0_us = 1500;
static uint32_t g_rc_ch1_us = 1500;
static uint32_t g_rc_ch3_us = 1500;
static uint32_t g_rc_valid = 0;
static uint32_t g_rc_throttle_us = 1000;
static uint32_t g_rc_arm_switch = 0;
static uint32_t g_rc_arm_allowed = 0;
static uint32_t g_rc_last_frame_tick = 0;
static uint32_t g_rc_frame_age_ms = 0;
static uint32_t g_rc_seen_arm_low = 0;
static uint32_t g_rc_arm_latched = 0;
static uint32_t g_rc_prev_arm_switch = 0;

#define MOTOR_PWM_MIN_US 1000U
#define MOTOR_PWM_IDLE_US 1000U
#define MOTOR_PWM_MAX_US 2000U
#define MOTOR_PWM_PERIOD_US 20000U
#define MOTOR_PWM_SAFE_US   1000U
#define MOTOR_STAGE_ALLOW_THROTTLE_OUTPUT 1

#define PWM_ESC_ACTIVE_HIGH 1
#define PWM_NS_PER_US 1000U

#define MOTOR_TEST_MAX_US 1500U
#define ESC_ARM_DELAY_MS 3000U

#define MOTOR_TRIM_ENABLE 1
#define MOTOR_TRIM_MIN_US (-50)
#define MOTOR_TRIM_MAX_US 50
#define MOTOR_MIN_SPIN_APPLY_US 1010U
#define CONTROL_MIN_ACTIVE_THROTTLE_US 1250U
#define CONTROL_STAGE_RATE_ANGLE_ASSIST 2U

#define SENSOR_STATE_OK       0U
#define SENSOR_STATE_HMC_BAD  1U
#define SENSOR_STATE_MPU_BAD  2U
#define SENSOR_STATE_MPU_CAL  4U

#define MOTOR_SAFE_OK              0U
#define MOTOR_SAFE_FAILSAFE        1U
#define MOTOR_SAFE_SENSOR_NOT_RDY  2U
#define MOTOR_SAFE_NOT_ARMED       3U
#define MOTOR_SAFE_THROTTLE_HIGH   4U

#define RC_THROTTLE_LOW_US 1100U
#define RC_THROTTLE_TEST_KILL_US 1350U
#define RC_ARM_SWITCH_THRESHOLD_US 1500U
#define RC_LOST_TIMEOUT_MS 300U

#define RC_MID_US 1500
#define RC_DEADBAND_US 20
#define RC_ROLL_SIGN   1
#define RC_PITCH_SIGN -1
#define RC_YAW_SIGN    1

#define BODY_RATE_ROLL_FROM_GX   1
#define BODY_RATE_PITCH_FROM_GY  1
#define BODY_RATE_YAW_FROM_GZ    1

#define BODY_ROLL_SIGN   1
#define BODY_PITCH_SIGN  -1
#define BODY_YAW_SIGN    1

#define ATT_SET_MAX_CD 2000
#define RATE_ROLL_SET_MAX_RAW 600
#define RATE_PITCH_SET_MAX_RAW 600
#define RATE_YAW_SET_MAX_RAW 500

#define RATE_PID_STAGE_ENABLE 1
#define RATE_PID_MAX_CORR_US 40
#define RATE_PID_ROLL_DIV 12
#define RATE_PID_PITCH_DIV 12
#define RATE_PID_YAW_DIV 16

#define ANGLE_ASSIST_ENABLE 1
#define ANGLE_ASSIST_MAX_CORR_US 25
#define ANGLE_ASSIST_DIV 180
#define ANGLE_ASSIST_ROLL_SIGN 1
#define ANGLE_ASSIST_PITCH_SIGN 1

#define AMP_CONFIG_DEADZONE_MIN_US 5U
#define AMP_CONFIG_DEADZONE_MAX_US 80U
#define AMP_CONFIG_RATE_DIV_MIN    8U
#define AMP_CONFIG_RATE_DIV_MAX    80U
#define AMP_CONFIG_ANGLE_DIV_MIN   100U
#define AMP_CONFIG_ANGLE_DIV_MAX   1200U

#define AMP_STATUS_RC_VALID        (1U << 0)
#define AMP_STATUS_SENSOR_READY    (1U << 1)
#define AMP_STATUS_MOTOR_ARMED     (1U << 2)
#define AMP_STATUS_MOTOR_OUT       (1U << 3)
#define AMP_STATUS_FAILSAFE        (1U << 4)
#define AMP_STATUS_EMERGENCY       (1U << 5)
#define AMP_STATUS_ATT_OK          (1U << 6)
#define AMP_STATUS_ARM_LATCHED     (1U << 7)
#define AMP_STATUS_SAFE_SHIFT      24U
#define AMP_STAGE3_EXTERNAL_MODULES 0

static uint32_t g_config_deadzone_us = RC_DEADBAND_US;
static uint32_t g_config_rate_roll_div = RATE_PID_ROLL_DIV;
static uint32_t g_config_rate_pitch_div = RATE_PID_PITCH_DIV;
static uint32_t g_config_rate_yaw_div = RATE_PID_YAW_DIV;
static uint32_t g_config_angle_div = ANGLE_ASSIST_DIV;

static const uint8_t g_duo_pwm_id_map[4] = {1U, 1U, 1U, 1U};
static const uint32_t g_duo_pwm_channel_map[4] = {3U, 1U, 0U, 2U};

static uint32_t g_amp_shm_poll_count = 0;
static uint32_t g_amp_emergency_stop_latched = 0;
static uint32_t g_amp_last_emergency_seq = 0;
static uint32_t g_amp_last_config_slot_seq = 0;
static uint32_t g_amp_last_config_update_seq = 0;
static uint32_t g_amp_last_arm_seq = 0;
static uint32_t g_amp_last_setpoint_seq = 0;
static uint32_t g_amp_config_seq = 0;
static uint32_t g_amp_config_update_count = 0;
static uint32_t g_amp_rt_report_seq = 0;
static uint32_t g_amp_status_flags = 0;
static uint32_t g_stage3_step_count = 0;
static uint32_t g_stage3_actuator_map_ok = 0;
static uint32_t g_stage3_external_enabled = 0;

#if AMP_STAGE3_EXTERNAL_MODULES
extern uint32_t amp_stage3_duo_pwm_map_ok(void);
#endif

static pwm_configuration_t g_pwm_cfg[4];
static uint32_t g_pwm_backend_ready = 0;
static uint32_t g_pwm_backend_err = 0;
static uint32_t g_pwm_real_enabled = 0;
static uint32_t g_pwm_period_us_dbg = 20000;
static uint32_t g_pwm_duty_us_dbg[4] = {1000, 1000, 1000, 1000};
static uint32_t g_pwm_period_clk_dbg[4] = {0};
static uint32_t g_pwm_duty_clk_dbg[4] = {0};
static uint32_t g_pwm_start_reg_dbg[4] = {0};
static uint32_t g_pwm_oe_reg_dbg[4] = {0};
static uint32_t g_pwm_active_high_dbg = PWM_ESC_ACTIVE_HIGH;
static uint32_t g_esc_arm_delay_done = 0;
static uint32_t g_esc_arm_delay_ms = 0;
static TickType_t g_esc_arm_start_tick = 0;

#define PWM_BACKEND_ERR_BAD_CHANNEL   2U

static inline uint32_t gpioa_read(uint32_t off)
{
	return *(volatile uint32_t *)(GPIOA_BASE + off);
}

static inline void gpioa_write(uint32_t off, uint32_t val)
{
	*(volatile uint32_t *)(GPIOA_BASE + off) = val;
}

static void bb_delay(void)
{
	volatile int i;

	for (i = 0; i < 300; i++) {
		__asm__ volatile("nop");
	}
}

static void bb_gpio_output_low(uint32_t mask)
{
	uint32_t v;

	v = gpioa_read(GPIO_SWPORTA_DR);
	v &= ~mask;
	gpioa_write(GPIO_SWPORTA_DR, v);

	v = gpioa_read(GPIO_SWPORTA_DDR);
	v |= mask;
	gpioa_write(GPIO_SWPORTA_DDR, v);
}

static void bb_gpio_release(uint32_t mask)
{
	uint32_t v;

	/*
	 * Release line as input. External pull-up should pull it high.
	 */
	v = gpioa_read(GPIO_SWPORTA_DDR);
	v &= ~mask;
	gpioa_write(GPIO_SWPORTA_DDR, v);
}

static int bb_gpio_read(uint32_t mask)
{
	return (gpioa_read(GPIO_EXT_PORTA) & mask) ? 1 : 0;
}

static void bb_scl_low(void)
{
	bb_gpio_output_low(BB_SCL_MASK);
	bb_delay();
}

static void bb_scl_release(void)
{
	bb_gpio_release(BB_SCL_MASK);
	bb_delay();
}

static void bb_sda_low(void)
{
	bb_gpio_output_low(BB_SDA_MASK);
	bb_delay();
}

static void bb_sda_release(void)
{
	bb_gpio_release(BB_SDA_MASK);
	bb_delay();
}

static int bb_sda_read(void)
{
	return bb_gpio_read(BB_SDA_MASK);
}

static void bb_i2c_idle(void)
{
	bb_sda_release();
	bb_scl_release();
	bb_delay();
}

static void bb_i2c_start(void)
{
	bb_sda_release();
	bb_scl_release();
	bb_delay();

	bb_sda_low();
	bb_delay();

	bb_scl_low();
	bb_delay();
}

static void bb_i2c_stop(void)
{
	bb_sda_low();
	bb_delay();

	bb_scl_release();
	bb_delay();

	bb_sda_release();
	bb_delay();
}

static void bb_i2c_write_bit(int bit)
{
	if (bit) {
		bb_sda_release();
	} else {
		bb_sda_low();
	}

	bb_delay();
	bb_scl_release();
	bb_delay();
	bb_scl_low();
	bb_delay();
}

static int bb_i2c_read_bit(void)
{
	int bit;

	bb_sda_release();
	bb_delay();

	bb_scl_release();
	bb_delay();

	bit = bb_sda_read();

	bb_scl_low();
	bb_delay();

	return bit;
}

static int bb_i2c_write_byte(uint8_t v)
{
	int i;
	int ack;

	for (i = 7; i >= 0; i--) {
		bb_i2c_write_bit((v >> i) & 1);
	}

	/*
	 * ACK is active low.
	 */
	ack = bb_i2c_read_bit();
	return ack == 0 ? 0 : -1;
}

static uint8_t bb_i2c_read_byte(int ack)
{
	int i;
	uint8_t v = 0;

	for (i = 7; i >= 0; i--) {
		v <<= 1;
		if (bb_i2c_read_bit()) {
			v |= 1;
		}
	}

	/*
	 * ack=1 means send ACK low.
	 * ack=0 means send NACK high.
	 */
	bb_i2c_write_bit(ack ? 0 : 1);

	return v;
}

static int bb_i2c_read_reg(uint8_t dev, uint8_t reg, uint8_t *val)
{
	int ret;

	if (val == NULL) {
		return -10;
	}

	*val = 0;

	bb_i2c_idle();

	bb_i2c_start();

	ret = bb_i2c_write_byte((uint8_t)(dev << 1));
	if (ret != 0) {
		bb_i2c_stop();
		return -1;  /* no ACK on device write address */
	}

	ret = bb_i2c_write_byte(reg);
	if (ret != 0) {
		bb_i2c_stop();
		return -2;  /* no ACK on register address */
	}

	bb_i2c_start();

	ret = bb_i2c_write_byte((uint8_t)((dev << 1) | 1U));
	if (ret != 0) {
		bb_i2c_stop();
		return -3;  /* no ACK on device read address */
	}

	*val = bb_i2c_read_byte(0);  /* NACK after one byte */
	bb_i2c_stop();

	return 0;
}

static int bb_i2c_write_reg(uint8_t dev, uint8_t reg, uint8_t val)
{
	int ret;

	bb_i2c_idle();

	bb_i2c_start();

	ret = bb_i2c_write_byte((uint8_t)(dev << 1));
	if (ret != 0) {
		bb_i2c_stop();
		return -1;
	}

	ret = bb_i2c_write_byte(reg);
	if (ret != 0) {
		bb_i2c_stop();
		return -2;
	}

	ret = bb_i2c_write_byte(val);
	if (ret != 0) {
		bb_i2c_stop();
		return -3;
	}

	bb_i2c_stop();
	return 0;
}

static int bb_i2c_read_regs(uint8_t dev, uint8_t reg, uint8_t *buf, int len)
{
	int ret;
	int i;

	if (buf == NULL || len <= 0) {
		return -10;
	}

	bb_i2c_idle();

	bb_i2c_start();

	ret = bb_i2c_write_byte((uint8_t)(dev << 1));
	if (ret != 0) {
		bb_i2c_stop();
		return -1;
	}

	ret = bb_i2c_write_byte(reg);
	if (ret != 0) {
		bb_i2c_stop();
		return -2;
	}

	bb_i2c_start();

	ret = bb_i2c_write_byte((uint8_t)((dev << 1) | 1U));
	if (ret != 0) {
		bb_i2c_stop();
		return -3;
	}

	for (i = 0; i < len; i++) {
		buf[i] = bb_i2c_read_byte(i != (len - 1));
	}

	bb_i2c_stop();
	return 0;
}


static int hmc5883l_config_once(void)
{
	int ret;

	if (g_hmc_cfg_done) {
		return 0;
	}

	/*
	 * Config A 0x70:
	 *   8 samples averaged, 15 Hz, normal measurement.
	 */
	ret = bb_i2c_write_reg(HMC5883L_ADDR, HMC5883L_CFG_A_REG, 0x70);
	if (ret != 0) {
		return 0x10 | (uint32_t)(-ret & 0xf);
	}

	/*
	 * Config B 0x20:
	 *   gain +/-1.3 Ga.
	 */
	ret = bb_i2c_write_reg(HMC5883L_ADDR, HMC5883L_CFG_B_REG, 0x60);
	if (ret != 0) {
		return 0x20 | (uint32_t)(-ret & 0xf);
	}

	/*
	 * Mode 0x00:
	 *   continuous measurement mode.
	 */
	ret = bb_i2c_write_reg(HMC5883L_ADDR, HMC5883L_MODE_REG, 0x00);
	if (ret != 0) {
		return 0x30 | (uint32_t)(-ret & 0xf);
	}

	g_hmc_cfg_done = 1;
	return 0;
}


static int hmc5883l_read_raw_once(void)
{
	uint8_t buf[6];
	int16_t x;
	int16_t y;
	int16_t z;
	int ret;

	ret = hmc5883l_config_once();
	if (ret != 0) {
		g_hmc_data_ok = 0;
		return 0x400 | (uint32_t)(ret & 0xff);
	}

	ret = bb_i2c_read_regs(HMC5883L_ADDR, HMC5883L_DATA_REG, buf, 6);
	if (ret != 0) {
		g_hmc_data_ok = 0;
		return 0x500 | (uint32_t)(-ret & 0xff);
	}

	x = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
	z = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
	y = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

	g_hmc_x = x;
	g_hmc_y = y;
	g_hmc_z = z;
	
	if (x == -4096 || y == -4096 || z == -4096) {
		g_hmc_data_ok = 0;
		return 0x600;
	}
	
	g_hmc_data_ok = 1;
	return 0;
}


static int mpu6050_config_once(uint8_t addr)
{
	int ret;

	if (g_mpu_cfg_done && g_mpu_addr_found == addr) {
		return 0;
	}

	/*
	 * Wake up MPU6050.
	 * PWR_MGMT_1 = 0x00: sleep off, internal oscillator.
	 */
	ret = bb_i2c_write_reg(addr, MPU6050_PWR_MGMT_1, 0x00);
	if (ret != 0) {
		return 0x100 | (uint32_t)(-ret & 0xff);
	}

	/*
	 * Sample rate divider.
	 * Gyro output rate is 1kHz after DLPF enabled.
	 * 0x09 -> 100Hz sample rate.
	 */
	ret = bb_i2c_write_reg(addr, MPU6050_SMPLRT_DIV, 0x09);
	if (ret != 0) {
		return 0x110 | (uint32_t)(-ret & 0xff);
	}

	/*
	 * CONFIG = 0x03:
	 * DLPF enabled, about 44Hz accel / 42Hz gyro bandwidth.
	 * Good for bring-up, less noisy than raw high-bandwidth mode.
	 */
	ret = bb_i2c_write_reg(addr, MPU6050_CONFIG, 0x03);
	if (ret != 0) {
		return 0x120 | (uint32_t)(-ret & 0xff);
	}

	/*
	 * GYRO_CONFIG = 0x00:
	 * +/-250 deg/s.
	 */
	ret = bb_i2c_write_reg(addr, MPU6050_GYRO_CONFIG, 0x00);
	if (ret != 0) {
		return 0x130 | (uint32_t)(-ret & 0xff);
	}

	/*
	 * ACCEL_CONFIG = 0x00:
	 * +/-2g.
	 */
	ret = bb_i2c_write_reg(addr, MPU6050_ACCEL_CONFIG, 0x00);
	if (ret != 0) {
		return 0x140 | (uint32_t)(-ret & 0xff);
	}

	g_mpu_cfg_done = 1;
	g_mpu_addr_found = addr;
	return 0;
}

static int mpu6050_read_raw_once(uint8_t addr)
{
	uint8_t buf[14];
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int ret;

	ret = mpu6050_config_once(addr);
	if (ret != 0) {
		g_mpu_data_ok = 0;
		return 0x200 | (uint32_t)(ret & 0xff);
	}

	/*
	 * Read 14 bytes from 0x3B:
	 * accel X/Y/Z, temp, gyro X/Y/Z.
	 */
	ret = bb_i2c_read_regs(addr, MPU6050_ACCEL_XOUT_H, buf, 14);
	if (ret != 0) {
		g_mpu_data_ok = 0;
		return 0x300 | (uint32_t)(-ret & 0xff);
	}

	ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
	ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
	az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

	gx = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
	gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
	gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

	g_mpu_ax = ax;
	g_mpu_ay = ay;
	g_mpu_az = az;
	g_mpu_gx = gx;
	g_mpu_gy = gy;
	g_mpu_gz = gz;
	g_mpu_data_ok = 1;

	return 0;
}

static void mpu6050_cal_update_once(void)
{
	if (!g_mpu_data_ok) {
		return;
	}

	if (g_mpu_cal_done) {
		return;
	}

	g_mpu_acc_sum_x += g_mpu_ax;
	g_mpu_acc_sum_y += g_mpu_ay;
	g_mpu_acc_sum_z += g_mpu_az;

	g_mpu_gyro_sum_x += g_mpu_gx;
	g_mpu_gyro_sum_y += g_mpu_gy;
	g_mpu_gyro_sum_z += g_mpu_gz;

	g_mpu_cal_count++;

	if (g_mpu_cal_count >= MPU_CAL_TARGET_COUNT) {
		g_mpu_acc_bias_x = (int32_t)(g_mpu_acc_sum_x / (int64_t)g_mpu_cal_count);
		g_mpu_acc_bias_y = (int32_t)(g_mpu_acc_sum_y / (int64_t)g_mpu_cal_count);
		g_mpu_acc_bias_z = (int32_t)(g_mpu_acc_sum_z / (int64_t)g_mpu_cal_count);

		g_mpu_gyro_bias_x = (int32_t)(g_mpu_gyro_sum_x / (int64_t)g_mpu_cal_count);
		g_mpu_gyro_bias_y = (int32_t)(g_mpu_gyro_sum_y / (int64_t)g_mpu_cal_count);
		g_mpu_gyro_bias_z = (int32_t)(g_mpu_gyro_sum_z / (int64_t)g_mpu_cal_count);

		g_mpu_cal_done = 1;
	}
}


static void sensor_update_corrected_once(void)
{
	g_sensor_ready = 0;
	g_att_ok = 0;
	g_sensor_state_error = SENSOR_STATE_OK;

	if (!g_hmc_ok || !g_hmc_data_ok) {
		g_sensor_state_error |= SENSOR_STATE_HMC_BAD;
		return;
	}

	if (!g_mpu_ok || !g_mpu_data_ok) {
		g_sensor_state_error |= SENSOR_STATE_MPU_BAD;
		return;
	}

	if (!g_mpu_cal_done) {
		g_sensor_state_error |= SENSOR_STATE_MPU_CAL;
		return;
	}

	g_mpu_ax_corr = g_mpu_ax;
	g_mpu_ay_corr = g_mpu_ay;
	g_mpu_az_corr = g_mpu_az;

	g_mpu_gx_corr = g_mpu_gx - g_mpu_gyro_bias_x;
	g_mpu_gy_corr = g_mpu_gy - g_mpu_gyro_bias_y;
	g_mpu_gz_corr = g_mpu_gz - g_mpu_gyro_bias_z;

	g_sensor_ready = 1;
}

static uint32_t amp_abs32_to_u32(int32_t v)
{
	if (v < 0) {
		return (uint32_t)(-(int64_t)v);
	}

	return (uint32_t)v;
}

static uint32_t amp_isqrt64(uint64_t v)
{
	uint64_t bit = 1ULL << 62;
	uint64_t res = 0;

	while (bit > v) {
		bit >>= 2;
	}

	while (bit != 0) {
		if (v >= res + bit) {
			v -= res + bit;
			res = (res >> 1) + bit;
		} else {
			res >>= 1;
		}
		bit >>= 2;
	}

	return (uint32_t)res;
}

static int32_t amp_atan2_cd_approx(int32_t y, int32_t x)
{
	uint32_t ax = amp_abs32_to_u32(x);
	uint32_t ay = amp_abs32_to_u32(y);
	int32_t base;

	if (ax == 0 && ay == 0) {
		return 0;
	}

	if (ax >= ay) {
		base = ax == 0 ? 9000 : (int32_t)(((uint64_t)ay * 4500U) / ax);
	} else {
		base = 9000 - (int32_t)(((uint64_t)ax * 4500U) / ay);
	}

	if (x >= 0) {
		return y >= 0 ? base : -base;
	}

	return y >= 0 ? (18000 - base) : (base - 18000);
}

static void attitude_update_debug_once(void)
{
	uint64_t ay2;
	uint64_t az2;
	int32_t horiz;

	g_att_ok = 0;

	if (!g_sensor_ready) {
		return;
	}

	if (!g_hmc_data_ok || !g_mpu_data_ok) {
		return;
	}

	/*
	 * Rate loop raw inputs.
	 * These are gyro bias-corrected raw values.
	 * Keep raw units for now; PID/mixer will consume these later.
	 */
	g_rate_roll_raw = g_mpu_gx_corr;
	g_rate_pitch_raw = g_mpu_gy_corr;
	g_rate_yaw_raw = g_mpu_gz_corr;

	/*
	 * Attitude debug:
	 * Roll/pitch from accelerometer.
	 * Yaw from HMC5883L magnetometer.
	 *
	 * This RTOS build may not link libm, so use a lightweight integer
	 * atan2 approximation and integer sqrt for debug centi-degrees.
	 */
	ay2 = (uint64_t)amp_abs32_to_u32(g_mpu_ay_corr) *
	      (uint64_t)amp_abs32_to_u32(g_mpu_ay_corr);
	az2 = (uint64_t)amp_abs32_to_u32(g_mpu_az_corr) *
	      (uint64_t)amp_abs32_to_u32(g_mpu_az_corr);
	horiz = (int32_t)amp_isqrt64(ay2 + az2);

	g_att_roll_cd = amp_atan2_cd_approx(g_mpu_ay_corr, g_mpu_az_corr);
	g_att_pitch_cd = amp_atan2_cd_approx(-g_mpu_ax_corr, horiz);
	g_att_yaw_cd = amp_atan2_cd_approx(g_hmc_y, g_hmc_x);

	if (g_att_yaw_cd < 0) {
		g_att_yaw_cd += 36000;
	}

	g_att_ok = 1;
}

static void body_axis_update_once(void)
{
	if (!g_att_ok || !g_sensor_ready) {
		return;
	}

	/*
	 * The vehicle is placed level during boot/calibration.
	 * Use the first valid attitude as level trim.
	 */
	if (!g_level_trim_done) {
		g_level_roll_cd = g_att_roll_cd;
		g_level_pitch_cd = g_att_pitch_cd;
		g_level_trim_done = 1;
	}

	g_body_roll_cd = BODY_ROLL_SIGN * (g_att_roll_cd - g_level_roll_cd);
	g_body_pitch_cd = BODY_PITCH_SIGN * (g_att_pitch_cd - g_level_pitch_cd);
	/*
	 * HMC yaw is kept as heading/debug information. Current control is yaw
	 * rate only; there is no yaw heading hold outer loop in this stage.
	 */
	g_body_yaw_cd = BODY_YAW_SIGN * g_att_yaw_cd;

	/*
	 * Body rate mapping.
	 * For now use gyro corrected raw values directly.
	 * If physical MPU orientation is reversed, only signs/macros should change.
	 */
#if BODY_RATE_ROLL_FROM_GX
	g_body_rate_roll_raw = BODY_ROLL_SIGN * g_rate_roll_raw;
#else
	g_body_rate_roll_raw = 0;
#endif
#if BODY_RATE_PITCH_FROM_GY
	g_body_rate_pitch_raw = BODY_PITCH_SIGN * g_rate_pitch_raw;
#else
	g_body_rate_pitch_raw = 0;
#endif
#if BODY_RATE_YAW_FROM_GZ
	g_body_rate_yaw_raw = BODY_YAW_SIGN * g_rate_yaw_raw;
#else
	g_body_rate_yaw_raw = 0;
#endif
}

static void motor_set_all(uint32_t us)
{
	if (us < MOTOR_PWM_MIN_US) {
		us = MOTOR_PWM_MIN_US;
	}
	if (us > MOTOR_PWM_MAX_US) {
		us = MOTOR_PWM_MAX_US;
	}

	g_motor_pwm_us[0] = us;
	g_motor_pwm_us[1] = us;
	g_motor_pwm_us[2] = us;
	g_motor_pwm_us[3] = us;
}

static void motor_force_safe(uint32_t reason)
{
	g_motor_armed = 0;
	g_motor_output_enabled = 0;
	g_motor_safety_reason = reason;
	g_pid_roll = 0;
	g_pid_pitch = 0;
	g_pid_yaw = 0;
	g_angle_corr_roll = 0;
	g_angle_corr_pitch = 0;
	g_rate_corr_roll = 0;
	g_rate_corr_pitch = 0;
	g_rate_corr_yaw = 0;
	g_mix_m0 = MOTOR_PWM_IDLE_US;
	g_mix_m1 = MOTOR_PWM_IDLE_US;
	g_mix_m2 = MOTOR_PWM_IDLE_US;
	g_mix_m3 = MOTOR_PWM_IDLE_US;
	motor_set_all(MOTOR_PWM_IDLE_US);
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

static uint32_t amp_config_float_to_div(float v,
				       uint32_t fallback,
				       uint32_t lo,
				       uint32_t hi)
{
	if (v < (float)lo || v > (float)hi) {
		return fallback;
	}

	return clamp_u32((uint32_t)v, lo, hi);
}

static volatile amp_shared_command_region_t *amp_shared_region_get(void)
{
	if (!g_amp_shm_ready || g_amp_shm_phys == 0U) {
		return NULL;
	}

	return (volatile amp_shared_command_region_t *)(uintptr_t)g_amp_shm_phys;
}

static uint32_t amp_build_status_flags(void)
{
	uint32_t flags = 0;

	if (g_rc_valid) {
		flags |= AMP_STATUS_RC_VALID;
	}
	if (g_sensor_ready) {
		flags |= AMP_STATUS_SENSOR_READY;
	}
	if (g_motor_armed) {
		flags |= AMP_STATUS_MOTOR_ARMED;
	}
	if (g_motor_output_enabled) {
		flags |= AMP_STATUS_MOTOR_OUT;
	}
	if (g_motor_safety_reason == MOTOR_SAFE_FAILSAFE) {
		flags |= AMP_STATUS_FAILSAFE;
	}
	if (g_amp_emergency_stop_latched) {
		flags |= AMP_STATUS_EMERGENCY;
	}
	if (g_att_ok) {
		flags |= AMP_STATUS_ATT_OK;
	}
	if (g_rc_arm_latched) {
		flags |= AMP_STATUS_ARM_LATCHED;
	}

	flags |= (g_motor_safety_reason & 0xffU) << AMP_STATUS_SAFE_SHIFT;
	return flags;
}

static void amp_shared_region_poll_once(void)
{
	volatile amp_shared_command_region_t *r;
	uint32_t seq;
	uint32_t update_seq;
	uint32_t deadzone;
	uint32_t rate_div;
	uint32_t angle_div;

	r = amp_shared_region_get();
	if (r == NULL) {
		return;
	}

	g_amp_shm_poll_count++;

	if (r->magic != AMP_SHARED_REGION_MAGIC ||
	    r->version != AMP_SHARED_REGION_VERSION) {
		return;
	}

	/*
	 * EmergencyStop is the only Linux-side command allowed to affect motor
	 * safety immediately. A non-zero reason latches emergency stop. A new
	 * emergency slot with reason 0 is treated as an explicit clear command.
	 */
	seq = r->emergency_stop.seq;
	if (seq != g_amp_last_emergency_seq) {
		g_amp_last_emergency_seq = seq;
		if (r->emergency_stop.payload.reason == 0U) {
			g_amp_emergency_stop_latched = 0;
		} else {
			g_amp_emergency_stop_latched = 1;
		}
	}

	if (g_amp_emergency_stop_latched) {
		motor_force_safe(MOTOR_SAFE_FAILSAFE);
	}

	seq = r->config.seq;
	update_seq = r->config.payload.update_seq;
	if ((seq != g_amp_last_config_slot_seq ||
	     update_seq != g_amp_last_config_update_seq) &&
	    (seq != 0U || update_seq != 0U)) {
		g_amp_last_config_slot_seq = seq;
		g_amp_last_config_update_seq = update_seq;
		g_amp_config_seq = seq;
		g_amp_config_update_count++;

		deadzone = r->config.payload.deadzone_us;
		if (deadzone != 0U) {
			g_config_deadzone_us = clamp_u32(deadzone,
							AMP_CONFIG_DEADZONE_MIN_US,
							AMP_CONFIG_DEADZONE_MAX_US);
		}

		/*
		 * Bring-up convention: rate_p and angle_p may carry requested
		 * integer dividers. Values outside the conservative range are
		 * ignored so Linux cannot make the controller dangerously hot.
		 */
		rate_div = amp_config_float_to_div(r->config.payload.rate_p,
						   g_config_rate_roll_div,
						   AMP_CONFIG_RATE_DIV_MIN,
						   AMP_CONFIG_RATE_DIV_MAX);
		angle_div = amp_config_float_to_div(r->config.payload.angle_p,
						    g_config_angle_div,
						    AMP_CONFIG_ANGLE_DIV_MIN,
						    AMP_CONFIG_ANGLE_DIV_MAX);

		g_config_rate_roll_div = rate_div;
		g_config_rate_pitch_div = rate_div;
		g_config_rate_yaw_div = rate_div;
		g_config_angle_div = angle_div;
	}

	/*
	 * Linux arm/setpoint slots are intentionally not wired into the realtime
	 * control path. IBUS remains the local control source in this stage.
	 */
	if (r->arm.seq != g_amp_last_arm_seq) {
		g_amp_last_arm_seq = r->arm.seq;
	}
	if (r->setpoint.seq != g_amp_last_setpoint_seq) {
		g_amp_last_setpoint_seq = r->setpoint.seq;
	}
}

static void amp_shared_region_write_rt_report_once(void)
{
	volatile amp_shared_command_region_t *r;
	amp_rt_report_t report;
	static TickType_t last_rt_report_tick = 0;
	TickType_t now;
	uint32_t seq;

	r = amp_shared_region_get();
	if (r == NULL) {
		return;
	}

	if (r->magic != AMP_SHARED_REGION_MAGIC ||
	    r->version != AMP_SHARED_REGION_VERSION) {
		return;
	}

	now = xTaskGetTickCount();
	if ((now - last_rt_report_tick) < pdMS_TO_TICKS(20)) {
		return;
	}
	last_rt_report_tick = now;

	seq = g_amp_rt_report_seq + 1U;
	g_amp_rt_report_seq = seq;
	g_amp_status_flags = amp_build_status_flags();

	report.monotonic_us =
		(uint64_t)now * (uint64_t)portTICK_PERIOD_MS * 1000ULL;
	report.roll_deg = (float)g_body_roll_cd / 100.0f;
	report.pitch_deg = (float)g_body_pitch_cd / 100.0f;
	report.yaw_deg = (float)g_body_yaw_cd / 100.0f;
	report.roll_rate_dps = (float)g_body_rate_roll_raw;
	report.pitch_rate_dps = (float)g_body_rate_pitch_raw;
	report.yaw_rate_dps = (float)g_body_rate_yaw_raw;
	report.motor_pwm_us[0] = (uint16_t)g_motor_pwm_us[0];
	report.motor_pwm_us[1] = (uint16_t)g_motor_pwm_us[1];
	report.motor_pwm_us[2] = (uint16_t)g_motor_pwm_us[2];
	report.motor_pwm_us[3] = (uint16_t)g_motor_pwm_us[3];
	report.status_flags = g_amp_status_flags;
	report.seq = seq;

	r->rt_report.payload = report;
	r->rt_report.payload_crc = 0U;
	r->rt_report.seq = seq;

	flush_dcache_range((uintptr_t)&r->rt_report, sizeof(r->rt_report));
}

static uint32_t stage3_actuator_map_check(void)
{
	return g_duo_pwm_id_map[0] == 1U && g_duo_pwm_channel_map[0] == 3U &&
	       g_duo_pwm_id_map[1] == 1U && g_duo_pwm_channel_map[1] == 1U &&
	       g_duo_pwm_id_map[2] == 1U && g_duo_pwm_channel_map[2] == 0U &&
	       g_duo_pwm_id_map[3] == 1U && g_duo_pwm_channel_map[3] == 2U;
}

static void stage3_flight_node_step_legacy_fallback(void)
{
	/*
	 * Stage 3 first cut:
	 * comm_main.c remains the realtime glue/main task while rt_core modules
	 * are introduced. The verified bring-up control path still owns PWM
	 * output; this hook only records that the new module boundary is alive
	 * and that the formal ActuatorDuoPwm map matches the active backend.
	 */
	g_stage3_step_count++;
#if AMP_STAGE3_EXTERNAL_MODULES
	g_stage3_actuator_map_ok = amp_stage3_duo_pwm_map_ok();
	g_stage3_external_enabled = 1;
#else
	g_stage3_actuator_map_ok = stage3_actuator_map_check();
	g_stage3_external_enabled = 0;
#endif
}

static int32_t motor_apply_trim_i32(uint32_t idx, int32_t us)
{
	int32_t trim = 0;
	int32_t min_spin = MOTOR_PWM_MIN_US;

	if (idx < 4U) {
		trim = clamp_i32(g_motor_trim_us[idx],
				 MOTOR_TRIM_MIN_US,
				 MOTOR_TRIM_MAX_US);
		min_spin = clamp_i32((int32_t)g_motor_min_spin_us[idx],
				     MOTOR_PWM_MIN_US,
				     MOTOR_TEST_MAX_US);
	}

#if MOTOR_TRIM_ENABLE
	us += trim;
#endif

	if (g_sp_throttle_us >= MOTOR_MIN_SPIN_APPLY_US && us < min_spin) {
		us = min_spin;
	}

	return clamp_i32(us, MOTOR_PWM_MIN_US, MOTOR_TEST_MAX_US);
}

static int32_t rc_norm_deadband(int32_t us)
{
	int32_t d = us - RC_MID_US;
	int32_t deadzone = (int32_t)g_config_deadzone_us;

	if (d > -deadzone && d < deadzone) {
		return 0;
	}

	return clamp_i32(d, -500, 500);
}

static void rc_update_freshness_once(void)
{
	TickType_t now;
	uint32_t age_ms;

	now = xTaskGetTickCount();

	if (g_rc_last_frame_tick == 0) {
		g_rc_frame_age_ms = 0xffffffffU;
		g_rc_valid = 0;
		g_rc_arm_allowed = 0;
		g_rc_seen_arm_low = 0;
		g_rc_arm_latched = 0;
		g_motor_armed = 0;
		return;
	}

	age_ms = (uint32_t)((now - g_rc_last_frame_tick) * portTICK_PERIOD_MS);
	g_rc_frame_age_ms = age_ms;

	if (age_ms > RC_LOST_TIMEOUT_MS) {
		g_rc_valid = 0;
		g_rc_arm_allowed = 0;
		g_rc_seen_arm_low = 0;
		g_rc_arm_latched = 0;
		g_motor_armed = 0;
		g_rc_throttle_us = 1000;
		g_rc_arm_switch = 0;
	}
}

static void motor_update_arm_gate(void)
{
	g_rc_arm_allowed = 0;

	rc_update_freshness_once();

	if (!g_rc_valid) {
		g_motor_armed = 0;
		g_rc_arm_latched = 0;
		g_rc_prev_arm_switch = g_rc_arm_switch;
		return;
	}

	if (!g_sensor_ready) {
		g_motor_armed = 0;
		g_rc_arm_latched = 0;
		g_rc_prev_arm_switch = g_rc_arm_switch;
		return;
	}

	if (g_rc_throttle_us > RC_THROTTLE_TEST_KILL_US) {
		g_motor_armed = 0;
		g_rc_arm_latched = 0;
		g_rc_prev_arm_switch = g_rc_arm_switch;
		return;
	}

	/*
	 * Important safety:
	 * after boot or RC reconnect, must first observe arm switch LOW.
	 */
	if (!g_rc_arm_switch) {
		g_rc_seen_arm_low = 1;
		g_rc_arm_latched = 0;
		g_motor_armed = 0;
		g_rc_prev_arm_switch = g_rc_arm_switch;
		return;
	}

	if (g_rc_arm_latched) {
		g_rc_prev_arm_switch = g_rc_arm_switch;
		g_rc_arm_allowed = 1;
		g_motor_armed = 1;
		return;
	}

	/*
	 * Enter armed state only when throttle is low. After latch, throttle can
	 * rise for bounded motor testing until RC_THROTTLE_TEST_KILL_US.
	 */
	if (g_rc_throttle_us > RC_THROTTLE_LOW_US) {
		g_motor_armed = 0;
		g_rc_prev_arm_switch = g_rc_arm_switch;
		return;
	}

	/*
	 * Arm only on low->high transition after seen low.
	 */
	if (g_rc_seen_arm_low &&
	    !g_rc_prev_arm_switch &&
	    g_rc_arm_switch) {
		g_rc_arm_latched = 1;
	}

	g_rc_prev_arm_switch = g_rc_arm_switch;

	if (!g_rc_arm_latched) {
		g_motor_armed = 0;
		return;
	}

	g_rc_arm_allowed = 1;
	g_motor_armed = 1;
}

static void control_update_setpoint_once(void)
{
	int32_t roll_in;
	int32_t pitch_in;
	int32_t yaw_in;

	g_sp_roll_cd = 0;
	g_sp_pitch_cd = 0;
	g_sp_roll_rate_raw = 0;
	g_sp_pitch_rate_raw = 0;
	g_sp_yaw_rate_raw = 0;
	g_sp_throttle_us = g_rc_throttle_us;

	if (!g_rc_valid) {
		g_sp_throttle_us = 1000;
		return;
	}

	roll_in = RC_ROLL_SIGN * rc_norm_deadband((int32_t)g_rc_ch0_us);
	pitch_in = RC_PITCH_SIGN * rc_norm_deadband((int32_t)g_rc_ch1_us);
	yaw_in = RC_YAW_SIGN * rc_norm_deadband((int32_t)g_rc_ch3_us);

	/*
	 * RC sticks express pilot intent:
	 * - throttle is collective base output for all motors
	 * - roll/pitch/yaw sticks are body-rate setpoints
	 * They are not direct raw motor PWM commands.
	 */
	g_sp_roll_cd = (roll_in * ATT_SET_MAX_CD) / 500;
	g_sp_pitch_cd = (pitch_in * ATT_SET_MAX_CD) / 500;
	g_sp_roll_rate_raw = (roll_in * RATE_ROLL_SET_MAX_RAW) / 500;
	g_sp_pitch_rate_raw = (pitch_in * RATE_PITCH_SET_MAX_RAW) / 500;
	g_sp_yaw_rate_raw = (yaw_in * RATE_YAW_SET_MAX_RAW) / 500;
}

static void control_update_pid_once(void)
{
	int32_t roll_rate_err;
	int32_t pitch_rate_err;
	int32_t yaw_rate_err;
	int32_t roll_pid;
	int32_t pitch_pid;
	int32_t yaw_pid;

	g_pid_roll = 0;
	g_pid_pitch = 0;
	g_pid_yaw = 0;
	g_angle_corr_roll = 0;
	g_angle_corr_pitch = 0;
	g_rate_corr_roll = 0;
	g_rate_corr_pitch = 0;
	g_rate_corr_yaw = 0;

	if (!g_sensor_ready || !g_att_ok || !g_rc_valid) {
		return;
	}

	control_update_setpoint_once();

	roll_rate_err = g_sp_roll_rate_raw - g_body_rate_roll_raw;
	pitch_rate_err = g_sp_pitch_rate_raw - g_body_rate_pitch_raw;
	yaw_rate_err = g_sp_yaw_rate_raw - g_body_rate_yaw_raw;

	/*
	 * Rate P-only.
	 * Feedback is body_rate from gyro; HMC yaw is not used for heading hold.
	 */
	if (RATE_PID_STAGE_ENABLE) {
		g_rate_corr_roll = clamp_i32(roll_rate_err / (int32_t)g_config_rate_roll_div,
					     -RATE_PID_MAX_CORR_US,
					     RATE_PID_MAX_CORR_US);
		g_rate_corr_pitch = clamp_i32(pitch_rate_err / (int32_t)g_config_rate_pitch_div,
					      -RATE_PID_MAX_CORR_US,
					      RATE_PID_MAX_CORR_US);
		g_rate_corr_yaw = clamp_i32(yaw_rate_err / (int32_t)g_config_rate_yaw_div,
					    -RATE_PID_MAX_CORR_US,
					    RATE_PID_MAX_CORR_US);
	}

	if (ANGLE_ASSIST_ENABLE) {
		if (g_sp_roll_rate_raw == 0) {
			g_angle_corr_roll =
				clamp_i32((-ANGLE_ASSIST_ROLL_SIGN * g_body_roll_cd) /
					  (int32_t)g_config_angle_div,
					  -ANGLE_ASSIST_MAX_CORR_US,
					  ANGLE_ASSIST_MAX_CORR_US);
		}

		if (g_sp_pitch_rate_raw == 0) {
			g_angle_corr_pitch =
				clamp_i32((-ANGLE_ASSIST_PITCH_SIGN * g_body_pitch_cd) /
					  (int32_t)g_config_angle_div,
					  -ANGLE_ASSIST_MAX_CORR_US,
					  ANGLE_ASSIST_MAX_CORR_US);
		}
	}

	roll_pid = g_rate_corr_roll + g_angle_corr_roll;
	pitch_pid = g_rate_corr_pitch + g_angle_corr_pitch;
	yaw_pid = g_rate_corr_yaw;

	g_pid_roll = clamp_i32(roll_pid, -RATE_PID_MAX_CORR_US, RATE_PID_MAX_CORR_US);
	g_pid_pitch = clamp_i32(pitch_pid, -RATE_PID_MAX_CORR_US, RATE_PID_MAX_CORR_US);
	g_pid_yaw = clamp_i32(yaw_pid, -RATE_PID_MAX_CORR_US, RATE_PID_MAX_CORR_US);
}

static void control_apply_active_throttle_floor_once(void)
{
	if (g_sp_throttle_us > MOTOR_PWM_IDLE_US &&
	    g_sp_throttle_us < CONTROL_MIN_ACTIVE_THROTTLE_US) {
		g_sp_throttle_us = CONTROL_MIN_ACTIVE_THROTTLE_US;
	}

	if (g_sp_throttle_us > MOTOR_TEST_MAX_US) {
		g_sp_throttle_us = MOTOR_TEST_MAX_US;
	}
}

static void control_update_mixer_once(void)
{
	int32_t t;

	t = (int32_t)g_sp_throttle_us;
	t = clamp_i32(t, MOTOR_PWM_MIN_US, MOTOR_TEST_MAX_US);

#if !RATE_PID_STAGE_ENABLE
	g_mix_m0 = t;
	g_mix_m1 = t;
	g_mix_m2 = t;
	g_mix_m3 = t;
	return;
#endif

	/*
	 * X mixer, logical order:
	 * m0 LF, m1 RF, m2 RR, m3 LB.
	 * Throttle is collective base, then small rate-PID torque correction.
	 */
	g_mix_m0 = t + g_pid_pitch + g_pid_roll - g_pid_yaw;
	g_mix_m1 = t + g_pid_pitch - g_pid_roll + g_pid_yaw;
	g_mix_m2 = t - g_pid_pitch - g_pid_roll - g_pid_yaw;
	g_mix_m3 = t - g_pid_pitch + g_pid_roll + g_pid_yaw;

	g_mix_m0 = clamp_i32(g_mix_m0, MOTOR_PWM_MIN_US, MOTOR_TEST_MAX_US);
	g_mix_m1 = clamp_i32(g_mix_m1, MOTOR_PWM_MIN_US, MOTOR_TEST_MAX_US);
	g_mix_m2 = clamp_i32(g_mix_m2, MOTOR_PWM_MIN_US, MOTOR_TEST_MAX_US);
	g_mix_m3 = clamp_i32(g_mix_m3, MOTOR_PWM_MIN_US, MOTOR_TEST_MAX_US);
}

static void motor_update_control_outputs(void)
{
	if (!g_motor_armed || !g_motor_output_enabled) {
		motor_set_all(MOTOR_PWM_IDLE_US);
		return;
	}

	if (!g_rc_valid || !g_sensor_ready) {
		motor_set_all(MOTOR_PWM_IDLE_US);
		return;
	}

	control_update_pid_once();
	control_apply_active_throttle_floor_once();
	control_update_mixer_once();

	/* Apply static per-motor trim only in the normal armed/output path. */
	g_motor_pwm_us[0] = (uint32_t)motor_apply_trim_i32(0, g_mix_m0);
	g_motor_pwm_us[1] = (uint32_t)motor_apply_trim_i32(1, g_mix_m1);
	g_motor_pwm_us[2] = (uint32_t)motor_apply_trim_i32(2, g_mix_m2);
	g_motor_pwm_us[3] = (uint32_t)motor_apply_trim_i32(3, g_mix_m3);
}

static uint32_t pwm_high_us_from_active_us(uint32_t active_us)
{
	if (PWM_ESC_ACTIVE_HIGH) {
		return active_us;
	}

	if (active_us >= MOTOR_PWM_PERIOD_US) {
		return 0;
	}

	return MOTOR_PWM_PERIOD_US - active_us;
}

static void pwm_debug_record_cfg(uint32_t ch, uint32_t active_us)
{
	if (ch >= 4U) {
		return;
	}

	g_pwm_period_us_dbg = MOTOR_PWM_PERIOD_US;
	g_pwm_active_high_dbg = PWM_ESC_ACTIVE_HIGH;
	g_pwm_duty_us_dbg[ch] = active_us;
}

static void pwm_backend_set_cfg_us(uint32_t ch, uint32_t active_us)
{
	uint32_t high_us;

	if (ch >= 4U) {
		return;
	}

	high_us = pwm_high_us_from_active_us(active_us);

	g_pwm_cfg[ch].period = MOTOR_PWM_PERIOD_US * PWM_NS_PER_US;
	g_pwm_cfg[ch].pulse = high_us * PWM_NS_PER_US;
	pwm_debug_record_cfg(ch, active_us);
}

static void pwm_backend_readback_ch(uint32_t ch)
{
	uint32_t hw_ch;
	unsigned long reg_base;

	if (ch >= 4U) {
		return;
	}

	hw_ch = g_pwm_cfg[ch].channel & PWM_MAX_CH;
	reg_base = cvi_pwm[g_pwm_cfg[ch].pwm_id].reg_base;

	g_pwm_period_clk_dbg[ch] = (uint32_t)cvi_pwm_get_period_ch(reg_base, hw_ch);
	g_pwm_duty_clk_dbg[ch] = (uint32_t)cvi_pwm_get_high_period_ch(reg_base, hw_ch);
	g_pwm_start_reg_dbg[ch] = PWM_PWMSTART(reg_base);
	g_pwm_oe_reg_dbg[ch] = PWM_PWM_OE(reg_base);
	g_pwm_active_high_dbg = (uint32_t)cvi_pwm_get_polarity(reg_base, hw_ch);
}

static void pwm_backend_init_once(void)
{
	/*
	 * motor0 = LF -> GP2 -> PWM_7 -> pwm_id 1, channel 3
	 * motor1 = RF -> GP4 -> PWM_5 -> pwm_id 1, channel 1
	 * motor2 = RR -> GP9 -> PWM_4 -> pwm_id 1, channel 0
	 * motor3 = LB -> GP3 -> PWM_6 -> pwm_id 1, channel 2
	 */
	uint32_t i;
	uint32_t pulse_us;

	if (g_pwm_backend_ready) {
		return;
	}

	for (i = 0; i < 4U; i++) {
		g_pwm_cfg[i].pwm_id = g_duo_pwm_id_map[i];
		g_pwm_cfg[i].channel = g_duo_pwm_channel_map[i];

		pulse_us = MOTOR_PWM_SAFE_US;
		pwm_backend_set_cfg_us(i, pulse_us);

		pwm_init(&g_pwm_cfg[i]);
		pwm_set_output_cfg(&g_pwm_cfg[i]);
		pwm_continuous_start(&g_pwm_cfg[i]);
		pwm_backend_readback_ch(i);
	}

	g_pwm_backend_ready = 1;
	g_pwm_backend_err = 0;
	g_pwm_real_enabled = 1;
}

static void esc_update_arm_delay_once(void)
{
	TickType_t now;
	uint32_t elapsed_ms;

	now = xTaskGetTickCount();

	if (g_esc_arm_start_tick == 0) {
		g_esc_arm_start_tick = now;
		g_esc_arm_delay_done = 0;
		g_esc_arm_delay_ms = 0;
		return;
	}

	elapsed_ms = (uint32_t)((now - g_esc_arm_start_tick) * portTICK_PERIOD_MS);
	g_esc_arm_delay_ms = elapsed_ms;

	if (elapsed_ms >= ESC_ARM_DELAY_MS) {
		g_esc_arm_delay_done = 1;
	}
}

static void pwm_backend_write_us(uint32_t ch, uint32_t pulse_us)
{
	if (ch >= 4U) {
		g_pwm_backend_err = PWM_BACKEND_ERR_BAD_CHANNEL;
		return;
	}

	if (pulse_us < MOTOR_PWM_MIN_US) {
		pulse_us = MOTOR_PWM_MIN_US;
	}
	if (pulse_us > MOTOR_PWM_MAX_US) {
		pulse_us = MOTOR_PWM_MAX_US;
	}

	if (!g_motor_armed) {
		pulse_us = MOTOR_PWM_SAFE_US;
	}
	if (!g_motor_output_enabled) {
		pulse_us = MOTOR_PWM_SAFE_US;
	}
	if (!MOTOR_STAGE_ALLOW_THROTTLE_OUTPUT) {
		pulse_us = MOTOR_PWM_SAFE_US;
	}
	if (!g_esc_arm_delay_done) {
		pulse_us = MOTOR_PWM_IDLE_US;
	}

	if (pulse_us > MOTOR_TEST_MAX_US) {
		pulse_us = MOTOR_TEST_MAX_US;
	}

	if (!g_pwm_backend_ready) {
		g_pwm_real_enabled = 0;
		return;
	}

	pwm_backend_set_cfg_us(ch, pulse_us);
	pwm_set_output_cfg(&g_pwm_cfg[ch]);
	pwm_continuous_start(&g_pwm_cfg[ch]);
	pwm_backend_readback_ch(ch);
}

static void motor_apply_outputs(void)
{
	uint32_t i;

	esc_update_arm_delay_once();
	pwm_backend_init_once();

	for (i = 0; i < 4; i++) {
		pwm_backend_write_us(i, g_motor_pwm_us[i]);
	}
}


static void amp_i2c_probe_once(void)
{
	uint8_t a = 0;
	uint8_t b = 0;
	uint8_t c = 0;
	uint8_t who = 0;
	int ret;
	int mpu_read_ok = 0;

	g_i2c_probe_count++;
	g_i2c_last_err = 0;

	g_hmc_data_ok = 0;
	g_hmc_ok = 0;
	g_hmc_bus = 0xff;
	g_hmc_scan_mask = 0;
	g_hmc_id_a = 0;
	g_hmc_id_b = 0;
	g_hmc_id_c = 0;

	g_mpu_whoami = 0;
	g_mpu_ok = 0;
	g_mpu_data_ok = 0;
	g_sensor_ready = 0;
	g_att_ok = 0;
	g_sensor_state_error = SENSOR_STATE_HMC_BAD | SENSOR_STATE_MPU_BAD;

	ret = bb_i2c_read_reg(HMC5883L_ADDR, HMC5883L_ID_A_REG, &a);
	if (ret != 0) {
		g_i2c_last_err = 0xBB010000U | (uint32_t)(-ret & 0xff);
		sensor_update_corrected_once();
		return;
	}

	ret = bb_i2c_read_reg(HMC5883L_ADDR, HMC5883L_ID_B_REG, &b);
	if (ret != 0) {
		g_i2c_last_err = 0xBB020000U | (uint32_t)(-ret & 0xff);
		sensor_update_corrected_once();
		return;
	}

	ret = bb_i2c_read_reg(HMC5883L_ADDR, HMC5883L_ID_C_REG, &c);
	if (ret != 0) {
		g_i2c_last_err = 0xBB030000U | (uint32_t)(-ret & 0xff);
		sensor_update_corrected_once();
		return;
	}

	g_hmc_id_a = a;
	g_hmc_id_b = b;
	g_hmc_id_c = c;
	g_hmc_bus = BB_OK_BUS_MARK;
	g_hmc_scan_mask = 0x80000000U;

	if (a == 0x48 && b == 0x34 && c == 0x33) {
		g_hmc_ok = 1;
		g_i2c_last_err = 0;

		ret = hmc5883l_read_raw_once();
		if (ret != 0) {
			g_i2c_last_err = 0xBB050000U | (uint32_t)(ret & 0xffff);
		}
	} else {
		g_hmc_ok = 0;
		g_hmc_data_ok = 0;
		g_i2c_last_err = 0xBB044400U;
	}

	ret = bb_i2c_read_reg(MPU6050_ADDR0, MPU6050_WHOAMI_REG, &who);
	if (ret == 0 && who == 0x68) {
		g_mpu_whoami = who;
		g_mpu_ok = 1;
		g_mpu_addr_found = MPU6050_ADDR0;

		ret = mpu6050_read_raw_once(MPU6050_ADDR0);
		if (ret != 0) {
			g_i2c_last_err = 0xBB060000U | (uint32_t)(ret & 0xffff);
		} else {
			mpu_read_ok = 1;
			mpu6050_cal_update_once();
			sensor_update_corrected_once();
			attitude_update_debug_once();
			body_axis_update_once();
		}
	} else {
		ret = bb_i2c_read_reg(MPU6050_ADDR1, MPU6050_WHOAMI_REG, &who);
		if (ret == 0 && who == 0x68) {
			g_mpu_whoami = who;
			g_mpu_ok = 1;
			g_mpu_addr_found = MPU6050_ADDR1;

			ret = mpu6050_read_raw_once(MPU6050_ADDR1);
			if (ret != 0) {
				g_i2c_last_err = 0xBB060000U | (uint32_t)(ret & 0xffff);
			} else {
				mpu_read_ok = 1;
				mpu6050_cal_update_once();
				sensor_update_corrected_once();
				attitude_update_debug_once();
				body_axis_update_once();
			}
		}
	}

	if (!mpu_read_ok) {
		g_mpu_data_ok = 0;
		sensor_update_corrected_once();
	}
}

static void ibus_probe_poll_once(void)
{
	static uint16_t ch[IBUS_CHANNELS] = {0};
	static TickType_t last_frame_tick = 0;
	static TickType_t last_print_tick = 0;
	static TickType_t last_dbg_tick = 0;
	static TickType_t last_i2c_tick = 0;
	static TickType_t last_stage3_tick = 0;
	static unsigned int good_frames = 0;
	static unsigned int bad_frames = 0;
	static unsigned int rx_bytes = 0;
	static unsigned int header20 = 0;
	static unsigned int header2040 = 0;
	static unsigned int started = 0;
	static uint8_t last_b = 0;
	static uint8_t prev_b = 0;

	uint8_t b;
	TickType_t now = xTaskGetTickCount();
	int failsafe;
	int budget = 128;

	if (!started) {
		started = 1;
		hal_uart_init(UART3, 115200, 25000000);
	
		g_amp_dbg->magic = AMP_DBG_MAGIC;
		g_amp_dbg->version = AMP_DBG_VERSION;
		g_amp_dbg->seq = 0;
		g_amp_dbg->tick = 0;
		g_amp_dbg->alive = 0;
		g_amp_dbg->mailbox_isr_count = 0;
		g_amp_dbg->mailbox_task_count = 0;
		g_amp_dbg->last_cmd_id = 0;
		g_amp_dbg->last_param_ptr = 0;
		g_amp_dbg->ibus_rx_bytes = 0;
		g_amp_dbg->ibus_good_frames = 0;
		g_amp_dbg->ibus_bad_frames = 0;
		g_amp_dbg->ibus_header20 = 0;
		g_amp_dbg->ibus_header2040 = 0;
		g_amp_dbg->ibus_failsafe = 1;
		g_amp_dbg->uart3_lsr = 0;
		g_amp_dbg->uart3_last_byte = 0;
		g_amp_dbg->shm_phys = 0;
		g_amp_dbg->shm_ready = 0;
		amp_dbg_flush();
	
		printf("[IBUS] poll in CMDQU task, UART3 init 115200, no extra task, no PWM, no arm\n");
	}

	while (budget-- > 0 && uart3_read_byte(&b)) {
		int ret;

		rx_bytes++;

		prev_b = last_b;
		last_b = b;

		if (b == 0x20) {
			header20++;
		}

		if (prev_b == 0x20 && b == 0x40) {
			header2040++;
		}

		ret = ibus_parse_byte(b, ch);

		if (ret == 1) {
			last_frame_tick = now;
			g_rc_last_frame_tick = xTaskGetTickCount();
			good_frames++;
		} else if (ret < 0) {
			bad_frames++;
		}
	}

		if ((now - last_i2c_tick) >= pdMS_TO_TICKS(20)) {
			amp_i2c_probe_once();
			last_i2c_tick = now;
		}

		if ((now - last_stage3_tick) >= pdMS_TO_TICKS(20)) {
			stage3_flight_node_step_legacy_fallback();
			last_stage3_tick = now;
		}

		
		failsafe = (good_frames == 0) ||
           ((now - last_frame_tick) > pdMS_TO_TICKS(300));

		if (!failsafe && good_frames > 0) {
			g_rc_valid = 1;
			g_rc_ch0_us = ch[0];
			g_rc_ch1_us = ch[1];
			g_rc_throttle_us = ch[2];
			g_rc_ch3_us = ch[3];
			g_rc_arm_switch = (ch[5] > RC_ARM_SWITCH_THRESHOLD_US) ? 1 : 0;
		} else {
			g_rc_valid = 0;
			g_rc_ch0_us = 1500;
			g_rc_ch1_us = 1500;
			g_rc_throttle_us = 1000;
			g_rc_ch3_us = 1500;
			g_rc_arm_switch = 0;
		}

		amp_shared_region_poll_once();

		control_update_setpoint_once();
		motor_update_arm_gate();

		if (g_amp_emergency_stop_latched) {
			motor_force_safe(MOTOR_SAFE_FAILSAFE);
		} else if (failsafe) {
			motor_force_safe(MOTOR_SAFE_FAILSAFE);
		} else if (!g_sensor_ready) {
			motor_force_safe(MOTOR_SAFE_SENSOR_NOT_RDY);
		} else if (!g_motor_armed) {
			motor_force_safe(MOTOR_SAFE_NOT_ARMED);
		} else {
			g_motor_output_enabled = 1;
			g_motor_safety_reason = MOTOR_SAFE_OK;
			motor_update_control_outputs();
		}
		motor_apply_outputs();
		amp_shared_region_write_rt_report_once();

		if ((now - last_dbg_tick) >= pdMS_TO_TICKS(20)) {
			g_amp_dbg->magic = AMP_DBG_MAGIC;
			g_amp_dbg->version = AMP_DBG_VERSION;
			g_amp_dbg->seq++;
			g_amp_dbg->tick = (uint32_t)now;
			g_amp_dbg->alive++;
		
			g_amp_dbg->mailbox_isr_count = g_mailbox_isr_count;
			g_amp_dbg->mailbox_task_count = g_mailbox_task_count;
			g_amp_dbg->last_cmd_id = g_last_cmd_id;
			g_amp_dbg->last_param_ptr = g_last_param_ptr;
		
			g_amp_dbg->ibus_rx_bytes = rx_bytes;
			g_amp_dbg->ibus_good_frames = good_frames;
			g_amp_dbg->ibus_bad_frames = bad_frames;
			g_amp_dbg->ibus_header20 = header20;
			g_amp_dbg->ibus_header2040 = header2040;
			g_amp_dbg->ibus_failsafe = failsafe;
			g_amp_dbg->uart3_lsr = g_uart3_last_lsr;
			g_amp_dbg->uart3_last_byte = g_uart3_last_byte;
		
			g_amp_dbg->shm_phys = g_amp_shm_phys;
			g_amp_dbg->shm_ready = g_amp_shm_ready;
		

			g_amp_dbg->i2c_probe_count = g_i2c_probe_count;
			g_amp_dbg->i2c_last_err = g_i2c_last_err;
				
			g_amp_dbg->hmc_addr = HMC5883L_ADDR;
			g_amp_dbg->hmc_id_a = g_hmc_id_a;
			g_amp_dbg->hmc_id_b = g_hmc_id_b;
			g_amp_dbg->hmc_id_c = g_hmc_id_c;
			g_amp_dbg->hmc_ok = g_hmc_ok;
			g_amp_dbg->hmc_bus = g_hmc_bus;
			g_amp_dbg->hmc_scan_mask = g_hmc_scan_mask;
			g_amp_dbg->hmc_data_ok = g_hmc_data_ok;
			g_amp_dbg->hmc_x = g_hmc_x;
			g_amp_dbg->hmc_y = g_hmc_y;
			g_amp_dbg->hmc_z = g_hmc_z;
				
			g_amp_dbg->mpu_addr = g_mpu_addr_found ? g_mpu_addr_found : MPU6050_ADDR0;
			g_amp_dbg->mpu_whoami = g_mpu_whoami;
			g_amp_dbg->mpu_ok = g_mpu_ok;
			g_amp_dbg->mpu_data_ok = g_mpu_data_ok;
			g_amp_dbg->mpu_ax = g_mpu_ax;
			g_amp_dbg->mpu_ay = g_mpu_ay;
			g_amp_dbg->mpu_az = g_mpu_az;
			g_amp_dbg->mpu_gx = g_mpu_gx;
			g_amp_dbg->mpu_gy = g_mpu_gy;
			g_amp_dbg->mpu_gz = g_mpu_gz;
			g_amp_dbg->mpu_cal_count = g_mpu_cal_count;
			g_amp_dbg->mpu_cal_done = g_mpu_cal_done;

			g_amp_dbg->mpu_acc_bias_x = g_mpu_acc_bias_x;
			g_amp_dbg->mpu_acc_bias_y = g_mpu_acc_bias_y;
			g_amp_dbg->mpu_acc_bias_z = g_mpu_acc_bias_z;

			g_amp_dbg->mpu_gyro_bias_x = g_mpu_gyro_bias_x;
			g_amp_dbg->mpu_gyro_bias_y = g_mpu_gyro_bias_y;
			g_amp_dbg->mpu_gyro_bias_z = g_mpu_gyro_bias_z;

			g_amp_dbg->sensor_ready = g_sensor_ready;

			g_amp_dbg->mpu_ax_corr = g_mpu_ax_corr;
			g_amp_dbg->mpu_ay_corr = g_mpu_ay_corr;
			g_amp_dbg->mpu_az_corr = g_mpu_az_corr;

			g_amp_dbg->mpu_gx_corr = g_mpu_gx_corr;
			g_amp_dbg->mpu_gy_corr = g_mpu_gy_corr;
			g_amp_dbg->mpu_gz_corr = g_mpu_gz_corr;

			g_amp_dbg->att_ok = g_att_ok;
			g_amp_dbg->att_roll_cd = g_att_roll_cd;
			g_amp_dbg->att_pitch_cd = g_att_pitch_cd;
			g_amp_dbg->att_yaw_cd = g_att_yaw_cd;

			g_amp_dbg->rate_roll_raw = g_rate_roll_raw;
			g_amp_dbg->rate_pitch_raw = g_rate_pitch_raw;
			g_amp_dbg->rate_yaw_raw = g_rate_yaw_raw;

			g_amp_dbg->level_trim_done = g_level_trim_done;
			g_amp_dbg->level_roll_cd = g_level_roll_cd;
			g_amp_dbg->level_pitch_cd = g_level_pitch_cd;

			g_amp_dbg->body_roll_cd = g_body_roll_cd;
			g_amp_dbg->body_pitch_cd = g_body_pitch_cd;
			g_amp_dbg->body_yaw_cd = g_body_yaw_cd;

			g_amp_dbg->body_rate_roll_raw = g_body_rate_roll_raw;
			g_amp_dbg->body_rate_pitch_raw = g_body_rate_pitch_raw;
			g_amp_dbg->body_rate_yaw_raw = g_body_rate_yaw_raw;

			g_amp_dbg->sp_roll_rate_raw = g_sp_roll_rate_raw;
			g_amp_dbg->sp_pitch_rate_raw = g_sp_pitch_rate_raw;

			g_amp_dbg->sp_roll_cd = g_sp_roll_cd;
			g_amp_dbg->sp_pitch_cd = g_sp_pitch_cd;
			g_amp_dbg->sp_yaw_rate_raw = g_sp_yaw_rate_raw;
			g_amp_dbg->sp_throttle_us = g_sp_throttle_us;

			g_amp_dbg->pid_roll = g_pid_roll;
			g_amp_dbg->pid_pitch = g_pid_pitch;
			g_amp_dbg->pid_yaw = g_pid_yaw;

			g_amp_dbg->mix_m0 = g_mix_m0;
			g_amp_dbg->mix_m1 = g_mix_m1;
			g_amp_dbg->mix_m2 = g_mix_m2;
			g_amp_dbg->mix_m3 = g_mix_m3;

			g_amp_dbg->motor_armed = g_motor_armed;
			g_amp_dbg->motor_output_enabled = g_motor_output_enabled;
			g_amp_dbg->motor_safety_reason = g_motor_safety_reason;
			g_amp_dbg->motor_pwm0 = g_motor_pwm_us[0];
			g_amp_dbg->motor_pwm1 = g_motor_pwm_us[1];
			g_amp_dbg->motor_pwm2 = g_motor_pwm_us[2];
			g_amp_dbg->motor_pwm3 = g_motor_pwm_us[3];
			g_amp_dbg->motor_trim0 = g_motor_trim_us[0];
			g_amp_dbg->motor_trim1 = g_motor_trim_us[1];
			g_amp_dbg->motor_trim2 = g_motor_trim_us[2];
			g_amp_dbg->motor_trim3 = g_motor_trim_us[3];

			g_amp_dbg->pwm_backend_ready = g_pwm_backend_ready;
			g_amp_dbg->pwm_backend_err = g_pwm_backend_err;
			g_amp_dbg->pwm_real_enabled = g_pwm_real_enabled;

			g_amp_dbg->pwm_period_us_dbg = g_pwm_period_us_dbg;
			g_amp_dbg->pwm_active_high_dbg = g_pwm_active_high_dbg;
			g_amp_dbg->pwm_duty0_us_dbg = g_pwm_duty_us_dbg[0];
			g_amp_dbg->pwm_duty1_us_dbg = g_pwm_duty_us_dbg[1];
			g_amp_dbg->pwm_duty2_us_dbg = g_pwm_duty_us_dbg[2];
			g_amp_dbg->pwm_duty3_us_dbg = g_pwm_duty_us_dbg[3];
			g_amp_dbg->pwm_period0_clk_dbg = g_pwm_period_clk_dbg[0];
			g_amp_dbg->pwm_period1_clk_dbg = g_pwm_period_clk_dbg[1];
			g_amp_dbg->pwm_period2_clk_dbg = g_pwm_period_clk_dbg[2];
			g_amp_dbg->pwm_period3_clk_dbg = g_pwm_period_clk_dbg[3];
			g_amp_dbg->pwm_duty0_clk_dbg = g_pwm_duty_clk_dbg[0];
			g_amp_dbg->pwm_duty1_clk_dbg = g_pwm_duty_clk_dbg[1];
			g_amp_dbg->pwm_duty2_clk_dbg = g_pwm_duty_clk_dbg[2];
			g_amp_dbg->pwm_duty3_clk_dbg = g_pwm_duty_clk_dbg[3];
			g_amp_dbg->pwm_start0_dbg = g_pwm_start_reg_dbg[0];
			g_amp_dbg->pwm_start1_dbg = g_pwm_start_reg_dbg[1];
			g_amp_dbg->pwm_start2_dbg = g_pwm_start_reg_dbg[2];
			g_amp_dbg->pwm_start3_dbg = g_pwm_start_reg_dbg[3];
			g_amp_dbg->pwm_oe0_dbg = g_pwm_oe_reg_dbg[0];
			g_amp_dbg->pwm_oe1_dbg = g_pwm_oe_reg_dbg[1];
			g_amp_dbg->pwm_oe2_dbg = g_pwm_oe_reg_dbg[2];
			g_amp_dbg->pwm_oe3_dbg = g_pwm_oe_reg_dbg[3];

			g_amp_dbg->esc_arm_delay_done = g_esc_arm_delay_done;
			g_amp_dbg->esc_arm_delay_ms = g_esc_arm_delay_ms;
			g_amp_dbg->esc_test_max_us = MOTOR_TEST_MAX_US;

			g_amp_dbg->rc_valid = g_rc_valid;
			g_amp_dbg->rc_throttle_us = g_rc_throttle_us;
			g_amp_dbg->rc_arm_switch = g_rc_arm_switch;
			g_amp_dbg->rc_arm_allowed = g_rc_arm_allowed;
			g_amp_dbg->rc_frame_age_ms = g_rc_frame_age_ms;
			g_amp_dbg->rc_seen_arm_low = g_rc_seen_arm_low;
			g_amp_dbg->rc_arm_latched = g_rc_arm_latched;

			g_amp_dbg->motor_min_spin0 = g_motor_min_spin_us[0];
			g_amp_dbg->motor_min_spin1 = g_motor_min_spin_us[1];
			g_amp_dbg->motor_min_spin2 = g_motor_min_spin_us[2];
			g_amp_dbg->motor_min_spin3 = g_motor_min_spin_us[3];

			g_amp_dbg->angle_corr_roll = g_angle_corr_roll;
			g_amp_dbg->angle_corr_pitch = g_angle_corr_pitch;
			g_amp_dbg->rate_corr_roll = g_rate_corr_roll;
			g_amp_dbg->rate_corr_pitch = g_rate_corr_pitch;
			g_amp_dbg->rate_corr_yaw = g_rate_corr_yaw;
			g_amp_dbg->sensor_state_error = g_sensor_state_error;
			g_amp_dbg->control_stage = CONTROL_STAGE_RATE_ANGLE_ASSIST;
			g_amp_dbg->control_min_active_throttle_us = CONTROL_MIN_ACTIVE_THROTTLE_US;
			g_amp_dbg->angle_assist_enable_dbg = ANGLE_ASSIST_ENABLE;
			g_amp_dbg->rate_pid_max_corr_us_dbg = RATE_PID_MAX_CORR_US;
			g_amp_dbg->angle_assist_max_corr_us_dbg = ANGLE_ASSIST_MAX_CORR_US;
			g_amp_dbg->amp_shm_poll_count = g_amp_shm_poll_count;
			g_amp_dbg->amp_emergency_latched = g_amp_emergency_stop_latched;
			g_amp_dbg->amp_config_seq = g_amp_config_seq;
			g_amp_dbg->amp_config_update_count = g_amp_config_update_count;
			g_amp_dbg->amp_rt_report_seq = g_amp_rt_report_seq;
			g_amp_dbg->amp_status_flags = g_amp_status_flags;
			g_amp_dbg->amp_config_deadzone = g_config_deadzone_us;
			g_amp_dbg->amp_config_rate_div = g_config_rate_roll_div;
			g_amp_dbg->amp_config_angle_div = g_config_angle_div;
			g_amp_dbg->stage3_step_count = g_stage3_step_count;
			g_amp_dbg->stage3_actuator_map_ok = g_stage3_actuator_map_ok;
			g_amp_dbg->stage3_external_enabled = g_stage3_external_enabled;

			amp_dbg_flush();
			last_dbg_tick = now;
		}


	if ((now - last_print_tick) >= pdMS_TO_TICKS(1000)) {
		

		printf("[IBUS] fs=%d rx=%d h20=%d h2040=%d good=%d bad=%d last=0x%x ch1=%d ch2=%d ch3=%d ch4=%d\n",
       failsafe,
       (int)rx_bytes,
       (int)header20,
       (int)header2040,
       (int)good_frames,
       (int)bad_frames,
       (unsigned int)last_b,
       (int)ch[0],
       (int)ch[1],
       (int)ch[2],
       (int)ch[3]);

		last_print_tick = now;
	}
}

#endif


void main_create_tasks(void)
{
	u8 i = 0;

#define TASK_INIT(_idx) \
do { \
	gTaskCtx[_idx].queHandle = xQueueCreate(gTaskCtx[_idx].queLength, sizeof(cmdqu_t)); \
	if (gTaskCtx[_idx].queHandle != NULL && gTaskCtx[_idx].runTask != NULL) { \
		xTaskCreate(gTaskCtx[_idx].runTask, gTaskCtx[_idx].name, gTaskCtx[_idx].stack_size, \
			    NULL, gTaskCtx[_idx].priority, NULL); \
	} \
} while(0)

	for (; i < ARRAY_SIZE(gTaskCtx); i++) {
		TASK_INIT(i);
	}
}

void main_cvirtos(void)
{
	printf("create cvi task\n");

	/* Start the tasks and timer running. */
	mailbox_hw_init();
	request_irq(MBOX_INT_C906_2ND, prvQueueISR, 0, "mailbox", (void *)0);
	main_create_tasks();

    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was either insufficient FreeRTOS heap memory available for the idle
    and/or timer tasks to be created, or vTaskStartScheduler() was called from
    User mode.  See the memory management section on the FreeRTOS web site for
    more details on the FreeRTOS heap http://www.freertos.org/a00111.html.  The
    mode from which main() is called is set in the C start up code and must be
    a privileged mode (not user mode). */
    printf("cvi task end\n");
	
	for (;;)
        ;
}


void prvCmdQuRunTask(void *pvParameters)
{
	/* Remove compiler warning about unused parameter. */
	(void)pvParameters;

	cmdqu_t rtos_cmdq;
	cmdqu_t *cmdq;
	cmdqu_t *rtos_cmdqu_t;
	int flags;
	int valid;
	int send_to_cpu = SEND_TO_CPU1;

	/* to compatible code with linux side */
	cmdq = &rtos_cmdq;

	printf("prvCmdQuRunTask run\n");

	for (;;) {
		if (xQueueReceive(gTaskCtx[0].queHandle, &rtos_cmdq, pdMS_TO_TICKS(2)) != pdTRUE) {
#if AMP_ENABLE_IBUS_PROBE
			ibus_probe_poll_once();
#endif
			continue;
		}

		g_mailbox_task_count++;
		g_last_cmd_id = rtos_cmdq.cmd_id;
		g_last_param_ptr = (uint32_t)rtos_cmdq.param_ptr;

#if AMP_ENABLE_IBUS_PROBE
		ibus_probe_poll_once();
#endif

		switch (rtos_cmdq.cmd_id) {
			case CMD_TEST_A:
				//do something
				//send to C906B
				rtos_cmdq.cmd_id = CMD_TEST_A;
				rtos_cmdq.param_ptr = 0x12345678;
				rtos_cmdq.resv.valid.rtos_valid = 1;
				rtos_cmdq.resv.valid.linux_valid = 0;
				printf("recv cmd(%d) from C906B...send [0x%x] to C906B\n", rtos_cmdq.cmd_id, rtos_cmdq.param_ptr);
				goto send_label;
			case CMD_TEST_B:
				//nothing to do
				printf("nothing to do...\n");
				continue;
			case CMD_TEST_C:
				rtos_cmdq.cmd_id = CMD_TEST_C;
				rtos_cmdq.param_ptr = 0x55aa;
				rtos_cmdq.resv.valid.rtos_valid = 1;
				rtos_cmdq.resv.valid.linux_valid = 0;
				printf("recv cmd(%d) from C906B...send [0x%x] to C906B\n", rtos_cmdq.cmd_id, rtos_cmdq.param_ptr);
				goto send_label;
			case CMD_DUO_LED:

				rtos_cmdq.cmd_id = CMD_DUO_LED;
				printf("recv cmd(%d) from C906B, param_ptr [0x%x]\n", rtos_cmdq.cmd_id, rtos_cmdq.param_ptr);
				if (rtos_cmdq.param_ptr == DUO_LED_ON) {
					duo_led_control(1);
				} else {
					duo_led_control(0);
				}
				rtos_cmdq.param_ptr = DUO_LED_DONE;
				rtos_cmdq.resv.valid.rtos_valid = 1;
				rtos_cmdq.resv.valid.linux_valid = 0;
				printf("recv cmd(%d) from C906B...send [0x%x] to C906B\n", rtos_cmdq.cmd_id, rtos_cmdq.param_ptr);
				goto send_label;

			case AMP_CMD_INIT_SHM:
			    /*
			     * Linux sends shared memory physical address once.
			     * Do not read the memory here yet. Just store the address and ACK.
			     */
			    g_amp_shm_phys = (unsigned int)rtos_cmdq.param_ptr;
			    g_amp_shm_ready = 1;

			    /*
			     * Do not printf before ACK. UART printing can delay SEND_WAIT.
			     */
			    // printf("AMP init shm phys=0x%x\n", (unsigned int)g_amp_shm_phys);

			    rtos_cmdq.param_ptr = DUO_LED_DONE;
			    rtos_cmdq.resv.valid.rtos_valid = 1;
			    rtos_cmdq.resv.valid.linux_valid = 0;

			    // printf("AMP init shm ack [0x%x]\n", (unsigned int)rtos_cmdq.param_ptr);

			    goto send_label;


			case 49:  /* AMP Arm / Failsafe / Disarmed */
			case 51:  /* AMP EmergencyStop */
			case 52:
			    rtos_cmdq.param_ptr = DUO_LED_DONE;
			    rtos_cmdq.resv.valid.rtos_valid = 1;
			    rtos_cmdq.resv.valid.linux_valid = 0;
			
			    goto send_label;			
				
			default:
				printf("cmdqu cmd_id %d is not supported\n", rtos_cmdq.cmd_id);
				continue;

send_label:
				/* used to send command to linux*/
				rtos_cmdqu_t = (cmdqu_t *) mailbox_context;
				cmdq->resv.valid.linux_valid = 0;
				cmdq->resv.valid.rtos_valid = 1;

				debug_printf("RTOS_CMDQU_SEND %d\n", send_to_cpu);
				debug_printf("ip_id=%d cmd_id=%d param_ptr=%x\n", cmdq->ip_id, cmdq->cmd_id, (unsigned int)cmdq->param_ptr);
				debug_printf("mailbox_context = %x\n", mailbox_context);
				debug_printf("linux_cmdqu_t = %x\n", rtos_cmdqu_t);
				debug_printf("cmdq->ip_id = %d\n", cmdq->ip_id);
				debug_printf("cmdq->cmd_id = %d\n", cmdq->cmd_id);
				debug_printf("cmdq->block = %d\n", cmdq->block);
				debug_printf("cmdq->para_ptr = %x\n", cmdq->param_ptr);

				drv_spin_lock_irqsave(&mailbox_lock, flags);
				if (flags == MAILBOX_LOCK_FAILED) {
					printf("drv_spin_lock_irqsave failed! ip_id=%d cmd_id=%d\n", cmdq->ip_id, cmdq->cmd_id);
					break;
				}

				for (valid = 0; valid < MAILBOX_MAX_NUM; valid++) {
					if (rtos_cmdqu_t->resv.valid.linux_valid == 0 && rtos_cmdqu_t->resv.valid.rtos_valid == 0) {
						// mailbox buffer context is 4 bytes write access
						int *ptr = (int *)rtos_cmdqu_t;

						*ptr = ((cmdq->ip_id << 0) | (cmdq->cmd_id << 8) | (cmdq->block << 15) |
								(cmdq->resv.valid.linux_valid << 16) |
								(cmdq->resv.valid.rtos_valid << 24));
						rtos_cmdqu_t->param_ptr = cmdq->param_ptr;
						debug_printf("rtos_cmdqu_t->linux_valid = %d\n", rtos_cmdqu_t->resv.valid.linux_valid);
						debug_printf("rtos_cmdqu_t->rtos_valid = %d\n", rtos_cmdqu_t->resv.valid.rtos_valid);
						debug_printf("rtos_cmdqu_t->ip_id =%x %d\n", &rtos_cmdqu_t->ip_id, rtos_cmdqu_t->ip_id);
						debug_printf("rtos_cmdqu_t->cmd_id = %d\n", rtos_cmdqu_t->cmd_id);
						debug_printf("rtos_cmdqu_t->block = %d\n", rtos_cmdqu_t->block);
						debug_printf("rtos_cmdqu_t->param_ptr addr=%x %x\n", &rtos_cmdqu_t->param_ptr, rtos_cmdqu_t->param_ptr);
						debug_printf("*ptr = %x\n", *ptr);
						// clear mailbox
						mbox_reg->cpu_mbox_set[send_to_cpu].cpu_mbox_int_clr.mbox_int_clr = (1 << valid);
						// trigger mailbox valid to rtos
						mbox_reg->cpu_mbox_en[send_to_cpu].mbox_info |= (1 << valid);
						mbox_reg->mbox_set.mbox_set = (1 << valid);
						break;
					}
					rtos_cmdqu_t++;
				}
				drv_spin_unlock_irqrestore(&mailbox_lock, flags);
				if (valid >= MAILBOX_MAX_NUM) {
				    printf("No valid mailbox is available\n");
				    continue;
				}


				break;
		}
	}
}

int prvQueueISR(int irq, void *dev_id)
{
	(void)irq;
	(void)dev_id;

	unsigned char set_val;
	unsigned char valid_val;
	int i;
	cmdqu_t *cmdq;
	BaseType_t YieldRequired = pdFALSE;

	set_val = mbox_reg->cpu_mbox_set[RECEIVE_CPU].cpu_mbox_int_int.mbox_int;

	if (set_val) {
		for(i = 0; i < MAILBOX_MAX_NUM; i++) {
			valid_val = set_val  & (1 << i);

			if (valid_val) {
				cmdqu_t rtos_cmdq;
				cmdq = (cmdqu_t *)(mailbox_context) + i;

				debug_printf("mailbox_context =%x\n", mailbox_context);
				debug_printf("sizeof mailbox_context =%x\n", sizeof(cmdqu_t));
				/* mailbox buffer context is send from linux, clear mailbox interrupt */
				mbox_reg->cpu_mbox_set[RECEIVE_CPU].cpu_mbox_int_clr.mbox_int_clr = valid_val;
				// need to disable enable bit
				mbox_reg->cpu_mbox_en[RECEIVE_CPU].mbox_info &= ~valid_val;

				// copy cmdq context (8 bytes) to buffer ASAP
				*((unsigned long *) &rtos_cmdq) = *((unsigned long *)cmdq);
				/* need to clear mailbox interrupt before clear mailbox buffer */
				*((unsigned long*) cmdq) = 0;

				/* mailbox buffer context is send from linux*/
				if (rtos_cmdq.resv.valid.linux_valid == 1) {
					g_mailbox_isr_count++;
					g_last_cmd_id = rtos_cmdq.cmd_id;
					g_last_param_ptr = (uint32_t)rtos_cmdq.param_ptr;
					debug_printf("cmdq=%x\n", cmdq);
					debug_printf("cmdq->ip_id =%d\n", rtos_cmdq.ip_id);
					debug_printf("cmdq->cmd_id =%d\n", rtos_cmdq.cmd_id);
					debug_printf("cmdq->param_ptr =%x\n", rtos_cmdq.param_ptr);
					debug_printf("cmdq->block =%x\n", rtos_cmdq.block);
					debug_printf("cmdq->linux_valid =%d\n", rtos_cmdq.resv.valid.linux_valid);
					debug_printf("cmdq->rtos_valid =%x\n", rtos_cmdq.resv.valid.rtos_valid);

					xQueueSendFromISR(gTaskCtx[0].queHandle, &rtos_cmdq, &YieldRequired);
				} else
					printf("rtos cmdq is not valid %d, ip=%d , cmd=%d\n",
						rtos_cmdq.resv.valid.rtos_valid, rtos_cmdq.ip_id, rtos_cmdq.cmd_id);
			}
		}
	}

	portYIELD_FROM_ISR(YieldRequired);
	return 0;
}
