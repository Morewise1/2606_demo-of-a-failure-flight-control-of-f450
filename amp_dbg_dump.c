#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>

#define AMP_DBG_SHM_ADDR 0x8FFFF000UL
#define AMP_DBG_SIZE 4096UL

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

int main(void)
{
    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    void *p = mmap(NULL, AMP_DBG_SIZE, PROT_READ, MAP_SHARED, fd, AMP_DBG_SHM_ADDR);
    if (p == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }

    volatile amp_dbg_shm_t *s = (volatile amp_dbg_shm_t *)p;

    printf("motor order: m0=LF/GP2 m1=RF/GP4 m2=RR/GP9 m3=LB/GP3\n");
    printf("motor_safe: 0=OK 1=FAILSAFE 2=SENSOR_NOT_READY 3=NOT_ARMED 4=THROTTLE_HIGH\n");

    for (;;) {
        const char *safe_name = "UNKNOWN";
        const char *sensor_warn = "";
        const char *att_warn = "";

        switch (s->motor_safety_reason) {
        case 0:
            safe_name = "OK";
            break;
        case 1:
            safe_name = "FAILSAFE";
            break;
        case 2:
            safe_name = "SENSOR_NOT_READY";
            break;
        case 3:
            safe_name = "NOT_ARMED";
            break;
        case 4:
            safe_name = "THROTTLE_HIGH";
            break;
        default:
            safe_name = "UNKNOWN";
            break;
        }

        if (s->sensor_ready && (!s->mpu_ok || !s->mpu_data_ok)) {
            sensor_warn = " WARN_SENSOR_STATE";
        }

        if (s->att_ok && !s->sensor_ready) {
            att_warn = " WARN_ATT_STATE";
        }

        printf("CORE magic=%08x ver=%u seq=%u tick=%u alive=%u "
               "isr=%u task=%u cmd=%u param=%08x shm_phys=%08x shm_ready=%u "
               "IBUS_RC rx=%u good=%u bad=%u h20=%u h2040=%u fs=%u "
               "lsr=%08x last=%02x rc_valid=%u thr=%u arm_sw=%u "
               "arm_allowed=%u rc_age=%u arm_seen_low=%u arm_latched=%u "
               "SENSOR i2c_cnt=%u i2c_err=%08x "
               "hmc_addr=%02x hmc_id=%02x,%02x,%02x hmc_ok=%u "
               "hmc_bus=%u hmc_mask=%x hmc_data_ok=%u hmc_xyz=%d,%d,%d "
               "mpu_addr=%02x mpu_whoami=%02x mpu_ok=%u "
               "mpu_data_ok=%u acc=%d,%d,%d gyro=%d,%d,%d "
               "cal=%u/%u acc_bias=%d,%d,%d gyro_bias=%d,%d,%d "
               "sensor_ready=%u acc_corr=%d,%d,%d gyro_corr=%d,%d,%d "
               "ATT_BODY att_ok=%u att=%d,%d,%d rate_raw=%d,%d,%d "
               "level=%u,%d,%d body=%d,%d,%d body_rate=%d,%d,%d "
               "SETPOINT control_stage=%u active_thr_min=%u angle_en=%u "
               "rate_max=%u angle_max=%u sp_rate=%d,%d sp=%d,%d,%d,%u "
               "CORR_PID rate_corr=%d,%d,%d angle_corr=%d,%d pid=%d,%d,%d "
               "MIX mix=%d,%d,%d,%d "
               "MOTOR motor_armed=%u motor_out=%u motor_safe=%u(%s) "
               "motor=%u,%u,%u,%u trim=%d,%d,%d,%d "
               "min_spin=%u,%u,%u,%u esc_delay=%u/%u esc_max=%u "
               "PWM pwm_ready=%u pwm_err=%u pwm_real=%u "
               "pwm_dbg_period=%u pwm_active_high=%u pwm_duty_us=%u,%u,%u,%u "
               "pwm_period_clk=%u,%u,%u,%u pwm_duty_clk=%u,%u,%u,%u "
               "pwm_start=%u,%u,%u,%u pwm_oe=%u,%u,%u,%u "
               "SAFETY sensor_state_error=%u "
               "AMP amp_shm_poll=%u amp_emerg=%u amp_cfg_seq=%u "
               "amp_cfg_updates=%u amp_rt_seq=%u amp_flags=%08x "
               "amp_cfg_deadzone=%u amp_cfg_rate_div=%u amp_cfg_angle_div=%u "
               "stage3_step=%u stage3_map_ok=%u stage3_external=%u%s%s\n",
               s->magic,
               s->version,
               s->seq,
               s->tick,
               s->alive,
               s->mailbox_isr_count,
               s->mailbox_task_count,
               s->last_cmd_id,
               s->last_param_ptr,
               s->shm_phys,
               s->shm_ready,
               s->ibus_rx_bytes,
               s->ibus_good_frames,
               s->ibus_bad_frames,
               s->ibus_header20,
               s->ibus_header2040,
               s->ibus_failsafe,
               s->uart3_lsr,
               s->uart3_last_byte & 0xff,
               s->rc_valid,
               s->rc_throttle_us,
               s->rc_arm_switch,
               s->rc_arm_allowed,
               s->rc_frame_age_ms,
               s->rc_seen_arm_low,
               s->rc_arm_latched,
               s->i2c_probe_count,
               s->i2c_last_err,
               s->hmc_addr,
               s->hmc_id_a,
               s->hmc_id_b,
               s->hmc_id_c,
               s->hmc_ok,
               s->hmc_bus,
               s->hmc_scan_mask,
               s->hmc_data_ok,
               s->hmc_x,
               s->hmc_y,
               s->hmc_z,
               s->mpu_addr,
               s->mpu_whoami,
               s->mpu_ok,
               s->mpu_data_ok,
               s->mpu_ax,
               s->mpu_ay,
               s->mpu_az,
               s->mpu_gx,
               s->mpu_gy,
               s->mpu_gz,
               s->mpu_cal_count,
               s->mpu_cal_done,
               s->mpu_acc_bias_x,
               s->mpu_acc_bias_y,
               s->mpu_acc_bias_z,
               s->mpu_gyro_bias_x,
               s->mpu_gyro_bias_y,
               s->mpu_gyro_bias_z,
               s->sensor_ready,
               s->mpu_ax_corr,
               s->mpu_ay_corr,
               s->mpu_az_corr,
               s->mpu_gx_corr,
               s->mpu_gy_corr,
               s->mpu_gz_corr,
               s->att_ok,
               s->att_roll_cd,
               s->att_pitch_cd,
               s->att_yaw_cd,
               s->rate_roll_raw,
               s->rate_pitch_raw,
               s->rate_yaw_raw,
               s->level_trim_done,
               s->level_roll_cd,
               s->level_pitch_cd,
               s->body_roll_cd,
               s->body_pitch_cd,
               s->body_yaw_cd,
               s->body_rate_roll_raw,
               s->body_rate_pitch_raw,
               s->body_rate_yaw_raw,
               s->control_stage,
               s->control_min_active_throttle_us,
               s->angle_assist_enable_dbg,
               s->rate_pid_max_corr_us_dbg,
               s->angle_assist_max_corr_us_dbg,
               s->sp_roll_rate_raw,
               s->sp_pitch_rate_raw,
               s->sp_roll_cd,
               s->sp_pitch_cd,
               s->sp_yaw_rate_raw,
               s->sp_throttle_us,
               s->rate_corr_roll,
               s->rate_corr_pitch,
               s->rate_corr_yaw,
               s->angle_corr_roll,
               s->angle_corr_pitch,
               s->pid_roll,
               s->pid_pitch,
               s->pid_yaw,
               s->mix_m0,
               s->mix_m1,
               s->mix_m2,
               s->mix_m3,
               s->motor_armed,
               s->motor_output_enabled,
               s->motor_safety_reason,
               safe_name,
               s->motor_pwm0,
               s->motor_pwm1,
               s->motor_pwm2,
               s->motor_pwm3,
               s->motor_trim0,
               s->motor_trim1,
               s->motor_trim2,
               s->motor_trim3,
               s->motor_min_spin0,
               s->motor_min_spin1,
               s->motor_min_spin2,
               s->motor_min_spin3,
               s->esc_arm_delay_done,
               s->esc_arm_delay_ms,
               s->esc_test_max_us,
               s->pwm_backend_ready,
               s->pwm_backend_err,
               s->pwm_real_enabled,
               s->pwm_period_us_dbg,
               s->pwm_active_high_dbg,
               s->pwm_duty0_us_dbg,
               s->pwm_duty1_us_dbg,
               s->pwm_duty2_us_dbg,
               s->pwm_duty3_us_dbg,
               s->pwm_period0_clk_dbg,
               s->pwm_period1_clk_dbg,
               s->pwm_period2_clk_dbg,
               s->pwm_period3_clk_dbg,
               s->pwm_duty0_clk_dbg,
               s->pwm_duty1_clk_dbg,
               s->pwm_duty2_clk_dbg,
               s->pwm_duty3_clk_dbg,
               s->pwm_start0_dbg,
               s->pwm_start1_dbg,
               s->pwm_start2_dbg,
               s->pwm_start3_dbg,
               s->pwm_oe0_dbg,
               s->pwm_oe1_dbg,
               s->pwm_oe2_dbg,
               s->pwm_oe3_dbg,
               s->sensor_state_error,
               s->amp_shm_poll_count,
               s->amp_emergency_latched,
               s->amp_config_seq,
               s->amp_config_update_count,
               s->amp_rt_report_seq,
               s->amp_status_flags,
               s->amp_config_deadzone,
               s->amp_config_rate_div,
               s->amp_config_angle_div,
               s->stage3_step_count,
               s->stage3_actuator_map_ok,
               s->stage3_external_enabled,
               sensor_warn,
               att_warn);

        sleep(1);
    }

    return 0;
}
