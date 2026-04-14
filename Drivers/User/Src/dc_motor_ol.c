#include "dc_motor_ol.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

#define DC_MOTOR_COUNT 4U

#define DC_MOTOR_PID_KP                     0.08f
#define DC_MOTOR_PID_KI                     0.015f
#define DC_MOTOR_PID_KD                     0.0f
#define DC_MOTOR_PID_INT_LIMIT              8000.0f
#define DC_MOTOR_OUTPUT_LIMIT               100.0f

static TIM_HandleTypeDef *s_encoder_tim[DC_MOTOR_COUNT] = {
    &htim2,
    &htim3,
    &htim4,
    &htim5
};

static const uint32_t s_pwm_channel[DC_MOTOR_COUNT] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4
};

static GPIO_TypeDef *s_dir_port[DC_MOTOR_COUNT] = {
    M1_PH_GPIO_Port,
    M2_PH_GPIO_Port,
    M3_PH_GPIO_Port,
    M4_PH_GPIO_Port
};

static const uint16_t s_dir_pin[DC_MOTOR_COUNT] = {
    M1_PH_Pin,
    M2_PH_Pin,
    M3_PH_Pin,
    M4_PH_Pin
};

static uint32_t s_encoder_prev_cnt[DC_MOTOR_COUNT] = {0, 0, 0, 0};
static int16_t s_target_speed_percent[DC_MOTOR_COUNT] = {0, 0, 0, 0};
static int32_t s_target_rpm[DC_MOTOR_COUNT] = {0, 0, 0, 0};
static volatile int32_t s_measured_rpm[DC_MOTOR_COUNT] = {0, 0, 0, 0};
static float s_pid_integral[DC_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float s_pid_prev_error[DC_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t s_motor_init_done = 0U;

static int16_t dc_motor_clamp_speed(int16_t speed)
{
    if (speed > 100)
    {
        return 100;
    }

    if (speed < -100)
    {
        return -100;
    }

    return speed;
}

static uint32_t dc_motor_abs_u32(int16_t value)
{
    if (value < 0)
    {
        return (uint32_t)(-value);
    }

    return (uint32_t)value;
}

static float dc_motor_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static int32_t dc_motor_speed_to_target_rpm(int16_t speed_percent)
{
    return ((int32_t)speed_percent * (int32_t)DCMOTOR_OL_MAX_TARGET_RPM) / 100;
}

static int32_t dc_motor_counts_to_rpm(int32_t counts_10ms)
{
    return (int32_t)(((int64_t)counts_10ms * 60000LL) /
        ((int64_t)DCMOTOR_OL_ENCODER_COUNTS_PER_REV * (int64_t)DCMOTOR_OL_SAMPLE_PERIOD_MS));
}

static int16_t dc_motor_pid_update(uint8_t index, int32_t target_rpm, int32_t measured_rpm)
{
    float ff;
    float error;
    float p_term;
    float i_term;
    float d_term;
    float output;
    float unsat_output;
    uint8_t integrate_enable;

    ff = ((float)target_rpm * 100.0f) / (float)DCMOTOR_OL_NO_LOAD_RPM;
    error = (float)(target_rpm - measured_rpm);

    p_term = DC_MOTOR_PID_KP * error;
    i_term = DC_MOTOR_PID_KI * s_pid_integral[index];
    d_term = DC_MOTOR_PID_KD * (error - s_pid_prev_error[index]);

    unsat_output = ff + p_term + i_term + d_term;
    integrate_enable = 1U;

    if ((unsat_output > DC_MOTOR_OUTPUT_LIMIT) && (error > 0.0f))
    {
        integrate_enable = 0U;
    }
    else if ((unsat_output < -DC_MOTOR_OUTPUT_LIMIT) && (error < 0.0f))
    {
        integrate_enable = 0U;
    }

    if (integrate_enable != 0U)
    {
        s_pid_integral[index] += error;
        s_pid_integral[index] = dc_motor_clampf(s_pid_integral[index], -DC_MOTOR_PID_INT_LIMIT, DC_MOTOR_PID_INT_LIMIT);
        i_term = DC_MOTOR_PID_KI * s_pid_integral[index];
    }

    s_pid_prev_error[index] = error;

    output = ff + p_term + i_term + d_term;
    output = dc_motor_clampf(output, -DC_MOTOR_OUTPUT_LIMIT, DC_MOTOR_OUTPUT_LIMIT);

    return (int16_t)output;
}

static void dc_motor_apply_output(uint8_t index, int16_t speed)
{
    uint32_t arr;
    uint32_t duty_permille;
    uint32_t compare;

    speed = dc_motor_clamp_speed(speed);

    if (speed >= 0)
    {
        HAL_GPIO_WritePin(s_dir_port[index], s_dir_pin[index], GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(s_dir_port[index], s_dir_pin[index], GPIO_PIN_RESET);
    }

    duty_permille = dc_motor_abs_u32(speed) * 10U;
    if (duty_permille != 0U)
    {
        duty_permille--;
    }
    arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    compare = (arr * duty_permille) / 999U;
    if (compare > arr)
    {
        compare = arr;
    }

    __HAL_TIM_SET_COMPARE(&htim1, s_pwm_channel[index], compare);
}

static int32_t dc_motor_calc_delta(uint32_t curr, uint32_t prev, uint32_t arr)
{
    if (arr <= 0xFFFFU)
    {
        return (int32_t)(int16_t)((uint16_t)curr - (uint16_t)prev);
    }

    return (int32_t)(curr - prev);
}

HAL_StatusTypeDef DCMotor_OL_Init(void)
{
    uint8_t i;

    for (i = 0U; i < DC_MOTOR_COUNT; ++i)
    {
        if (HAL_TIM_PWM_Start(&htim1, s_pwm_channel[i]) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    for (i = 0U; i < DC_MOTOR_COUNT; ++i)
    {
        if (HAL_TIM_Encoder_Start(s_encoder_tim[i], TIM_CHANNEL_ALL) != HAL_OK)
        {
            return HAL_ERROR;
        }

        s_encoder_prev_cnt[i] = __HAL_TIM_GET_COUNTER(s_encoder_tim[i]);
        s_target_speed_percent[i] = 0;
        s_target_rpm[i] = 0;
        s_measured_rpm[i] = 0;
        s_pid_integral[i] = 0.0f;
        s_pid_prev_error[i] = 0.0f;
    }

    DCMotor_OL_StopAll();
    s_motor_init_done = 1U;
    return HAL_OK;
}

void DCMotor_OL_SetSpeed(uint8_t motor_index, int16_t speed_percent)
{
    uint8_t idx;
    int16_t prev_speed_percent;

    if (s_motor_init_done == 0U)
    {
        return;
    }

    if ((motor_index == 0U) || (motor_index > DC_MOTOR_COUNT))
    {
        return;
    }

    idx = (uint8_t)(motor_index - 1U);
    speed_percent = dc_motor_clamp_speed(speed_percent);
    prev_speed_percent = s_target_speed_percent[idx];

    if (speed_percent != prev_speed_percent)
    {
        /* Clear control memory on setpoint step to avoid carrying old integral bias. */
        s_pid_integral[idx] = 0.0f;
        s_pid_prev_error[idx] = 0.0f;
    }

    s_target_speed_percent[idx] = speed_percent;
    s_target_rpm[idx] = dc_motor_speed_to_target_rpm(speed_percent);

    if (speed_percent == 0)
    {
        s_pid_integral[idx] = 0.0f;
        s_pid_prev_error[idx] = 0.0f;
        dc_motor_apply_output(idx, 0);
    }
}

void DCMotor_OL_StopAll(void)
{
    uint8_t i;

    for (i = 0U; i < DC_MOTOR_COUNT; ++i)
    {
        s_target_speed_percent[i] = 0;
        s_target_rpm[i] = 0;
        s_measured_rpm[i] = 0;
        s_pid_integral[i] = 0.0f;
        s_pid_prev_error[i] = 0.0f;
        dc_motor_apply_output(i, 0);
    }
}

void DCMotor_OL_Tick10ms(void)
{
    uint8_t i;

    if (s_motor_init_done == 0U)
    {
        return;
    }

    for (i = 0U; i < DC_MOTOR_COUNT; ++i)
    {
        uint32_t curr;
        uint32_t prev;
        uint32_t arr;
        int32_t delta;
        int32_t measured_rpm;
        int16_t output_percent;

        curr = __HAL_TIM_GET_COUNTER(s_encoder_tim[i]);
        prev = s_encoder_prev_cnt[i];
        arr = __HAL_TIM_GET_AUTORELOAD(s_encoder_tim[i]);

        delta = dc_motor_calc_delta(curr, prev, arr);
        s_encoder_prev_cnt[i] = curr;

        measured_rpm = dc_motor_counts_to_rpm(delta);
        s_measured_rpm[i] = measured_rpm;

        if (s_target_speed_percent[i] == 0)
        {
            dc_motor_apply_output(i, 0);
            continue;
        }

        output_percent = dc_motor_pid_update(i, s_target_rpm[i], measured_rpm);
        dc_motor_apply_output(i, output_percent);
    }
}

int32_t DCMotor_OL_GetSpeedRpm(uint8_t motor_index)
{
    if ((motor_index == 0U) || (motor_index > DC_MOTOR_COUNT))
    {
        return 0;
    }

    return s_measured_rpm[motor_index - 1U];
}