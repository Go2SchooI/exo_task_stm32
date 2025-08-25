#include "exo_controller.h"

exo_controller_t exo_controller = {0};

float TargetVelocity = 0, TargetTorque = 0;
float TargetAngle1 = 0, TargetAngle2 = 0,
      TargetAngle3 = 0, TargetAngle4 = 0; // 目标角度
float a = -35.0f, b = 0.5f, c = -40.0f;
float a1 = 60.0f, c1 = 70.0f;

static void set_exo_mode(void);
static void get_exo_ctrl_value(void);
static void limit_exo_ctrl_value(void);
static void set_exo_control(void);
static void send_exo_motor_current(void);

void exo_init(void)
{
  // PID初始化
  PID_Init(&exo_controller.xzy_shoulder.lk_motor.PID_Velocity, 2000, 1000, 0, 1.5, 0.1, 0, 0, 0, 0, 0, 0, Integral_Limit | Trapezoid_Intergral | OutputFilter);
  PID_Init(&exo_controller.xzy_shoulder.lk_motor.PID_Angle, 8600, 4000, 0, 30, 0, 2, 0, 0, 0, 0, 0, Integral_Limit | Trapezoid_Intergral | OutputFilter);
  exo_controller.xzy_shoulder.lk_motor.max_out = 0;
  exo_controller.xzy_shoulder.lk_motor.zero_offset = 0;

  PID_Init(&exo_controller.xzy_shoulder.dm_motor[0].PID_Velocity, 18, 10, 0, 0.7, 0, 0, 0, 0, 0, 0, 0, Integral_Limit | Trapezoid_Intergral | OutputFilter);
  PID_Init(&exo_controller.xzy_shoulder.dm_motor[0].PID_Angle, 50, 5, 0, 0.4, 0.05, 0.01, 0, 0, 0, 0, 0, Integral_Limit | Trapezoid_Intergral | OutputFilter);
  exo_controller.xzy_shoulder.dm_motor[0].max_out = 0;
  exo_controller.xzy_shoulder.dm_motor[0].zero_offset = 31760;

  PID_Init(&exo_controller.xzy_shoulder.dm_motor[1].PID_Velocity, 18, 10, 0, 0.7, 0, 0, 0, 0, 0, 0, 0, Integral_Limit | Trapezoid_Intergral | OutputFilter);
  PID_Init(&exo_controller.xzy_shoulder.dm_motor[1].PID_Angle, 50, 5, 0, 0.4, 0.05, 0.01, 0, 0, 0, 0, 0, Integral_Limit | Trapezoid_Intergral | OutputFilter);
  exo_controller.xzy_shoulder.dm_motor[1].max_out = 0;
  exo_controller.xzy_shoulder.dm_motor[1].zero_offset = 33620;

  PID_Init(&exo_controller.elbow.dm_motor.PID_Velocity, 18, 10, 0, 0.7, 0, 0, 0, 0, 0, 0, 0, Integral_Limit | Trapezoid_Intergral | OutputFilter);
  PID_Init(&exo_controller.elbow.dm_motor.PID_Angle, 50, 5, 0, 0.4, 0.05, 0.01, 0, 0, 0, 0, 0, Integral_Limit | Trapezoid_Intergral | OutputFilter);
  exo_controller.elbow.dm_motor.max_out = 0;
  exo_controller.elbow.dm_motor.zero_offset = 31706;

  exo_controller.xzy_shoulder.lk_motor.reduction_ratio = 1.0f;
  exo_controller.xzy_shoulder.dm_motor[0].reduction_ratio = 1.0f;
  exo_controller.xzy_shoulder.dm_motor[1].reduction_ratio = 1.0f;
  exo_controller.elbow.dm_motor.reduction_ratio = 1.0f;

  exo_controller.xzy_shoulder.SSM.alpha = 32.0f;
  exo_controller.xzy_shoulder.SSM.offset_Z = 22.5f;

  exo_dynamics_params_init(&exo_controller.dynamics_params);

  DM_CANx_SendStdData(&hcan1, 0x01, ENABLE_MOTOR, 8);
  HAL_Delay(2);
  DM_CANx_SendStdData(&hcan1, 0x02, ENABLE_MOTOR, 8);
  HAL_Delay(2);
  DM_CANx_SendStdData(&hcan1, 0x03, ENABLE_MOTOR, 8);
  HAL_Delay(2);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 启动PWM
}

void exo_task(void)
{
  exo_controller.dt = DWT_GetDeltaT(&exo_controller.DWT_count);
  exo_controller.t += exo_controller.dt;

  set_exo_mode();
  get_exo_ctrl_value();
  set_exo_control();
  limit_exo_ctrl_value();
  send_exo_motor_current();
}

static void set_exo_mode(void)
{
  static uint32_t reset_count = 0;
  static float reset_timestamp = 0;
  static uint8_t pin_state = 0, last_pin_state = 0;

  pin_state = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
  // press the key
  if (pin_state == GPIO_PIN_RESET)
  {
    reset_count++;
    reset_timestamp = exo_controller.t;
  }

  if (last_pin_state == GPIO_PIN_SET && pin_state == GPIO_PIN_RESET)
  {
    reset_count = 0;
    exo_controller.mode = SILENCE_MODE;
  }

  if (reset_count > 1000)
  {
    if ((exo_controller.t - reset_timestamp) < 2.0f)
      TIM_Set_PWM(&htim4, TIM_CHANNEL_3, 2000);
    else
      TIM_Set_PWM(&htim4, TIM_CHANNEL_3, 0);

    if ((exo_controller.t - reset_timestamp) > 5.0f)
      exo_controller.mode = ANGLE_MODE;
  }

  if (exo_controller.last_mode != exo_controller.mode)
    exo_controller.mode_change_timestamp = exo_controller.t;

  last_pin_state = pin_state;
  exo_controller.last_mode = exo_controller.mode;
}

static void get_exo_ctrl_value(void)
{
  SSM_inner_forward_kinematics(exo_controller.xzy_shoulder.SSM.theta,
                               exo_controller.xzy_shoulder.SSM.alpha, exo_controller.xzy_shoulder.SSM.phi);
  SSM_2_shoulder_angle(exo_controller.xzy_shoulder.SSM.phi,
                       exo_controller.xzy_shoulder.SSM.offset_Z, exo_controller.xzy_shoulder.human_xzy_angle);

  switch (exo_controller.mode)
  {
  case SILENCE_MODE:
    TargetAngle1 = exo_controller.xzy_shoulder.dm_motor[0].angle_in_degree;
    TargetAngle2 = exo_controller.xzy_shoulder.dm_motor[1].angle_in_degree;
    TargetAngle3 = exo_controller.xzy_shoulder.lk_motor.angle_in_degree;
    TargetAngle4 = exo_controller.elbow.dm_motor.angle_in_degree;
    break;

  case ANGLE_MODE:
    if (exo_controller.t - exo_controller.mode_change_timestamp < 1)
    {
      TargetAngle1 = exo_controller.xzy_shoulder.dm_motor[0].angle_in_degree;
      TargetAngle2 = exo_controller.xzy_shoulder.dm_motor[1].angle_in_degree;
      TargetAngle3 = exo_controller.xzy_shoulder.lk_motor.angle_in_degree;
      TargetAngle4 = exo_controller.elbow.dm_motor.angle_in_degree;
    }
    else if (exo_controller.debug_mode == 5)
    {
      TargetAngle4 = 50 * sin(b * exo_controller.t + PI / 3) + 60;
    }
    else if (exo_controller.debug_mode == 6)
    {
      TargetAngle3 = a * sin(b * exo_controller.t + PI) + c;
      TargetAngle4 = 50 * sin(b * exo_controller.t + PI / 3) + 60;
    }
    else if (exo_controller.debug_mode == 7)
    {
      TargetAngle1 = -20 * sin(b * exo_controller.t + PI) - 25;
      TargetAngle2 = 20 * sin(b * exo_controller.t + PI / 3) + 25;
    }
    else if (exo_controller.debug_mode == 8) // ab
    {
      TargetAngle1 = -30 * sin(b * exo_controller.t + PI) - 65;
      TargetAngle2 = -30 * sin(b * exo_controller.t + PI) - 20;
    }
    else if (exo_controller.debug_mode == 9)
    {
      TargetAngle1 = -17 * sin(b * exo_controller.t + PI) - 20;
      TargetAngle2 = 17 * sin(b * exo_controller.t + PI) + 20;
    }

  case NORMAL_MODE:
    shoulder_angle_2_SSM(exo_controller.xzy_shoulder.human_ctrl_xzy_angle,
                         exo_controller.xzy_shoulder.SSM.offset_Z, exo_controller.xzy_shoulder.SSM.SSM_xzy_angle);
    SSM_inner_inv_kinematics(exo_controller.xzy_shoulder.SSM.SSM_xzy_angle,
                             exo_controller.xzy_shoulder.SSM.alpha, exo_controller.xzy_shoulder.SSM.theta);
    break;
  }
}

static void limit_exo_ctrl_value(void)
{
  if (fabsf(TargetAngle1 - TargetAngle2) > MAX_SSM_MOTOR_ANFLE_DIFFERENCE)
  {
    TargetAngle1 = exo_controller.xzy_shoulder.dm_motor[0].angle_in_degree;
    TargetAngle2 = exo_controller.xzy_shoulder.dm_motor[1].angle_in_degree;
  }
  TargetAngle1 = float_constrain(TargetAngle1, SSM_MOTOR1_MIN, SSM_MOTOR1_MAX);
  TargetAngle2 = float_constrain(TargetAngle2, SSM_MOTOR2_MIN, SSM_MOTOR2_MAX);
  TargetAngle3 = float_constrain(TargetAngle3, SSM_Y_ANGLE_MIN, SSM_Y_ANGLE_MAX);
}

static void set_exo_control(void)
{
  float VelocityLoopInput, TorqueLoopInput;

  switch (exo_controller.mode)
  {
  case SILENCE_MODE:
    exo_controller.xzy_shoulder.lk_motor.Output = 0;
    exo_controller.xzy_shoulder.dm_motor[0].Output = 0;
    exo_controller.xzy_shoulder.dm_motor[1].Output = 0;
    exo_controller.elbow.dm_motor.Output = 0;
    break;

  case ANGLE_MODE:
    calculate_dynamics_feedforward(&exo_controller.dynamics_params,
                                   exo_controller.ctrl_human_value.q, exo_controller.ctrl_human_value.q_dot,
                                   exo_controller.ctrl_human_value.q_ddot, exo_controller.feedforward_output);

    /* -------------------------------- lk motor -------------------------------- */
    /* ---------------------------- shoulder motor 3 ---------------------------- */
    if (exo_controller.debug_mode == 6 || exo_controller.debug_mode == 4)
      PID_Calculate(&exo_controller.xzy_shoulder.lk_motor.PID_Angle, exo_controller.xzy_shoulder.INS_shoulder.xzy_order_angle[2], TargetAngle3);
    else
      PID_Calculate(&exo_controller.xzy_shoulder.lk_motor.PID_Angle, exo_controller.xzy_shoulder.lk_motor.angle_in_degree, TargetAngle3);
    VelocityLoopInput = float_constrain(exo_controller.xzy_shoulder.lk_motor.PID_Angle.Output,
                                        -exo_controller.xzy_shoulder.lk_motor.PID_Angle.MaxOut, exo_controller.xzy_shoulder.lk_motor.PID_Angle.MaxOut);
    PID_Calculate(&exo_controller.xzy_shoulder.lk_motor.PID_Velocity, exo_controller.xzy_shoulder.lk_motor.velocity_in_rpm, VelocityLoopInput);
    TorqueLoopInput = float_constrain(exo_controller.xzy_shoulder.lk_motor.PID_Velocity.Output,
                                      -exo_controller.xzy_shoulder.lk_motor.PID_Velocity.MaxOut, exo_controller.xzy_shoulder.lk_motor.PID_Velocity.MaxOut);
    exo_controller.xzy_shoulder.lk_motor.Output = float_constrain(TorqueLoopInput, -exo_controller.xzy_shoulder.lk_motor.max_out, exo_controller.xzy_shoulder.lk_motor.max_out);

    /* --------------------------------- dm motor -------------------------------- */
    /* ---------------------------- shoulder motor 1 ---------------------------- */
    PID_Calculate(&exo_controller.xzy_shoulder.dm_motor[0].PID_Angle, exo_controller.xzy_shoulder.dm_motor[0].angle_in_degree, TargetAngle1);
    VelocityLoopInput = float_constrain(exo_controller.xzy_shoulder.dm_motor[0].PID_Angle.Output,
                                        -exo_controller.xzy_shoulder.dm_motor[0].PID_Angle.MaxOut, exo_controller.xzy_shoulder.dm_motor[0].PID_Angle.MaxOut);

    PID_Calculate(&exo_controller.xzy_shoulder.dm_motor[0].PID_Velocity, exo_controller.xzy_shoulder.dm_motor[0].velocity_in_radps, VelocityLoopInput);
    TorqueLoopInput = float_constrain(exo_controller.xzy_shoulder.dm_motor[0].PID_Velocity.Output,
                                      -exo_controller.xzy_shoulder.dm_motor[0].PID_Velocity.MaxOut, exo_controller.xzy_shoulder.dm_motor[0].PID_Velocity.MaxOut);

    exo_controller.xzy_shoulder.dm_motor[0].Output = float_constrain(TorqueLoopInput, -exo_controller.xzy_shoulder.dm_motor[0].max_out, exo_controller.xzy_shoulder.dm_motor[0].max_out);

    /* ---------------------------- shoulder motor 1 ---------------------------- */
    PID_Calculate(&exo_controller.xzy_shoulder.dm_motor[1].PID_Angle, exo_controller.xzy_shoulder.dm_motor[1].angle_in_degree, TargetAngle2);
    VelocityLoopInput = float_constrain(exo_controller.xzy_shoulder.dm_motor[1].PID_Angle.Output,
                                        -exo_controller.xzy_shoulder.dm_motor[1].PID_Angle.MaxOut, exo_controller.xzy_shoulder.dm_motor[1].PID_Angle.MaxOut);

    PID_Calculate(&exo_controller.xzy_shoulder.dm_motor[1].PID_Velocity, exo_controller.xzy_shoulder.dm_motor[1].velocity_in_radps, VelocityLoopInput);
    TorqueLoopInput = float_constrain(exo_controller.xzy_shoulder.dm_motor[1].PID_Velocity.Output,
                                      -exo_controller.xzy_shoulder.dm_motor[1].PID_Velocity.MaxOut, exo_controller.xzy_shoulder.dm_motor[1].PID_Velocity.MaxOut);

    exo_controller.xzy_shoulder.dm_motor[1].Output = float_constrain(TorqueLoopInput, -exo_controller.xzy_shoulder.dm_motor[1].max_out, exo_controller.xzy_shoulder.dm_motor[1].max_out);

    /* ------------------------------- elbow motor ------------------------------ */
    PID_Calculate(&exo_controller.elbow.dm_motor.PID_Angle, exo_controller.elbow.dm_motor.angle_in_degree, TargetAngle4);
    VelocityLoopInput = float_constrain(exo_controller.elbow.dm_motor.PID_Angle.Output,
                                        -exo_controller.elbow.dm_motor.PID_Angle.MaxOut, exo_controller.elbow.dm_motor.PID_Angle.MaxOut);
    PID_Calculate(&exo_controller.elbow.dm_motor.PID_Velocity, exo_controller.elbow.dm_motor.velocity_in_radps, VelocityLoopInput);
    TorqueLoopInput = float_constrain(exo_controller.elbow.dm_motor.PID_Velocity.Output,
                                      -exo_controller.elbow.dm_motor.PID_Velocity.MaxOut, exo_controller.elbow.dm_motor.PID_Velocity.MaxOut);

    exo_controller.elbow.dm_motor.Output = float_constrain(TorqueLoopInput, -exo_controller.elbow.dm_motor.max_out, exo_controller.elbow.dm_motor.max_out);
    break;

  case Velocity:
    // 速度控制
    // Motor_Speed_Calculate(&exo_controller.xzy_shoulder.lk_motor, exo_controller.xzy_shoulder.lk_motor.velocity_in_rpm, TargetVelocity);

    // PID_Calculate(&exo_controller.xzy_shoulder.lk_motor.PID_Velocity, exo_controller.xzy_shoulder.lk_motor.velocity_in_rpm, TargetVelocity);
    // TorqueLoopInput = float_constrain(exo_controller.xzy_shoulder.lk_motor.PID_Velocity.Output,
    //                                   -exo_controller.xzy_shoulder.lk_motor.PID_Velocity.MaxOut, exo_controller.xzy_shoulder.lk_motor.PID_Velocity.MaxOut);

    // exo_controller.xzy_shoulder.lk_motor.Output = float_constrain(TorqueLoopInput, -exo_controller.xzy_shoulder.lk_motor.max_out, exo_controller.xzy_shoulder.lk_motor.max_out);

    /* --------------------------------- dm motor -------------------------------- */
    PID_Calculate(&exo_controller.xzy_shoulder.dm_motor[0].PID_Velocity, exo_controller.xzy_shoulder.dm_motor[0].velocity_in_radps, TargetVelocity);
    TorqueLoopInput = float_constrain(exo_controller.xzy_shoulder.dm_motor[0].PID_Velocity.Output,
                                      -exo_controller.xzy_shoulder.dm_motor[0].PID_Velocity.MaxOut, exo_controller.xzy_shoulder.dm_motor[0].PID_Velocity.MaxOut);

    exo_controller.xzy_shoulder.dm_motor[0].Output = float_constrain(TorqueLoopInput, -exo_controller.xzy_shoulder.dm_motor[0].max_out, exo_controller.xzy_shoulder.dm_motor[0].max_out);

    // 避免模式切换疯掉
    TargetAngle1 = exo_controller.xzy_shoulder.dm_motor[0].angle_in_degree;
    break;

  case Debug:
    exo_controller.xzy_shoulder.lk_motor.Output = exo_controller.xzy_shoulder.lk_motor.debug_output;
    TargetAngle1 = exo_controller.xzy_shoulder.lk_motor.angle_in_degree;
    break;
  }
}

static uint8_t check_motor_angle(void)
{
  static float flag_change_timestamp;
  static uint8_t tmp_check_flag;

  if (check_float_range(exo_controller.xzy_shoulder.dm_motor[0].angle_in_degree, SSM_MOTOR1_MIN, SSM_MOTOR1_MAX) &&
      check_float_range(exo_controller.xzy_shoulder.dm_motor[1].angle_in_degree, SSM_MOTOR2_MIN, SSM_MOTOR2_MAX) &&
      check_float_range(exo_controller.xzy_shoulder.INS_shoulder.xzy_order_angle[2], SSM_Y_ANGLE_MIN, SSM_Y_ANGLE_MAX) &&
      check_float_range(exo_controller.elbow.dm_motor.angle_in_degree, ELBOW_MOTOR_MIN, ELBOW_MOTOR_MAX))
    tmp_check_flag = 1;
  else if (tmp_check_flag == 1)
  {
    tmp_check_flag = 0;
    flag_change_timestamp = exo_controller.t;
  }

  if ((tmp_check_flag == 0) && ((exo_controller.t - flag_change_timestamp) > 0.005f))
    exo_controller.motor_angle_check_flag = 0;
  else
    exo_controller.motor_angle_check_flag = 1;

  return exo_controller.motor_angle_check_flag;
}

static void send_exo_motor_current(void)
{
  static uint8_t CAN_send_status;

  if (exo_controller.mode == SILENCE_MODE || check_motor_angle() == 0)
  {
    CAN_send_status = Send_DM_MIT_Command(&hcan1, 0X01, 0, 0, 0, 0, 0);
    DWT_Delay(0.0003f);
    CAN_send_status = CAN_send_status | Send_DM_MIT_Command(&hcan1, 0X02, 0, 0, 0, 0, 0);
    DWT_Delay(0.0003f);
    CAN_send_status = CAN_send_status | Send_DM_MIT_Command(&hcan1, 0X03, 0, 0, 0, 0, 0);

    exo_controller.CAN_send_status = CAN_send_status | Send_LK_Current_Single(&hcan2, 1, 0);

    if (exo_controller.CAN_send_status == HAL_OK)
      ;
    else
      exo_controller.CAN_send_error_count++;
  }
  else
  {
    CAN_send_status = Send_DM_MIT_Command(&hcan1, 0X01, 0, 0, 0, 0, exo_controller.xzy_shoulder.dm_motor[0].Output);
    DWT_Delay(0.0003f);
    CAN_send_status = CAN_send_status | Send_DM_MIT_Command(&hcan1, 0X03, 0, 0, 0, 0, exo_controller.xzy_shoulder.dm_motor[1].Output);
    DWT_Delay(0.0003f);
    CAN_send_status = CAN_send_status | Send_DM_MIT_Command(&hcan1, 0X02, 0, 0, 0, 0, exo_controller.elbow.dm_motor.Output);

    exo_controller.CAN_send_status = CAN_send_status | Send_LK_Current_Single(&hcan2, 1, exo_controller.xzy_shoulder.lk_motor.Output);

    if (exo_controller.CAN_send_status == HAL_OK)
      ;
    else
      exo_controller.CAN_send_error_count++;
  }
}
