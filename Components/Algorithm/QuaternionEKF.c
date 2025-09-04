/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.3.0
 * @date    2022/8/13
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  ———————
 *  as + 1
 ******************************************************************************
 */
#include "QuaternionEKF.h"

const float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};
float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
float IMU_QuaternionEKF_K[18];
float IMU_QuaternionEKF_H[18];

static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
void IMU_QuaternionEKF_Init(QEKF_INS_t *qekf_ins, float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
    qekf_ins->IMU_QuaternionEKF.User_Data = qekf_ins;

    qekf_ins->Initialized = 1;
    qekf_ins->Q1 = process_noise1;
    qekf_ins->Q2 = process_noise2;
    qekf_ins->R = measure_noise;
    qekf_ins->ChiSquareTestThreshold = 1e-8;
    qekf_ins->ConvergeFlag = 0;
    qekf_ins->ErrorCount = 0;
    qekf_ins->UpdateCount = 0;
    if (lambda > 1)
    {
        lambda = 1;
    }
    qekf_ins->lambda = lambda;
    qekf_ins->accLPFcoef = lpf;

    // 初始化矩阵维度信息
    Kalman_Filter_Init(&qekf_ins->IMU_QuaternionEKF, 6, 0, 3);
    Matrix_Init(&qekf_ins->ChiSquare, 1, 1, (float *)qekf_ins->ChiSquare_Data);

    // 姿态初始化
    qekf_ins->IMU_QuaternionEKF.xhat_data[0] = 1;
    qekf_ins->IMU_QuaternionEKF.xhat_data[1] = 0;
    qekf_ins->IMU_QuaternionEKF.xhat_data[2] = 0;
    qekf_ins->IMU_QuaternionEKF.xhat_data[3] = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    qekf_ins->IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    qekf_ins->IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    qekf_ins->IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    qekf_ins->IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    qekf_ins->IMU_QuaternionEKF.SkipEq3 = TRUE;
    qekf_ins->IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(qekf_ins->IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(qekf_ins->IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s²
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(QEKF_INS_t *qekf_ins, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;
    if (!qekf_ins->Initialized)
    {
        IMU_QuaternionEKF_Init(qekf_ins, 10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
    }

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    qekf_ins->dt = dt;

    qekf_ins->Gyro[0] = gx - qekf_ins->GyroBias[0];
    qekf_ins->Gyro[1] = gy - qekf_ins->GyroBias[1];
    qekf_ins->Gyro[2] = gz - qekf_ins->GyroBias[2];

    // set F
    halfgxdt = 0.5f * qekf_ins->Gyro[0] * dt;
    halfgydt = 0.5f * qekf_ins->Gyro[1] * dt;
    halfgzdt = 0.5f * qekf_ins->Gyro[2] * dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(qekf_ins->IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    qekf_ins->IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    qekf_ins->IMU_QuaternionEKF.F_data[2] = -halfgydt;
    qekf_ins->IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    qekf_ins->IMU_QuaternionEKF.F_data[6] = halfgxdt;
    qekf_ins->IMU_QuaternionEKF.F_data[8] = halfgzdt;
    qekf_ins->IMU_QuaternionEKF.F_data[9] = -halfgydt;

    qekf_ins->IMU_QuaternionEKF.F_data[12] = halfgydt;
    qekf_ins->IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    qekf_ins->IMU_QuaternionEKF.F_data[15] = halfgxdt;

    qekf_ins->IMU_QuaternionEKF.F_data[18] = halfgzdt;
    qekf_ins->IMU_QuaternionEKF.F_data[19] = halfgydt;
    qekf_ins->IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (qekf_ins->UpdateCount == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        qekf_ins->Accel[0] = ax;
        qekf_ins->Accel[1] = ay;
        qekf_ins->Accel[2] = az;
    }
    qekf_ins->Accel[0] = qekf_ins->Accel[0] * qekf_ins->accLPFcoef / (qekf_ins->dt + qekf_ins->accLPFcoef) + ax * qekf_ins->dt / (qekf_ins->dt + qekf_ins->accLPFcoef);
    qekf_ins->Accel[1] = qekf_ins->Accel[1] * qekf_ins->accLPFcoef / (qekf_ins->dt + qekf_ins->accLPFcoef) + ay * qekf_ins->dt / (qekf_ins->dt + qekf_ins->accLPFcoef);
    qekf_ins->Accel[2] = qekf_ins->Accel[2] * qekf_ins->accLPFcoef / (qekf_ins->dt + qekf_ins->accLPFcoef) + az * qekf_ins->dt / (qekf_ins->dt + qekf_ins->accLPFcoef);

    // set z,单位化重力加速度向量
    accelInvNorm = invSqrt(qekf_ins->Accel[0] * qekf_ins->Accel[0] + qekf_ins->Accel[1] * qekf_ins->Accel[1] + qekf_ins->Accel[2] * qekf_ins->Accel[2]);
    for (uint8_t i = 0; i < 3; i++)
    {
        qekf_ins->IMU_QuaternionEKF.MeasuredVector[i] = qekf_ins->Accel[i] * accelInvNorm; // 用加速度向量更新量测值
    }

    // get body state
    qekf_ins->gyro_norm = 1.0f / invSqrt(qekf_ins->Gyro[0] * qekf_ins->Gyro[0] +
                                         qekf_ins->Gyro[1] * qekf_ins->Gyro[1] +
                                         qekf_ins->Gyro[2] * qekf_ins->Gyro[2]);
    qekf_ins->accl_norm = 1.0f / accelInvNorm;

    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (qekf_ins->gyro_norm < 0.3f && qekf_ins->accl_norm > 9.8f - 0.5f && qekf_ins->accl_norm < 9.8f + 0.5f)
    {
        qekf_ins->StableFlag = 1;
    }
    else
    {
        qekf_ins->StableFlag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    qekf_ins->IMU_QuaternionEKF.Q_data[0] = qekf_ins->Q1 * qekf_ins->dt;
    qekf_ins->IMU_QuaternionEKF.Q_data[7] = qekf_ins->Q1 * qekf_ins->dt;
    qekf_ins->IMU_QuaternionEKF.Q_data[14] = qekf_ins->Q1 * qekf_ins->dt;
    qekf_ins->IMU_QuaternionEKF.Q_data[21] = qekf_ins->Q1 * qekf_ins->dt;
    qekf_ins->IMU_QuaternionEKF.Q_data[28] = qekf_ins->Q2 * qekf_ins->dt;
    qekf_ins->IMU_QuaternionEKF.Q_data[35] = qekf_ins->Q2 * qekf_ins->dt;
    qekf_ins->IMU_QuaternionEKF.R_data[0] = qekf_ins->R;
    qekf_ins->IMU_QuaternionEKF.R_data[4] = qekf_ins->R;
    qekf_ins->IMU_QuaternionEKF.R_data[8] = qekf_ins->R;

    // 调用kalman_filter.c封装好的函数,注意几个User_Funcx_f的调用
    Kalman_Filter_Update(&qekf_ins->IMU_QuaternionEKF);

    // 获取融合后的数据,包括四元数和xy零飘值
    qekf_ins->q[0] = qekf_ins->IMU_QuaternionEKF.FilteredValue[0];
    qekf_ins->q[1] = qekf_ins->IMU_QuaternionEKF.FilteredValue[1];
    qekf_ins->q[2] = qekf_ins->IMU_QuaternionEKF.FilteredValue[2];
    qekf_ins->q[3] = qekf_ins->IMU_QuaternionEKF.FilteredValue[3];
    qekf_ins->GyroBias[0] = qekf_ins->IMU_QuaternionEKF.FilteredValue[4];
    qekf_ins->GyroBias[1] = qekf_ins->IMU_QuaternionEKF.FilteredValue[5];
    qekf_ins->GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

    // // 利用四元数反解欧拉角
    // qekf_ins->Yaw = atan2f(2.0f * (qekf_ins->q[0] * qekf_ins->q[3] + qekf_ins->q[1] * qekf_ins->q[2]), 2.0f * (qekf_ins->q[0] * qekf_ins->q[0] + qekf_ins->q[1] * qekf_ins->q[1]) - 1.0f) * 57.295779513f;
    // qekf_ins->Pitch = atan2f(2.0f * (qekf_ins->q[0] * qekf_ins->q[1] + qekf_ins->q[2] * qekf_ins->q[3]), 2.0f * (qekf_ins->q[0] * qekf_ins->q[0] + qekf_ins->q[3] * qekf_ins->q[3]) - 1.0f) * 57.295779513f;
    // qekf_ins->Roll = asinf(-2.0f * (qekf_ins->q[1] * qekf_ins->q[3] - qekf_ins->q[0] * qekf_ins->q[2])) * 57.295779513f;

    // // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    // if (qekf_ins->Yaw - qekf_ins->YawAngleLast > 180.0f)
    // {
    //     qekf_ins->YawRoundCount--;
    // }
    // else if (qekf_ins->Yaw - qekf_ins->YawAngleLast < -180.0f)
    // {
    //     qekf_ins->YawRoundCount++;
    // }
    // qekf_ins->YawTotalAngle = 360.0f * qekf_ins->YawRoundCount + qekf_ins->Yaw;
    // qekf_ins->YawAngleLast = qekf_ins->Yaw;
    qekf_ins->UpdateCount++; // 初始化低通滤波用,计数测试用
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
 *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
 *
 * @param kf
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
    static float qInvNorm;

    QEKF_INS_t *qekf_ins = (QEKF_INS_t *)kf->User_Data;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // quaternion normalize
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F_data[4] = q1 * qekf_ins->dt / 2;
    kf->F_data[5] = q2 * qekf_ins->dt / 2;

    kf->F_data[10] = -q0 * qekf_ins->dt / 2;
    kf->F_data[11] = q3 * qekf_ins->dt / 2;

    kf->F_data[16] = -q3 * qekf_ins->dt / 2;
    kf->F_data[17] = -q0 * qekf_ins->dt / 2;

    kf->F_data[22] = q2 * qekf_ins->dt / 2;
    kf->F_data[23] = -q1 * qekf_ins->dt / 2;

    // fading filter,防止零飘参数过度收敛
    kf->P_data[28] /= qekf_ins->lambda;
    kf->P_data[35] /= qekf_ins->lambda;

    // 限幅,防止发散
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    QEKF_INS_t *qekf_ins = (QEKF_INS_t *)kf->User_Data;
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 计算预测得到的重力加速度方向(通过姿态获取的)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算预测值和各个轴的方向余弦
    for (uint8_t i = 0; i < 3; i++)
    {
        qekf_ins->OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    // 利用加速度计数据修正
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,卡方检验
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &qekf_ins->ChiSquare);
    // rk is small,filter converged/converging
    if (qekf_ins->ChiSquare_Data[0] < 0.5f * qekf_ins->ChiSquareTestThreshold)
    {
        qekf_ins->ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (qekf_ins->ChiSquare_Data[0] > qekf_ins->ChiSquareTestThreshold && qekf_ins->ConvergeFlag)
    {
        if (qekf_ins->StableFlag)
        {
            qekf_ins->ErrorCount++; // 载体静止时仍无法通过卡方检验
        }
        else
        {
            qekf_ins->ErrorCount = 0;
        }

        if (qekf_ins->ErrorCount > 50)
        {
            // 滤波器发散
            qekf_ins->ConvergeFlag = 0;
            kf->SkipEq5 = FALSE; // step-5 is cov mat P updating
        }
        else
        {
            //  残差未通过卡方检验 仅预测
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE; // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (qekf_ins->ChiSquare_Data[0] > 0.1f * qekf_ins->ChiSquareTestThreshold && qekf_ins->ConvergeFlag)
        {
            qekf_ins->AdaptiveGainScale = (qekf_ins->ChiSquareTestThreshold - qekf_ins->ChiSquare_Data[0]) / (0.9f * qekf_ins->ChiSquareTestThreshold);
        }
        else
        {
            qekf_ins->AdaptiveGainScale = 1;
        }
        qekf_ins->ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++)
    {
        kf->K_data[i] *= qekf_ins->AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->K_data[i * 3 + j] *= qekf_ins->OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

    // 零漂修正限幅,一般不会有过大的漂移
    if (qekf_ins->ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * qekf_ins->dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * qekf_ins->dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * qekf_ins->dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * qekf_ins->dt;
            }
        }
    }

    // 不修正yaw轴数据
    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
