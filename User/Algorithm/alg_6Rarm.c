/**
******************************************************************************
 * @file           : alg_6Rarm.c
 * @author         : WHY
 * @date           : 2026-3-23
 * @brief          : 6R机械臂的前馈动力学算法模块
 *
 * 本文件提供了6轴机械臂的运算方法：
 *      包括本机械臂的动力学参数
 *     包括从DH参数到齐次变换矩阵（旋转矩阵、平移向量）的转化、
 *     牛顿欧拉法动力学前馈（已知关节角度及导数，目标角加速度，前向递推连杆质心加速度、角加速度，后向递推关节受力）
 *
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "alg_6Rarm.h"
#include "arm_math.h"  //armdsp库
#include "alg_ahrs.h"
#include "robot_def.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#define DOF 6                     // 机械臂自由度

// -------------------- 全局变量 --------------------
// DH参数（由用户初始化）
float DH_a[DOF]={0,0,0.447f,0,0,0};                  // a0-a5
float DH_alpha[DOF]={0,-90*ANGLE_TO_RAD,0,-90*ANGLE_TO_RAD,90*ANGLE_TO_RAD,-90*ANGLE_TO_RAD};              // alpha0-5
float DH_d[DOF]={0.060f,0,0,0.159f+0.066,0,0.185f};                  // d1-d6
// 关节运动参数（由用户提供）
float theta[DOF];                 // 关节角度theta1-theta6
float theta_dot[DOF]={0};             // 关节角速度
float theta_ddot[DOF]={0};            // 关节角加速度

float mass[DOF] = {1.752f, 1.374f, 0.494f, 0.457f, 0.454f, 0.577f};//连杆i+1的质量（1-6）
const float Pc[DOF][3] = {
    {0.0f, 0.0f, 0.084f},//缺
    {0.149f, 0.0f, 0.0f},
    {0.0f, 0.073f,0.0f },
    {0.0f, 0.0f, -0.065f},
    {0.0f,0.020f,0.0f },
    {0.0f,0.0f,-0.165f}
};//连杆i+1质心在坐标系i+1中的矢量;连杆1质心在坐标系1中的矢量 到 连杆6质心在坐标系6中的矢量

// 连杆质量与惯性张量（由用户提供）
float inertia[DOF][6] = {
    // 连杆1（缺）
    {0.012079f, 0.009465f, 0.008856f, 0.0f, 0.0f, 0.0f},
    // 连杆2
    {0.004300f, 0.038220f, 0.035088f, 0.0f, 0.0f, 0.0f},
    // 连杆3
    {0.001048f, 0.000407f, 0.000894f, 0.0f, 0.0f, 0.0f},
    // 连杆4
    {0.000378f, 0.000305f, 0.000304f, 0.0f, 0.0f, 0.0f},
    // 连杆5
    {0.000565f, 0.000367f, 0.000402f, 0.0f, 0.0f, 0.0f},
    // 连杆6
    {0.002245f, 0.000914f, 0.001552f, 0.0f, 0.0f, 0.0f}
};


// 旋转矩阵和转置（3x3，行优先存储）
float R[DOF][9];                  // R[i] = 从i+1系到i系的旋转矩阵 从^0_1R 到 ^5_6R
float R_T[DOF][9];                // R_T[i] = 从i系到i+1系的旋转矩阵 从^0_1R 到 ^6_5R

// 位置矢量（在i-1坐标系下）
float P[DOF][3];                  // P[i] = 坐标系i+1原点在i系中的位置

// 运动学中间变量
float omega[DOF][3];              // omega[i] = 连杆i+1在i+1系中角速度,omega0代码中单列
float alpha[DOF][3];              // alpha[i] = 连杆i+1在i+1系中角加速度
float a[DOF][3];                  // a[i] = 坐标系i+1原点在i+1系中线加速度
float a_c[DOF][3];                // a_c[i] = 连杆i+1质心在i+1系中线加速度

// 力与力矩（在i系中）
float f[DOF+1][3];                // f[i] = 关节i+1施加在连杆i+1上的力,f[DOF]为机械臂末端受力
float n[DOF+1][3];                // n[i] = 关节i+1施加在连杆i+1上的力矩
float tau[DOF];                   // tau[i] = 关节i+1的驱动力矩

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


// -------------------- 辅助函数 --------------------
/**
 * @brief 三维向量叉乘
 * @param a 左项
 * @param b 右项
 * @param out 叉乘输出
 */
static void cross_3(const float *a, const float *b, float *out) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}
/**
 * @brief 从由电机零点确定的关节角度转化到DH参数法确定的关节角度
 * @param JointRad 关节弧度
 */
void DATA_JointAngleToTheta(const float *JointRad)
{
    theta[0]=JointRad[0];
    theta[1]=-(JointRad[1]+90*ANGLE_TO_RAD+65*ANGLE_TO_RAD);
    theta[2]=JointRad[2]+90*ANGLE_TO_RAD-40*ANGLE_TO_RAD;
    theta[3]=JointRad[3];
    theta[4]=-JointRad[4];
    theta[5]=JointRad[5];
}

// -------------------- 函数1：计算旋转矩阵及相关矢量 --------------------
/**
 * @brief 由DH参数计算齐次变换矩阵
 */
void DATA_TransformMatrices(void) {
    for (int i = 0; i < DOF; i++) {
        // 改进型DH需要用到上一连杆的参数
        float a_prev = DH_a[i];
        float alpha_prev = DH_alpha[i];
        float d = DH_d[i];
        float th = theta[i];

        float s = arm_sin_f32(th);
        float c = arm_cos_f32(th);
        float c_alpha_prev = arm_cos_f32(alpha_prev);
        float s_alpha_prev = arm_sin_f32(alpha_prev);

        // 构建旋转矩阵 R_i^{i-1}（从i系到i-1系）
        float R_i[9] = {
            c,                  -s,               0.0f,
            s * c_alpha_prev,  c * c_alpha_prev,  -s_alpha_prev,
            s * s_alpha_prev,  c * s_alpha_prev , c_alpha_prev
        };

        // 转置矩阵 R_T_i（从i-1系到i系）
        float R_T_i[9] = {
            R_i[0], R_i[3], R_i[6],
            R_i[1], R_i[4], R_i[7],
            R_i[2], R_i[5], R_i[8]
        };

        // 存储旋转矩阵及其转置
        for (int j = 0; j < 9; j++) {
            R[i][j] = R_i[j];
            R_T[i][j] = R_T_i[j];
        }

        // 计算位置矢量 P_i（i原点在i-1系中的位置）
        P[i][0] = a_prev;
        P[i][1] = -s_alpha_prev * d;
        P[i][2] = c_alpha_prev * d;

        // 计算质心在自身坐标系中的位置 Pc
    }
}

/**
 * @brief 使用牛顿-欧拉法计算机械臂前馈力矩（仅考虑重力）
 * @param tau_out 输出力矩数组指针，长度DOF
 * @note  外部设置theta\theta_dot\theta_ddot
 */
void DATA_ForwardDynamics(float *tau_out) {
    // ---------- 1. 正向递推：计算角速度、角加速度、线加速度、质心加速度 ----------
    // 基座初始值（均为零）
    float omega0[3] = {0.0f, 0.0f, 0.0f};
    float alpha0[3] = {0.0f, 0.0f, 0.0f};   //角加速度
    float a0[3] = {0.0f, 0.0f, 9.81f};      // 基座原点加速度

    for (int i = 0; i < DOF; i++) {
        float R_T_i_in_ip[9];//i在i+1中
        for (int j = 0; j < 9; j++) R_T_i_in_ip[j] = R_T[i][j];

        // 存储变量：前一连杆的角速度/角加速度/线加速度转换到当前连杆坐标系
        float omega_i_in_ip[3], alpha_i_in_ip[3], a_i_in_ip[3];
        arm_matrix_instance_f32 mat_R_T, vec_i, vec_cur;
        arm_mat_init_f32(&mat_R_T, 3, 3, R_T_i_in_ip);//变换矩阵i在i+1中

        // 角速度转换坐标
        arm_mat_init_f32(&vec_i, 3, 1, (i == 0) ? omega0 : omega[i-1]);
        arm_mat_init_f32(&vec_cur, 3, 1, omega_i_in_ip);
        arm_mat_mult_f32(&mat_R_T, &vec_i, &vec_cur);

        // 计算当前连杆角速度 omega_ip_in_ip i=0-5
        float z0[3] = {0.0f, 0.0f, 1.0f};//转轴都是z轴
        for (int j = 0; j < 3; j++) {
            omega[i][j] = omega_i_in_ip[j];//TODO:目前速度均为0省去后面部分+ theta_dot[i] * z0[j]
        }

        // 角加速度转换
        arm_mat_init_f32(&vec_i, 3, 1, (i == 0) ? alpha0 : alpha[i-1]);
        arm_mat_init_f32(&vec_cur, 3, 1, alpha_i_in_ip);
        arm_mat_mult_f32(&mat_R_T, &vec_i, &vec_cur);

        // 计算角加速度 alpha_ip_in_ip i=0-5
        //float cross_term[3];//TODO:目前速度均为0省去部分
        //cross_product(omega_prev_in_i, theta_dot, cross_term);//TODO:目前速度均为0省去部分

        for (int j = 0; j < 3; j++) {
            //cross_term[j] *= theta_dot[i];//TODO:目前速度均为0省去部分
            alpha[i][j] = alpha_i_in_ip[j];//TODO:目前速度加速度均为0省去部分+ cross_term[j] + theta_ddot[i] * z0[j]
        }

        // 线加速度转换
        arm_mat_init_f32(&vec_i, 3, 1, (i == 0) ? a0 : a[i-1]);
        arm_mat_init_f32(&vec_cur, 3, 1, a_i_in_ip);
        arm_mat_mult_f32(&mat_R_T, &vec_i, &vec_cur);
        // 计算叉乘项
        float cross_alpha_P[3], omega_cross_P[3], omega_cross_omega_cross_P[3];
        cross_3(alpha[i], P[i], cross_alpha_P);
        cross_3(omega[i], P[i], omega_cross_P);
        cross_3(omega[i], omega_cross_P, omega_cross_omega_cross_P);
        //两个叉乘项坐标转换
        float cross_alpha_P_in_ip[3], omega_cross_omega_cross_P_in_ip[3];
        arm_mat_init_f32(&vec_i, 3, 1, cross_alpha_P);
        arm_mat_init_f32(&vec_cur, 3, 1, cross_alpha_P_in_ip);
        arm_mat_mult_f32(&mat_R_T, &vec_i, &vec_cur);

        arm_mat_init_f32(&vec_i, 3, 1, omega_cross_omega_cross_P);
        arm_mat_init_f32(&vec_cur, 3, 1, omega_cross_omega_cross_P_in_ip);
        arm_mat_mult_f32(&mat_R_T, &vec_i, &vec_cur);

        // 计算线加速度 a_ip_in_ip（原点）
        for (int j = 0; j < 3; j++) {
            a[i][j] = cross_alpha_P_in_ip[j] + omega_cross_omega_cross_P_in_ip[j] + a_i_in_ip[j] ;
        }

        // 计算质心加速度 a_c_i
        float cross_alpha_pc[3], omega_cross_pc[3], omega_cross_omega_cross_pc[3];
        cross_3(alpha[i], Pc[i], cross_alpha_pc);
        cross_3(omega[i], Pc[i], omega_cross_pc);
        cross_3(omega[i], omega_cross_pc, omega_cross_omega_cross_pc);
        for (int j = 0; j < 3; j++) {
            a_c[i][j] = cross_alpha_pc[j] + omega_cross_omega_cross_pc[j] + a[i][j];
        }
    }
    //
    // // ---------- 2. 计算重力在各连杆坐标系中的表示 ----------
    // // 计算从基座到各连杆的旋转矩阵 R_0_i（向量从基系转到i系）
    // float R_0_i[DOF][9];
    // for (int j = 0; j < 9; j++) R_0_i[0][j] = R_T[0][j];//0在1中表示
    // for (int i = 1; i < DOF; i++) {
    //     float R_T_i[9];
    //     for (int j = 0; j < 9; j++) R_T_i[j] = R_T[i][j];
    //     arm_matrix_instance_f32 mat_R_T, mat_R_0_im1, mat_R_0_i;
    //     arm_mat_init_f32(&mat_R_T, 3, 3, R_T_i);
    //     arm_mat_init_f32(&mat_R_0_im1, 3, 3, R_0_i[i-1]);
    //     arm_mat_init_f32(&mat_R_0_i, 3, 3, R_0_i[i]);
    //     arm_mat_mult_f32(&mat_R_T, &mat_R_0_im1, &mat_R_0_i);
    // }

    // ---------- 3. 反向递推：计算关节力和力矩 ----------
    // 初始化末端外力/力矩为零
    for (int j = 0; j < 3; j++) {
        f[DOF][j] = 0.0f;
        n[DOF][j] = 0.0f;
    }
    for (int i = DOF-1; i >= 0; i--) {

        //计算合力
        float F_i[3]={0};
        for (int j = 0; j < 3; j++) {
            F_i[j] = mass[i] * a_c[i][j];
        }
        // 计算f i+1在i中
        float R_ip_in_i[9];
        float f_ip_in_i[3];
        float R_7_in6[9]={1,0,0,0,1,0,0,0,1};
        for (int j = 0; j < 9; j++)
        {
            if (i==DOF-1)
                R_ip_in_i[j] = R_7_in6[j];
            else
                R_ip_in_i[j] = R[i+1][j];
        }
        arm_matrix_instance_f32 mat_R, mat_a_ip, mat_a_ip_in_i;
        arm_mat_init_f32(&mat_R, 3, 3, R_ip_in_i);
        arm_mat_init_f32(&mat_a_ip, 3, 1, f[i+1]);
        arm_mat_init_f32(&mat_a_ip_in_i, 3, 1, f_ip_in_i);
        arm_mat_mult_f32(&mat_R, &mat_a_ip, &mat_a_ip_in_i);
        for (int j = 0; j < 3; j++) {
            f[i][j] = f_ip_in_i[j] + F_i[j] ;
        }

        // 计算n i+1在i中
        float n_ip_in_i[3];
        arm_mat_init_f32(&mat_R, 3, 3, R_ip_in_i);
        arm_mat_init_f32(&mat_a_ip, 3, 1, n[i+1]);
        arm_mat_init_f32(&mat_a_ip_in_i, 3, 1, n_ip_in_i);
        arm_mat_mult_f32(&mat_R, &mat_a_ip, &mat_a_ip_in_i);

        // 惯性项：I_i * alpha_i
        float I_alpha[3];
        I_alpha[0] = inertia[i][0]*alpha[i][0] + inertia[i][3]*alpha[i][1] + inertia[i][4]*alpha[i][2];
        I_alpha[1] = inertia[i][3]*alpha[i][0] + inertia[i][1]*alpha[i][1] + inertia[i][5]*alpha[i][2];
        I_alpha[2] = inertia[i][4]*alpha[i][0] + inertia[i][5]*alpha[i][1] + inertia[i][2]*alpha[i][2];

        // 陀螺项：cross(omega, I_i * omega)
        float I_omega[3];
        I_omega[0] = inertia[i][0]*omega[i][0] + inertia[i][3]*omega[i][1] + inertia[i][4]*omega[i][2];
        I_omega[1] = inertia[i][3]*omega[i][0] + inertia[i][1]*omega[i][1] + inertia[i][5]*omega[i][2];
        I_omega[2] = inertia[i][4]*omega[i][0] + inertia[i][5]*omega[i][1] + inertia[i][2]*omega[i][2];
        float cross_omega_Iomega[3];
        cross_3(omega[i], I_omega, cross_omega_Iomega);

        // 计算力矩中的交叉项
        float cross_r_fnext[3], cross_P_f[3];
        cross_3(Pc[i], F_i, cross_r_fnext);
        if (i==DOF-1)
        {
            float P_7_in_6[3]={0,0,0};
            cross_3(P_7_in_6, f_ip_in_i, cross_P_f);
        }else
            cross_3(P[i+1], f_ip_in_i, cross_P_f);

        // 合力矩
        for (int j = 0; j < 3; j++) {
            n[i][j] = I_alpha[j] + cross_omega_Iomega[j] +n_ip_in_i[j] + cross_r_fnext[j] + cross_P_f[j] ;
        }

        // 关节扭矩（旋转关节，绕z轴）TODO:由于本机械臂按Z0简化，其实得乘Z确定轴向
        tau[i] = n[i][2];
    }

    // 输出结果
    tau_out[0]=tau[0]/afARMMotorRatio[0];
    tau_out[1]=-tau[1]/afARMMotorRatio[1];
    tau_out[2]=tau[2]/afARMMotorRatio[2];
    tau_out[3]=tau[3]/afARMMotorRatio[3];
    tau_out[4]=-tau[4]/afARMMotorRatio[4];
    tau_out[5]=tau[5]/afARMMotorRatio[5];
}