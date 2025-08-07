/**
 * @file PID.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "PID.hpp"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief �޷�����
 * 
 * @tparam Type 
 * @param x ��������
 * @param Min ��Сֵ
 * @param Max ���ֵ
 */
template <typename Type>
void Math_Constrain(Type *x, Type Min, Type Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
}

/**
 * @brief �����ֵ
 * 
 * @tparam Type 
 * @param x ��������
 * @return Type x�ľ���ֵ
 */
template <typename Type>
Type Math_Abs(Type x)
{
    return((x > 0) ? x : -x); 
}

/**
 * @brief PID��ʼ��
 * 
 * @param __K_P Pֵ
 * @param __K_I Iֵ
 * @param __K_D Dֵ
 * @param __I_Out_Max �����޷�
 * @param __Out_Max ����޷�
 */
void Class_PID::Init(float __K_P, float __K_I, float __K_D, float __I_Out_Max, float __Out_Max)
{
    K_P = __K_P;
    K_I = __K_I;
    K_D = __K_D;
    I_Out_Max = __I_Out_Max;
    Out_Max = __Out_Max;
}

/**
 * @brief �趨PID��P
 * 
 * @param __K_P PID��P
 */
void Class_PID::Set_K_P(float __K_P)
{
    K_P = __K_P;
}

/**
 * @brief �趨PID��I
 * 
 * @param __K_I PID��I
 */
void Class_PID::Set_K_I(float __K_I)
{
    K_I = __K_I;
}

/**
 * @brief �趨PID��D
 * 
 * @param __K_D PID��D
 */
void Class_PID::Set_K_D(float __K_D)
{
    K_D = __K_D;
}

/**
 * @brief �趨�����޷�, 0Ϊ������
 * 
 * @param __I_Out_Max �����޷�, 0Ϊ������
 */
void Class_PID::Set_I_Out_Max(float __I_Out_Max)
{
    I_Out_Max = __I_Out_Max;
}

/**
 * @brief �趨����޷�, 0Ϊ������
 * 
 * @param __Out_Max ����޷�, 0Ϊ������
 */
void Class_PID::Set_Out_Max(float __Out_Max)
{
    Out_Max = __Out_Max;
}

/**
 * @brief �趨Ŀ��ֵ
 * 
 * @param __Target Ŀ��ֵ
 */
void Class_PID::Set_Target(float __Target)
{
    Target = __Target;
}

/**
 * @brief �趨��ǰֵ
 * 
 * @param __Now ��ǰֵ
 */
void Class_PID::Set_Now(float __Now)
{
    Now = __Now;
}

/**
 * @brief �趨����, Error�������ֵ�ڲ����
 * 
 * @param __Dead_Zone ����, Error�������ֵ�ڲ����
 */
void Class_PID::Set_Dead_Zone(float __Dead_Zone)
{
    Dead_Zone = __Dead_Zone;
}

/**
 * @brief �趨�����ڶ���ֵ, 0Ϊ������
 * 
 * @param __Variable_Speed_I_A �����ڶ���ֵ, 0Ϊ������
 */
void Class_PID::Set_Variable_Speed_I_A(float __Variable_Speed_I_A)
{
    Variable_Speed_I_A = __Variable_Speed_I_A;
}

/**
 * @brief �趨��������, 0Ϊ������
 * 
 * @param __Variable_Speed_I_B ��������, 0Ϊ������
 */
void Class_PID::Set_Variable_Speed_I_B(float __Variable_Speed_I_B)
{
    Variable_Speed_I_B = __Variable_Speed_I_B;
}

/**
 * @brief �趨���ַ�����ֵ����Ϊ����, 0Ϊ������
 * 
 * @param __I_Separate_Threshold ���ַ�����ֵ����Ϊ����, 0Ϊ������
 */
void Class_PID::Set_I_Separate_Threshold(float __I_Separate_Threshold)
{
    I_Separate_Threshold = __I_Separate_Threshold;
}

/**
 * @brief �趨΢������
 * 
 * @param __D_First ΢������
 */
void Class_PID::Set_D_First(Enum_D_First __D_First)
{
    D_First = __D_First;
}

/**
 * @brief ��ȡ���ֵ
 * 
 * @return float ���ֵ
 */
float Class_PID::Get_Out()
{
    return(Out);
}

/**
 * @brief PID����ֵ
 * 
 * @return float ���ֵ
 */
void Class_PID::Adjust_TIM_PeriodElapsedCallback()
{
    // P���
    float p_out = 0.0f;
    // I���
    float i_out = 0.0f;
    // D���
    float d_out = 0.0f;
    // F���
    float f_out = 0.0f;
    // ���
    float error;
    // ����ֵ���
    float abs_error;
    // ���Ա��ٻ���
    float speed_ratio;

    error = Target - Now;
    abs_error = Math_Abs(error);

    // �ж�����
    if (abs_error < Dead_Zone)
    {
        Target = Now;
        error = 0.0f;
        abs_error = 0.0f;
    }

    // ����p��

    p_out = K_P * error;

    // ����i��

    if (Variable_Speed_I_A == 0.0f && Variable_Speed_I_B == 0.0f)
    {
        // �Ǳ��ٻ���
        speed_ratio = 1.0f;
    }
    else
    {
        // ���ٻ���
        if (abs_error <= Variable_Speed_I_A)
        {
            speed_ratio = 1.0f;
        }
        else if (Variable_Speed_I_A < abs_error && abs_error < Variable_Speed_I_B)
        {
            speed_ratio = (Variable_Speed_I_B - abs_error) / (Variable_Speed_I_B - Variable_Speed_I_A);
        }
        else if (abs_error >= Variable_Speed_I_B)
        {
            speed_ratio = 0.0f;
        }
    }
    // �����޷�
    if (I_Out_Max != 0.0f)
    {
        Math_Constrain(&Integral_Error, -I_Out_Max / K_I, I_Out_Max / K_I);
    }
    if (I_Separate_Threshold == 0.0f)
    {
        // û�л��ַ���
        Integral_Error += speed_ratio * D_T * error;
        i_out = K_I * Integral_Error;
    }
    else
    {
        // �л��ַ���
        if (abs_error < I_Separate_Threshold)
        {
            // ���ڻ��ַ���������
            Integral_Error += speed_ratio * D_T * error;
            i_out = K_I * Integral_Error;
        }
        else
        {
            // �ڻ��ַ���������
            Integral_Error = 0.0f;
            i_out = 0.0f;
        }
    }

    // ����d��

    if (D_First == D_First_YES)
    {
        // û��΢������
        d_out = K_D * (error - Pre_Error) / D_T;
    }
    else
    {
        // ΢������ʹ��
        d_out = K_D * (p_out + i_out - Pre_Out) / D_T;
    }

    // �������

    Out = p_out + i_out + d_out + f_out;

    // ����޷�
    if (Out_Max != 0.0f)
    {
        Math_Constrain(&Out, -Out_Max, Out_Max);
    }

    // �ƺ���
    Pre_Now = Now;
    Pre_Target = Target;
    Pre_Out = Out;
    Pre_Error = error;
}

/* Function prototypes -------------------------------------------------------*/


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
