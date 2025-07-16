/*
 * Guandao_Plus.c
 *
 *  Created on: 2025��3��26��
 *      Author: ������
 */
#include "zf_common_headfile.h"
#include <stdbool.h> // ���� stdbool.h
#include "control.h"
#include "flash.h"
#include "imu.h"
#include "motor.h"
#include "PID.h"
#include "Display.h"
#include "Guandao_Plus.h"
#include "key.h"
#include "filter.h"
/**
 * ======= �������㷨�ɵ��������� =======
 * ���²���Ӱ��Pure Pursuit�㷨�����ܺ�Ч�ʣ��ɸ���ʵ�ʳ�������
 */
// �������ҷ�Χ��ǰ������ٸ��㣩
// - �����ֵ��������ڴ��������ƫ��·��ʱ��³����
// - ��С��ֵ������߼���Ч�ʣ��ʺ�·���򵥻���ƫ���С�ĳ���
// - �Ƽ���Χ��10~50
#define LOOKAHEAD_SEARCH_RANGE 30

// Ԥ�������������������������ǰ��������������
// - �����ֵ���Դ���ϴ��Ԥ����룬�������Ӽ�����
// - ��С��ֵ�������Ч�ʣ��ʺ�Ԥ������С��·�����ܼ��ĳ���
// - �Ƽ���Χ��10~30
#define LOOKAHEAD_MAX_STEPS 20

// ��Ծ���������ƽ�����ࣨ��λ��cm��
// - ��ֵӦ���ڻ�ӽ�·�����ʵ�ʼ��
// - ��·����ÿ1cm��¼һ�Σ�����Ϊ1.0f
// - ��·�����಻ȷ�������鱣������Ϊ1.0f
// - ��ֱֵ��Ӱ����Ծ������׼ȷ�Ժ�Ч��
#define LOOKAHEAD_AVG_DIST 1.0f

// ������ٵľ������ϵ��
#define SCALE_FACTOR 80

/**
 * ===================================
 */

// С���������
#define distance (1.0) // ÿ���1cm��¼һ�ε�λ
#define WHEELBASE 17   // �־ࣨ�����־��룬��λcm��

float X_Memery_Plus[FLASH_PAGE_LENGTH * 6] = {0};
float Y_Memery_Plus[FLASH_PAGE_LENGTH * 6] = {0};
float X_Memery_Store_Plus[FLASH_PAGE_LENGTH * 6] = {0}; // С������·�������챱����ϵ����λcm
float Y_Memery_Store_Plus[FLASH_PAGE_LENGTH * 6] = {0}; // С��ÿ��1cm��¼һ��·����
volatile float X = 0, Y = 0;                            // С��ʵʱλ�ã����챱����ϵ����λcm

uint8_t road_memery_finish_Plus_flag = 0; // ·��������ɱ�־λ
uint8_t road_memery_start_Plus_flag = 0;  // ·�����俪ʼ��־λ
uint8_t road_recurrent_Plus_flag = 0;     // ·�����ֱ�־λ
uint16_t NUM_L_Plus = 0, NUM_R_Plus = 0;  // ÿ�߹�1cm,NUM+1
uint16_t road_destination = 0;            // ·���յ�

/*           */
volatile int16 err_guandao_plus = 0; // �ߵ�����
volatile float e_lat = 0;            // �ߵ��������
volatile int32_t speed_left = 0;     // �����ٶȣ�����/5ms��
volatile int32_t speed_right = 0;    // �����ٶȣ�����/5ms��
// С��ʵʱ״̬
volatile int locate_index = 0;           // С����λ��
static int target_index = 0;             // С��Ŀ���
const int32_t MAX_PHYSICAL_DELTA = 1000; // ��������������������ڷ����Լ��

volatile SlipFlags wheelSlipFlags = SLIP_FLAG_NONE;
volatile float leftWheelSpeedCmps = 0;
volatile float rightWheelSpeedCmps = 0;
volatile float currentAvgSpeedCmps = 0;

/**
 * @brief �������ݽṹ�壬��װ�������ӵı������������
 *
 * @��Ա total_pulses     �ۼ��������������������㹦��
 * @��Ա encoder_last     ��һ�ζ�ȡ�ı�����ֵ�����ڼ�������
 * @��Ա encoder_prev_speed �ٶȼ����׼ֵ��������һ���ж�ʱ�ı�����ֵ
 * @��Ա finish_flag;     �ﵽ�趨��ֵ��־λ
 * @��Ա target_pulses    Ŀ�����������ﵽ��ֵ�󴥷��������㣨Ӧ��Ϊ������
 */

// ��ʼ���������ݽṹ��
// - total_pulses: ��ʼ�ۼ�������Ϊ0
// - last_total_pulses: ��ʼ��һ��������Ϊ0
// - encoder_last: ��ʼ������ֵΪ0
// - encoder_prev_speed: ��ʼ�ٶȻ�׼ֵΪ0
// - target_pulses: ����������ֵΪ366���壨�����ʵ�ʵ�����
WheelData wheel_left = {
    .total_pulses = 0,
    .last_total_pulses = 0,
    .encoder_last = 0,
    .encoder_prev_speed = 0,
    .target_pulses = 366};

// ��ʼ���������ݽṹ�壨����ͬ���֣�
WheelData wheel_right = {
    .total_pulses = 0,
    .last_total_pulses = 0,
    .encoder_last = 0,
    .encoder_prev_speed = 0,
    .target_pulses = 366};

/**
 * @brief ����������������Զ�����int16���
 *
 * @param current ��ǰ������ֵ��int16���ͣ�
 * @param last ��һ�α�����ֵ��int16���ͣ�
 * @return int32_t �������������ʵ����������仯��
 *
 * @˵��
 * - ������������Ϊ16λ�з�������-32768~32767����
 *   �����������ʱ�����32767��Ϊ-32768����ֱ�������õ�����������
 * - �˺���ͨ���жϲ�ֵ��Χ����������ͷ��������
 */
static inline int32_t calculate_delta(int16_t current, int16_t last)
{
    int32_t delta = (int32_t)current - (int32_t)last;

    // ������������������32767��-32768��ʵ������Ϊ+1��
    if (delta > 32767)
    {
        delta -= 65536; // 65536 = 2^16
    }
    // ����������������-32768��32767��ʵ������Ϊ-1��
    else if (delta < -32768)
    {
        delta += 65536;
    }

    return delta;
}

/**
 * @brief �������������ۼ�ֵ��������������
 *
 * @param wheel ��������ָ�루ָ�����ֻ����ֵĽṹ�壩
 * @param current_encoder ��ǰ������ֵ
 *
 * @˵��
 * - �˺���Ӧ����ѭ���ж��ڵ��ã�ȷ��ʵʱ���������ۼ�ֵ��
 * - �����Լ������쳣�������䣨���źŸ��ţ���
 * - ʹ��"����ֵ��ⷨ"ͬʱ֧������ͷ����ۼƣ����ǰ�����˽���ʱ������ס���⡣
 * - �������delta�����������ת�л�ʱ������ֵ���䵼�µļ�����ס���⡣
 */
int32_t Update_Wheel_Pulses(WheelData *wheel, int16_t current_encoder)
{
    // 1. �������� (�����������)
    int32_t delta = calculate_delta(current_encoder, wheel->encoder_last);
    int32_t forward_crossings = 0; // �ں�����ʼʱ��������ʼ��

    // 2. �����������߼�
    //    ��ֵ��Ҫ����ʵ�����������Ŀ���ǲ���Ӳ�����䣬�������˸������塣
    if (abs(delta) > 1000)
    {
        // printf("Jump detected! Delta: %d, LastEnc: %d, CurrEnc: %d\n",
        //        (int)delta, (int)wheel->encoder_last, (int)current_encoder); // �������

        // �����ۼ����壬����������ź�������
        wheel->total_pulses = 0;
        // ���ε��ò�����λ�ƣ���Ϊ��������������
        forward_crossings = 0; // ȷ������ʱ����0
    }
    else
    {
        // --- ���������ۼӺͼ���ֵ���� ---
        // ������һ�ε��ۼ�����ֵ�����ڼ�����ֵ (����������µ��߼���ʵ��δʹ�ã�������Ҳ�޺�)
        wheel->last_total_pulses = wheel->total_pulses;

        // �����ۼ���������֧������ת����ֵ��ʾ����ת����
        wheel->total_pulses += delta;

        // ����ǰ����ֵ��Խ����
        if (wheel->total_pulses >= wheel->target_pulses)
        {
            forward_crossings = wheel->total_pulses / wheel->target_pulses; // ���㴩Խ�˶��ٴ���ֵ
            wheel->total_pulses %= wheel->target_pulses;                    // ����������������һ���ۼ�
        }
        // ������Խ (�����Ҫ��������߼��Ļ�)
        else if (wheel->total_pulses <= -wheel->target_pulses)
        {
            wheel->total_pulses %= wheel->target_pulses; // ģ��ȡģ����������ֵ��Χ��
                                                         // ȷ������պ��� -target_pulses��ģ������Ϊ 0
            if (wheel->total_pulses == -wheel->target_pulses)
            {
                wheel->total_pulses = 0;
            }
        }
    }

    // 3. ���浱ǰ������ֵ���´�ʹ��
    wheel->encoder_last = current_encoder;

    // 4. ���ر��μ������ǰ��������
    return forward_crossings;
}

/**
 * @brief ��ȡ�����ڹ̶�ʱ�䴰��5ms�ڵ��ٶȣ�����������
 *
 * @param wheel ��������ָ��
 * @param current_encoder ��ǰ������ֵ
 * @return int32_t ʱ�䴰���ڵ�������������ֵΪ���򣬸�ֵΪ����
 *
 * @˵��
 * - �˺���Ӧ�ڶ�ʱ�жϣ���5ms���е��ã�ȷ���̶�ʱ������
 * - ÿ�ε��û����encoder_prev_speedΪ��ǰֵ�����´μ���ʹ�á�
 */
int32_t Get_Wheel_Speed(WheelData *wheel, int16_t current_encoder)
{
    // ���㵱ǰ������ֵ����һ���ж�ʱ�Ļ�׼ֵ������
    int32_t delta = calculate_delta(current_encoder, wheel->encoder_prev_speed);

    // ���»�׼ֵΪ��ǰֵ��������һ���ٶȼ���
    wheel->encoder_prev_speed = current_encoder;

    if (abs(delta) > 1000)
        delta = 0;
    return delta;
}

/*---------------------------------------------
 * 1. ·�����亯��
 *---------------------------------------------*/
void Distance_Get_Plus(void)
{
    // ȷ������Խ���������
    if (NUM_L_Plus >= FLASH_PAGE_LENGTH * 6 - 2)
    {
        road_memery_finish_Plus_flag = 1; // ·��������ɱ�־λ
        return;                           // ֱ�ӷ��أ����ټ�¼�µĵ�
    }

    NUM_L_Plus++;
    X += distance * (-sinf(yaw_plus));
    Y += distance * cosf(yaw_plus);
    X_Memery_Plus[NUM_L_Plus] = X;
    Y_Memery_Plus[NUM_L_Plus] = Y;
}

/*---------------------------------------------
 * 2. Ԥ�����Һ���
 *---------------------------------------------*/
/**
 * @brief ���Ҿ��뵱ǰλ��L���׵�Ԥ�������
 * @param L Ԥ����루��λ�����ף�
 * @return Ԥ�����·�������е�����
 *
 * @˵�� �˺���ʹ��"���뷨"����Ԥ��㣬ȷ��Ԥ���ʼ���ڳ���ǰ���̶�������봦
 * �㷨��������
 * 1. ���ҵ����뵱ǰλ�������ǰ��·������Ϊ�ο��㣨��λ�㣩
 * 2. ʹ����Ծʽ�������ԣ����ٽӽ����ܵ�Ԥ���λ��
 * 3. ����Ծλ�þ�ȷ�������ҵ���һ�����뵱ǰλ�ô��ڵ���Ԥ�����L�ĵ�
 *
 * ���ƣ�
 * - ��ȼ򵥵�"����ƫ�Ʒ�"���Ӿ�ȷ���ܹ���Ӧ·����ֲ������ȵ����
 * - ʹ��"ֻѡǰ����"���ԣ�����С����ͷ����
 * - ��Ծʽ����������ټ������������ʺϳ�·��
 * - �ϸ�ı߽籣������ֹ����Խ��
 */
void find_lookahead_index(float L)
{
    /**
     * ����1���ҵ����뵱ǰλ�������ǰ��·����
     */
    // ��ʼ����С����Ϊ��󸡵�ֵ��ȷ����һ�αȽ�ʱһ�������
    float min_distance = FLT_MAX;
    int closest_index = 0;

    // ������Χ���ƣ��Ե�ǰĿ���Ϊ���ģ���ǰ�������LOOKAHEAD_SEARCH_RANGE����
    // ����������Ӧ����ƫ��·����ת��������ͬʱ���Ƽ�����
    int search_start = target_index - LOOKAHEAD_SEARCH_RANGE;
    int search_end = target_index + LOOKAHEAD_SEARCH_RANGE;

    // �߽��飺��ֹ����Խ�磬ȷ����ȫ��������
    if (search_start < 0)
        search_start = 0;
    if (search_end >= FLASH_PAGE_LENGTH * 6)
        search_end = FLASH_PAGE_LENGTH * 6 - 1;

    // ���趨��Χ��Ѱ�Ҿ��뵱ǰλ�������ǰ��·����
    for (int i = search_start; i <= search_end; i++)
    {
        // ���㵱ǰ�㵽·�����ŷ�Ͼ���
        float dx = X_Memery_Store_Plus[i] - X; // X�����ֵ
        float dy = Y_Memery_Store_Plus[i] - Y; // Y�����ֵ
        float dist = sqrtf(dx * dx + dy * dy); // ŷ�Ͼ���

        // ��������㣺����ǰ������С����������������
        if (dist < min_distance)
        {
            min_distance = dist;
            closest_index = i;
        }
    }

    // ���¶�λ��Ϊ�ҵ�������㣨����С����·���ϵ�"ͶӰ��"��
    locate_index = closest_index;

    /**
     * ����2����Ծʽ��������
     */
    // ����Ԥ���������Χ�������ǰ����LOOKAHEAD_MAX_STEPS����
    // �������Ա����ڳ�·��ʱ�������յ㣬���Ч��
    int search_limit = closest_index + LOOKAHEAD_MAX_STEPS;
    if (search_limit >= FLASH_PAGE_LENGTH * 6)
        search_limit = FLASH_PAGE_LENGTH * 6 - 1;

    // ������µ���Ծ������Ԥ������ƽ������
    // �������Կ��ٽӽ����ܵ�Ԥ���λ�ã����ٺ�����ȷ�����ķ�Χ
    int jump_steps = (int)(L / LOOKAHEAD_AVG_DIST);
    if (jump_steps < 1)
        jump_steps = 1; // ȷ��������Ծ1��

    // ������ԾĿ��㣺��������������ǰ��Ծ
    int jump_target = closest_index + jump_steps;
    if (jump_target > search_limit)
        jump_target = search_limit; // ȷ��������������Χ

    // ������Ծ�㵽��ǰλ�õ�ʵ�ʾ���
    float jump_dx = X_Memery_Store_Plus[jump_target] - X;
    float jump_dy = Y_Memery_Store_Plus[jump_target] - Y;
    float jump_dist = sqrtf(jump_dx * jump_dx + jump_dy * jump_dy);

    /**
     * ����3��ȷ����ȷ������Χ
     */
    // ������Ծ�����ȷ����Ҫ��ȷ����������
    int fine_search_start, fine_search_end;

    if (jump_dist < L)
    {
        // ���1����Ծ�������С��Ԥ�����
        // ˵��Ԥ�������Ծ��֮�󣬴���Ծ�㿪ʼ��ǰ����
        fine_search_start = jump_target;
        fine_search_end = search_limit;
    }
    else
    {
        // ���2����Ծ������Ѵ���Ԥ�����
        // ˵��Ԥ�������������Ծ��֮�䣬��С������Χ
        fine_search_start = closest_index;
        fine_search_end = jump_target;
    }

    /**
     * ����4����ȷ����Ԥ���
     */
    // ��ȷ���ľ�ȷ���������ڣ��ҵ���һ��������ڵ���L�ĵ�
    for (int i = fine_search_start; i <= fine_search_end; i++)
    {
        float dx = X_Memery_Store_Plus[i] - X;
        float dy = Y_Memery_Store_Plus[i] - Y;
        float dist = sqrtf(dx * dx + dy * dy);

        // �ҵ���һ��������ڵ���Ԥ�����L�ĵ���ΪԤ���
        if (dist >= L)
        {
            target_index = i; // ����Ŀ��㣨Ԥ��㣩
            return;           // �ҵ����ʵ���������أ������������
        }
    }

    // �߽紦���������������û���ҵ����ʵ㣨���ѽӽ�·���յ㣩
    // ��ʹ��������������һ������ΪԤ���
    target_index = fine_search_end;
}

/*---------------------------------------------
 * 3. ���������㣨��ǰ������ϵ��
 *---------------------------------------------*/
/**
 * @brief ����Ԥ���ĺ�������ǰ������ϵ��
 * @param lookahead_index Ԥ�������
 * @return �������e_lat�����ף���ֵ��ʾԤ������Ҳࣩ
 */
float calculate_e_lat(int lookahead_index)
{
    float dx = X_Memery_Store_Plus[lookahead_index] - X;
    float dy = Y_Memery_Store_Plus[lookahead_index] - Y;
    // ֱ��ʹ�ú����yaw_plus��˳ʱ��Ϊ������ʱ��Ϊ����
    float rotated_x = dx * cosf(yaw_plus) + dy * sinf(yaw_plus);
    return rotated_x; // ��ֵ��ʾԤ�����С���Ҳ�
}

/*---------------------------------------------
 * 4. Pure Pursuit�����㷨
 *---------------------------------------------*/

/**
 * @brief �����ٿ����㷨������
 * @return ���ٿ���������ֵ��ʾ��ת����ֵ��ʾ��ת��
 *
 * @˵�� �������㷨�ĺ���˼���ǣ�
 * 1. ���ݵ�ǰλ�ú�Ԥ�����L����·�����ҵ�һ��Ԥ���
 * 2. ����ӵ�ǰλ�õ�Ԥ��������
 * 3. �������ʼ�����ٿ�����
 *
 * �ŵ㣺�㷨��ֱ�ۣ�������С���ʺϵ��ٳ���
 * �������ţ�Ԥ�����L����ؼ�������LԽС����Խ�����������𵴣�LԽ�����Խƽ�������پ��Ƚ���
 */
int16_t pure_pursuit_control()
{

    // ����Ԥ����루��λ�����ף�
    // ע�����ٳ���ʹ�ý�С��Ԥ�����ɻ�ø���ȷ�ĸ���Ч��
    // ��ѡ�Ż��������ٶȶ�̬���� L_distance = K1 * speed + K2
    static float L_distance = 20.0f; // Ԥ����룬��λ���ף����ٳ���ʹ�ý�Сֵ

    // ʹ��"���뷨"����Ԥ���
    find_lookahead_index(L_distance);

    // �����������������ϵ�£�Ԥ�����Գ���ĺ���ƫ�
    e_lat = calculate_e_lat(target_index);

    // ����Ԥ��㵽��ǰλ�õ�ʵ�ʾ���
    float dx = X_Memery_Store_Plus[target_index] - X;
    float dy = Y_Memery_Store_Plus[target_index] - Y;
    float L_actual = sqrtf(dx * dx + dy * dy); // ʵ��Ԥ����룬��λ����

    // ��ֹ���㣨ʵ�ʹ����в�����ܳ��֣���Ϊ�����Ա�̣�
    if (L_actual < 1e-3f)
        L_actual = 1e-3f;

    // �������� - �����ٿ��ƺ��Ĺ�ʽ
    // ����k = 2 * ������� / Ԥ������ƽ��
    float curvature = 2.0f * e_lat / (L_actual * L_actual);

    // ������ת��Ϊ���ٿ�����
    // delta_v = ���� * �־� * ����ϵ��
    // ע������ϵ���ɸ���ʵ�ʿ���Ч������
    int16_t delta_v = (int16_t)(curvature * WHEELBASE * SCALE_FACTOR);

    // �������ƣ���ֹ������غͿ�����ͻ��
    if (delta_v > 200)
        delta_v = 200;
    if (delta_v < -200)
        delta_v = -200;

    return delta_v; // ���ز��ٿ�����
}

// �򻬼��
/*-----------------------------------------------------------------------------
 * �������ƣ�UpdateWheelSpeeds
 * ������������speed_left,speed_rightת��Ϊʵ������������
 * ���������leftPulses  - ��������������������/�������ڣ���Speed_left
 *          rightPulses - ��������������������/�������ڣ���speed_right
 * �����������
 * ע    �⣺���ڹ̶�ʱ��������5ms�������Ա�֤�ٶȼ���׼ȷ��
 *----------------------------------------------------------------------------*/
void UpdateWheelSpeeds(int32_t leftPulses, int32_t rightPulses)
{

    // ����ת���ٶȣ������� �� ����/��
    // ��ʽ���ٶ� = (�̶�ʱ��������������/ÿcm������) /�������(��λ��)
    static float scale = (1000.0f / SAMPLING_INTERVAL_MS);

    leftWheelSpeedCmps = (float)leftPulses / ENCODER_PULSES_PER_REVOLUTION * scale;
    rightWheelSpeedCmps = (float)rightPulses / ENCODER_PULSES_PER_REVOLUTION * scale;
}

/*-----------------------------------------------------------------------------
 * �������ƣ�DetectLateralSlip
 * �����������������ƣ�����ת��ƥ�䣩
 * �����������
 * �������������wheelSlipFlags�еĺ����Ʊ�־λ
 * �����߼����Ƚ�����ƫ���ʣ��������ٲ��IMUʵ��ƫ���ʵ�ƫ��
 *----------------------------------------------------------------------------*/
void DetectLateralSlip(void)
{
    // ��������ƫ���ʣ���λ������/�룩
    // ��ʽ������ƫ���� = (�����ٶ� - �����ٶ�) / �־�
    float theoreticalYawRate = (rightWheelSpeedCmps - leftWheelSpeedCmps) / WHEEL_BASE_CM;

    // ����ƫ����ƫ�����ֵ��
    float yawRateDeviation = fabsf(theoreticalYawRate - gyro_param.gyro_z); // IMU������ƫ�����ٶȣ���λ������/�룩

    // ����ƽ���ٶȣ��ж��Ƿ�������������
    float avgSpeedCmps = 0.5f * (fabsf(leftWheelSpeedCmps) + fabsf(rightWheelSpeedCmps));

    // ����߼�
    if (avgSpeedCmps > MIN_LATERAL_DETECT_SPEED_CMPS &&
        yawRateDeviation > LATERAL_SLIP_YAW_RATE_THRESHOLD)
    {
        // �жϻ��Ʒ���
        if (theoreticalYawRate > gyro_param.gyro_z)
        {
            wheelSlipFlags |= SLIP_FLAG_LATERAL_RIGHT; // ���ִ򻬵���ת����
        }
        else
        {
            wheelSlipFlags |= SLIP_FLAG_LATERAL_LEFT; // ���ִ򻬵���ת�����
        }
    }
}

/*-----------------------------------------------------------------------------
 * �������ƣ�DetectLongitudinalSlip
 * ������������������ƣ������ֿ�ת������
 * �����������
 * �������������wheelSlipFlags�е������Ʊ�־λ
 * �����߼����Ƚ����ټ�������ۼ��ٶ���IMUʵ�ʼ��ٶȵĲ���
 *----------------------------------------------------------------------------*/
void DetectLongitudinalSlip(void)
{

    // ���㵱ǰƽ�����٣���λ����/�룩
    currentAvgSpeedCmps = 0.5f * (leftWheelSpeedCmps + rightWheelSpeedCmps) / 100.0f;

    float accelDiff = fabsf(currentAvgSpeedCmps - acc_y_speed); // �������ٶ�����ٶȼ��ٶȲ��λ����/�룩

    // printf("%f,%f\n", accelDiff, currentAvgSpeedCmps); // ��ӡ����
    //  ����߼�
    if (currentAvgSpeedCmps > MIN_LONGITUDINAL_DETECT_SPEED_CMPS)
    {
        if (accelDiff > LONGITUDINAL_ACCEL_DIFF_THRESHOLD)
        {
            wheelSlipFlags |= SLIP_FLAG_LONGITUDINAL_ACCEL; // ����
        }
    }

    // static float prevAvgSpeedCmps = 0.0f; // ��һ�ε�ƽ���ٶ�

    // // ���㵱ǰƽ�����٣���λ������/�룩
    // float currentAvgSpeedCmps = 0.5f * (leftWheelSpeedCmps + rightWheelSpeedCmps);

    // // �������ۼ��ٶȣ���λ����/�룩
    // // ��ʽ�����ٶ� = (��ǰ�ٶ� - �ϴ��ٶ�) / ����ʱ��
    // float timeIntervalSec = 1000.0f / SAMPLING_INTERVAL_MS;
    // float theoreticalAccelMps2 = (currentAvgSpeedCmps - prevAvgSpeedCmps) / 100.0f * timeIntervalSec;

    // // ������ʷ�ٶ�
    // prevAvgSpeedCmps = currentAvgSpeedCmps;

    // // ������ٶȲ��죨����ֵ��
    // float accelDiff = fabsf(theoreticalAccelMps2 - acc_param.acc_y); // IMU������ǰ����ٶȣ���λ����/��?��

    // printf("%f,%f\n", accelDiff, currentAvgSpeedCmps); // ��ӡ����
    // // ����߼�
    // if (currentAvgSpeedCmps > MIN_LONGITUDINAL_DETECT_SPEED_CMPS)
    // {
    //     if (theoreticalAccelMps2 > 0.1f && accelDiff > LONGITUDINAL_ACCEL_DIFF_THRESHOLD)
    //     {
    //         wheelSlipFlags |= SLIP_FLAG_LONGITUDINAL_ACCEL; // ���ٻ��ƣ������ֿ�ת��
    //     }
    //     else if (theoreticalAccelMps2 < -0.1f && accelDiff > LONGITUDINAL_ACCEL_DIFF_THRESHOLD)
    //     {
    //         wheelSlipFlags |= SLIP_FLAG_LONGITUDINAL_DECEL; // ���ٻ��ƣ���̥������
    //     }
    // }
}

void Slip_Check(void)
{

    UpdateWheelSpeeds(speed_left, speed_right);
    DetectLongitudinalSlip();
    DetectLateralSlip();
}