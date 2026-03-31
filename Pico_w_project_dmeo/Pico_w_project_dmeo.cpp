#include "ds4_on_pico_w.hpp"
#include "pico/stdlib.h"

#include <algorithm>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

#define LOG_HEADER "[main]"

/* ---------- define for MPU9250 ---------- */
#define SPI_PORT_MPU spi1
#define READ_BIT 0x80

#define PIN_MISO_MPU 8
#define PIN_CS_MPU   9
#define PIN_SCK_MPU  10
#define PIN_MOSI_MPU 11
/* ---------------------------------------- */

/* ----------- define for L6470 ----------- */
#define SPI_PORT_L6470 spi0
#define Front_BIT 0x51
#define Back_BIT 0x50
#define Stop_BIT 0xB0

#define PIN_MISO_L6470 3
#define PIN_CS_L6470 5
#define PIN_SCK_L6470 2
#define PIN_MOSI_L6470 4

// defining parameter addresses
#define L6470_ABS_POS       0x01 // 現在位置
#define L6470_EL_POS        0x02 // 電気的位置
#define L6470_MARK          0x03 // マーク（絶対）位置
#define L6470_SPEED         0x04 // 現在の速度
#define L6470_ACC           0x05 // 加速度
#define L6470_DEC           0x06 // 減速度
#define L6470_MAX_SPEED     0x07 // 最大速度
#define L6470_MIN_SPEED     0x08 // 最小速度
#define L6470_FS_SPD        0x15 // フルステップ速度のしきい値
#define L6470_KVAL_HOLD     0x09 // 待機中のKVAL値!
#define L6470_KVAL_RUN      0x0A // 定速のKVAL値!
#define L6470_KVAL_ACC      0x0B // 加速開始のKVAL値!
#define L6470_KVAL_DEC      0x0C // 減速開始のKVAL値!
#define L6470_INT_SPD       0x0D // 傾斜の交差速度 
#define L6470_ST_SLP        0x0E // 始動スロープ!
#define L6470_FN_SLP_ACC    0x0F // 加速終端スロープ!
#define L6470_FN_SLP_DEC    0x10 // 減速終端スロープ!
#define L6470_K_THERM       0x11 // 温度補償定数
#define L6470_ADC_OUT       0x12 // ADC出力
#define L6470_OCD_TH        0x13 // 過電流しきい値!
#define L6470_STALL_TH      0x14 // 失速（脱調）のしきい値!
#define L6470_STEP_MODE     0x16 // ステップモード!
#define L6470_ALARM_EN      0x17 // アラーム有効
#define L6470_CONFIG        0x18 // コンフィグ!
#define L6470_STATUS        0x19 // ステータス
/* ---------------------------------------- */

/* --------- define used for calc ---------- */
#define accel_se     0.061
#define gyro_se      0.00763    // for gyro value correction
#define micro        0.000001   // pow(10.0, -6.0)
#define Rad          M_PI / 180.0  // for RAD conversion
#define Step_convert 0.0149011  // For converting the input value to the motor to step/s
#define feedback     0.1        // for feedback
#define feed_step    71205      // 内部速度から時間あたりのステップ数??への変換用

#define MAX_SPEED    5000      // Max motor output
#define START_SPEED  2000       // min motor output
#define VST          0.2        // target velocity of the sphere(m/s)
#define Wheel_R      30         // Enter wheel radius in centimeters(ホイールの半径、センチで入力)
#define Sphere_R     150        // Enter sphere radius in centimeters(球体の半径、センチで入力)
#define milli        0.001      // convert centimeters to meters
/* ----------------------------------------- */

// 状態の定義
enum MoveState {
    IDLE,        // 待機状態
    MOVING_W     // Wキーで前進中
};

// 動作パラメータの定義
const float TARGET_DISTANCE_W = 3.0f;     // 目標距離 (m)
const float VST_W_MAX = 0.5f;             // 最大目標速度 (m/s)
const float VST_W_MIN = 0.05f;            // 最低目標速度 (m/s) これ以下になると停止
const float SLOWDOWN_START_DISTANCE = 2.5f; // この距離から減速を開始 (m)


static inline void cs_select_MPU() 
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS_MPU, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect_MPU() 
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS_MPU, 1);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_select_L6470()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS_L6470, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect_L6470()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS_L6470, 1);
    asm volatile("nop \n nop \n nop");
}

//センサのリセット信号を送る関数
static void mpu9250_reset() 
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6A, 0x03};
    cs_select_MPU();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_MPU, buf, 2);
    cs_deselect_MPU();
    sleep_us(1);

    buf[0]=0x6B;
    buf[1]=0x00;
    cs_select_MPU();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_MPU, buf, 2);
    cs_deselect_MPU();
    sleep_us(1);

    buf[0]=0x6C;
    buf[1]=0x00;
    cs_select_MPU();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_MPU, buf, 2);
    cs_deselect_MPU();
    sleep_us(1);

    buf[0]=0x1B;
    buf[1]=0x00;
    cs_select_MPU();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_MPU, buf, 2);
    cs_deselect_MPU();
    sleep_us(1);

    buf[0]=0x1C;
    buf[1]=0x00;
    cs_select_MPU();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_MPU, buf, 2);
    cs_deselect_MPU();
    sleep_us(1);
}

//センサのレジスタのIDを読み出す関数
static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) 
{
    reg |= READ_BIT;
    cs_select_MPU();
    spi_write_blocking(SPI_PORT_MPU, &reg, 1);
    sleep_us(1);
    spi_read_blocking(SPI_PORT_MPU, 0, buf, len);
    cs_deselect_MPU();
    sleep_us(1);
}

//センサの加速度、ジャイロ、温度を読み出す関数
static void mpu9250_read_raw1(int16_t acceleration2[], int16_t gyro2[], int16_t *temp2) 
{
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    read_registers(0x3B, buffer, 6);

    for (int i = 0; i < 3; i++)
    {
        acceleration2[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    read_registers(0x43, buffer, 6);

    for (int i = 0; i < 3; i++)
    {
        gyro2[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now temperature from reg 0x41 for 2 bytes
    read_registers(0x41, buffer, 2);

    *temp2 = buffer[0] << 8 | buffer[1];
}


//センサの加速度、ジャイロ、温度を読み出す関数(offset)
static void mpu9250_read_raw2(int32_t gy_offset[], int16_t acceleration[], int16_t gyro[], int16_t& temp) 
{
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    read_registers(0x3B, buffer, 6);

    for (int i = 0; i < 3; i++) {
        acceleration[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);//[0,1,2]=[x,y,z]　他のセンサ値も同様
    }

    // Now gyro data from reg 0x43 for 6 bytes
    read_registers(0x43, buffer, 6);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    read_registers(0x41, buffer, 2);

    temp = buffer[0] << 8 | buffer[1];
    
    for(int i = 0; i<3 ; i++){
        //acceleration[i] -= ac_offset[i];
        gyro[i] -= gy_offset[i];
    }
}


void mpu9250_calibration(int32_t gy_offset[], int16_t acceleration2[], int16_t gyro2[], int16_t *temp2)
{
    // ここで初期化する
    gy_offset[0] = 0;
    gy_offset[1] = 0;
    gy_offset[2] = 0;
    /*printf("---calibration() Start-----\n");
    printf("ac_offset0. X = %6d, Y = %6d, Z = %6d\n",ac_offset[0],ac_offset[1],ac_offset[2]);
    printf("gy_offset0. X = %6d, Y = %6d, Z = %6d\n",gy_offset[0],gy_offset[1],gy_offset[2]);   */
    for(int t=0; t<10000; t++){
        mpu9250_read_raw1(acceleration2, gyro2, temp2);
        for(int i=0;i<3;i++){
            //ac_offset[i] += acceleration2[i]; 
            gy_offset[i] += gyro2[i];
        }
        /*if (t%1000 == 0){
            printf("acce1eration. X = %6d, Y = %6d, Z = %6d\n",acceleration[0],acceleration[1],acceleration[2]);
            printf("gyro. X = %6d, Y = %6d, Z = %6d\n",gyro[0],gyro[1],gyro[2]);
            printf("ac_offset1. X = %6d, Y = %6d, Z = %6d\n",ac_offset[0],ac_offset[1],ac_offset[2]);
            printf("gy_offset1. X = %6d, Y = %6d, Z = %6d\n",gy_offset[0],gy_offset[1],gy_offset[2]);   
        }   */
    }
    for(int i=0;i<3;i++){
        //ac_offset[i] = ac_offset[i]/1000;
        gy_offset[i] = gy_offset[i]/10000;
    }
    /*printf("ac_offset2. X = %3f, Y = %3f, Z = %3f\n",ac_offset[0],ac_offset[1],ac_offset[2]);
    printf("gy_offset2. X = %3f, Y = %3f, Z = %3f\n",gy_offset[0],gy_offset[1],gy_offset[2]);
    printf("---calibration() Finish-----\n");   */
}

//L6470リセット時に使用する関数。ただ信号送信するだけ。
void L6470_setting(uint8_t buf[3],uint8_t value[3])
{
    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,buf,3);
    cs_deselect_L6470();
    sleep_us(1);

    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,value,3);
    cs_deselect_L6470();
    sleep_us(1);

    return;
}

//モータドライバのリセット用関数
void L6470_reset()  
{
    uint8_t buf[3]={0xC0,0xC0,0xC0};//レジスタアドレス用
    uint8_t value[3]={0x00,0x00,0x00};//レジスタ値入力用

    L6470_setting(buf,value);

    //Kvalの設定　詳しくはL6470データシートp38
    //0x29=1.92V
    //0x36=2.5V
    //0x40=3V
    //0x0A 稼働時のKval設定
    
    for(int i = 0; i < 3; i++){//最大速度設定
        buf[i] = L6470_MAX_SPEED;
        value[i]=0x03;
    }
    cs_select_L6470();//送るバイト数が18bitなので、関数を使わず手動でデータ送信。
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,buf,3);
    cs_deselect_L6470();
    sleep_us(1);

    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,value,3);
    cs_deselect_L6470();
    sleep_us(1);
    for(int i = 0; i < 3; i++){
        value[i]=0xFF;
    }
    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,value,3);
    cs_deselect_L6470();
    sleep_us(1);

    for(int i = 0; i < 3; i++){//停止時のKval値設定
        buf[i] = L6470_KVAL_HOLD;
        value[i]=0x5A;
    }
    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//走行時のKval値設定
        buf[i] = L6470_KVAL_RUN;
    }
    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//加速時のKval値設定
        buf[i] = L6470_KVAL_ACC;
    }
    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//減速時のKval値設定
        buf[i] = L6470_KVAL_DEC;
    }
    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//始動の傾斜の設定
        buf[i]=0x0E;
        value[i]=0x19;
    }
    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//加速の最終傾斜の設定
        buf[i]=0x0F;
        value[i]=0x29;
    }
    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//減速の最終傾斜の設定
        buf[i]=0x10;
        value[i]=0x29;
    }
    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//ステップモードの設定
        buf[i]=0x16;
        value[i]=0x03;
    }
    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//過電流のしきい値の設定
        buf[i]=0x13;
        value[i]=0x0F;
    }

    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//失速（脱調）のしきい値の設定
        buf[i]=L6470_STALL_TH;
        value[i]=0x40;
    }

    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//デバイスの設定
        buf[i]=0x18;
        value[i]=0x2E;
    }

    cs_select_L6470();//送るバイト数が16bitなので、関数を使わず手動でデータ送信。
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,buf,3);
    cs_deselect_L6470();
    sleep_us(1);

    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,value,3);
    cs_deselect_L6470();
    sleep_us(1);

    for(int i = 0; i < 3; i++){
        value[i]=0x88;
    }

    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,value,3);
    cs_deselect_L6470();
    sleep_us(1);

    for(int i = 0; i < 3;i++)   //モータの加速度の設定
    {
        buf[i]=0x05;
        value[i]=0x20;
    }
    L6470_setting(buf,value); 
}

//w1,2,3は回転スピード
void L6470_move(int w1,int w2,int w3) 
{
    uint8_t rotate[]={Stop_BIT,Stop_BIT,Stop_BIT};//モーターの正転、逆転を管理する行列
    uint8_t nullsp[]={0,0,0};//スピード送信時にはじめに送る空の行列
    uint8_t firstsp[]={0,0,0};//スピード送信用の行列 始めの8bit
    uint8_t lastsp[]={0,0,0};//後ろの8bit 合計16bit
    uint16_t speed[]={0,0,0};//回転のスピード,最大値65535まで
    //正ならば正回転、負ならば逆回転、0ならば停止
    if(w1>0){rotate[0]=Front_BIT;}
    else if(w1<0){rotate[0]=Back_BIT;}
    else{rotate[0]=Stop_BIT;}

    speed[0]=abs(w1);

    if(w2>0){rotate[1]=Front_BIT;}
    else if(w2<0){rotate[1]=Back_BIT;}
    else{rotate[1]=Stop_BIT;}

    speed[1]=abs(w2);

    if(w3>0){rotate[2]=Front_BIT;}
    else if(w3<0){rotate[2]=Back_BIT;}
    else{rotate[2]=Stop_BIT;}

    speed[2]=abs(w3);
    
    for(int i=0;i<3;i++)
    {
        firstsp[i]= speed[i] >> 8;//16bitのデータを8つ分だけ右にシフトさせて、頭の8bitだけを取り出した。
        lastsp[i]= (255)&(speed[i]);//16bitのデータとb'00001111'の論理積をして後ろの8bitだけ取り出した。
    }

    /*
    printf("F%x\n",firstsp[0]);
    printf("L%x\n",lastsp[0]);
    printf("R%x\n",rotate[0]);

    printf("F%x\n",firstsp[1]);
    printf("L%x\n",lastsp[1]);
    printf("R%x\n",rotate[1]);

    printf("F%x\n",firstsp[2]);
    printf("L%x\n",lastsp[2]);
    printf("R%x\n",rotate[2]);
    */

    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,rotate,3);
    cs_deselect_L6470();
    sleep_us(1);

    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,nullsp,3);
    cs_deselect_L6470();
    sleep_us(1);

    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,firstsp,3);
    cs_deselect_L6470();
    sleep_us(1);

    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470,lastsp,3);
    cs_deselect_L6470();
}

void L6470_manualstop(int& Ypulse_num, int& Xpulse_num)
{
    uint8_t buf[] = {Stop_BIT, Stop_BIT, Stop_BIT};
    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470, buf, 3);
    cs_deselect_L6470();

    Xpulse_num = 0;
    Ypulse_num = 0;
}


void calculate_XY_Velocity(float direction, float Vst, float& Vst_y, float& Vst_x)
{
    // convert angle to radians(角度をラジアンに変換)
    float angle_in_radians = direction * M_PI / 180.0;

    // calculate the components of X and Y(XとYの成分を計算)
    Vst_x = Vst * cos(angle_in_radians);
    Vst_y = Vst * sin(angle_in_radians);
}

void P_ctrl_Y(float Kp, float Vst_Y, float Vsy, float& cor_P)
{

    float Vdiff_Y  = 0;    // deviation (偏差)
    Vdiff_Y = Vsy - Vst_Y;  // Deviation between the target speed and current speed of the sphere (球体の目標速度と現在速度の偏差)
    cor_P   = Kp * Vdiff_Y; // Determine the amount of operation correction (操作補正量の決定)

}
void P_ctrl_X(float Kp, float Vst_X, float Vsx, float& cor_P)
{
    float Vdiff_X  = 0;    // deviation (偏差)
    Vdiff_X = Vsx - Vst_X;  // Deviation between the target speed and current speed of the sphere (球体の目標速度と現在速度の偏差)
    cor_P   = Kp * Vdiff_X; // Determine the amount of operation correction (操作補正量の決定)
}

void D_ctrl_Y(float Kd, float Vst_Y, float Vsy, float& cor_D)
{ 
    static uint32_t prev_time = time_us_32(); // ループ開始前の時間（staticで一度だけ宣言）
    uint32_t now = time_us_32();
    float deltaT = (now - prev_time) / 1e6f; // 秒に変換（μs → s）
    prev_time = now; // 次回用に保存
    //const float deltaT = 450 * pow(10.0, -9.0); //1 cycle of program[μs](プログラムの1周期)
    static float Vdiff_X_before = 0;
    float Vdiff_X = 0; //偏差
    float Vdd_X = 0;
    Vdiff_X = Vsy - Vst_Y; //偏差
    Vdd_X = Vdiff_X - Vdiff_X_before;
    cor_D = Kd * ((Vdd_X) / deltaT);
    Vdiff_X_before = Vdiff_X;
}
void D_ctrl_X(float Kd, float Vst_X, float Vsx, float& cor_D)
{ 
    static uint32_t prev_time = time_us_32(); // ループ開始前の時間（staticで一度だけ宣言）
    uint32_t now = time_us_32();
    float deltaT = (now - prev_time) / 1e6f; // 秒に変換（μs → s）
    prev_time = now; // 次回用に保存
    //const float deltaT = 450 * pow(10.0, -9.0); //1 cycle of program[μs](プログラムの1周期)
    static float Vdiff_X_before = 0;
    float Vdiff_X = 0; //偏差
    float Vdd_X = 0;
    Vdiff_X = Vsx - Vst_X; //偏差
    Vdd_X = Vdiff_X - Vdiff_X_before;
    cor_D = Kd * ((Vdd_X) / deltaT);
    Vdiff_X_before = Vdiff_X;
}

void cor_tot(float cor_P_Y, float cor_D_Y, float cor_P_X, float cor_D_X, float& cor_PD_Y, float& cor_PD_X)
{
    // Initial value setting(初期値設定)
    static float previous_cor_PD_Y = 0;//(前回のcor_tot)
    static float previous_cor_PD_X = 0;//(前回のcor_tot)
    cor_PD_Y = cor_P_Y + cor_D_Y + previous_cor_PD_Y;
    cor_PD_X = cor_P_X + cor_D_X + previous_cor_PD_X;
    previous_cor_PD_Y = cor_PD_Y;
    previous_cor_PD_X = cor_PD_X;
}

// 振動制御：Y方向の成分のみを計算して返す
float get_vib_Y(float degree)
{
    if (-2 < degree && degree < 2)
    {
        return 0.0f;
    }
    else
    {
        // 元のコード: w1 = -2000 * 9.8 * sin... * (sqrt(3)/2) でした。
        // 分配関数で (sqrt(3)/2) が掛かるため、ここでは「元の強さ」だけを返します。
        // 元のw1への寄与がマイナスだったので、Y成分としてもマイナスとします。
        return 2000.0f * 9.8f * sin(degree * Rad);
    }
}

// 振動制御：X方向の成分のみを計算して返す
float get_vib_X(float degree)
{
    if (-2 < degree && degree < 2)
    {
        return 0.0f;
    }
    else
    {
        // 元のコード: w3 = -2000 * 9.8 * sin...
        // X成分はw3にそのまま(係数1で)効くため、この値を返します。
        return -2000.0f * 9.8f * sin(degree * Rad);
    }
}

void yaw_angle_control(float Target_angle, float current_angle, float yaw_Kp, float yaw_kd, int& yaw_output)
{
    float angle_diff = Target_angle - current_angle;
    if (angle_diff > -1 && angle_diff < 1) // 目標角度と現在角度の差が±1°以内ならば、0にする
    {
        angle_diff = 0;
    }
    // 差分を ±180° にラップ（これが超重要！）
    if (angle_diff > 180.0f)  angle_diff -= 360.0f;
    if (angle_diff < -180.0f) angle_diff += 360.0f;

    // 微分制御用の時間差
    static uint32_t prev_time = time_us_32();
    uint32_t now = time_us_32();
    float deltaT = (now - prev_time) / 1e6f;
    prev_time = now;

    // 微分成分
    static float angle_diff_before = 0;
    float angle_dd = angle_diff - angle_diff_before;
    angle_diff_before = angle_diff;

    // P + D制御
    float result = yaw_Kp * angle_diff + yaw_kd * (angle_dd / deltaT);
    if (result > MAX_SPEED) result = MAX_SPEED;
    if (result < -MAX_SPEED) result = -MAX_SPEED;
    
    // intに変換（範囲制限は必要ならここで）
    yaw_output = -(int)result;
}



// --- 【新規】X, Y, Yaw の速度成分を3つのモーター指令値に分配する関数 ---
// 教授の指示「最後に分配する」を実行する場所です。
// 修論の式(3.4.3)に基づき、座標系の値を各モーターの回転数に変換します。
void distribute_to_motors(float input_X, float input_Y, int yaw_output, int& w1, int& w2, int& w3)
{
    // Y成分の符号反転処理
    // 元のコードで (-cor_PD_Y) となっていたため、入力Yを反転させて計算に使用します。
    // ※もし前後進が逆になる場合は、ここを positive に戻してください。
    float eff_Y = -input_Y; 

    // w1: 左前 (-0.5X + √3/2Y)
    w1 = (int)(-input_X * 0.5f + eff_Y * 0.866025f) + yaw_output;
    
    // w2: 右前 (-0.5X - √3/2Y)
    w2 = (int)(-input_X * 0.5f - eff_Y * 0.866025f) + yaw_output;
    
    // w3: 後ろ (X)
    w3 = (int)(input_X) + yaw_output;
}

/**
 * @brief 6軸IMUデータから姿勢角を計算する。
 * @param acceleration  加速度センサーの生データ配列 [x, y, z]
 * @param gyro          ジャイロセンサーの生データ配列 [x, y, z] (バイアス補正済み)
 * @param Vgx           (出力) ワールド座標系でのX軸周りの角速度 [rad/s]
 * @param Vgy           (出力) ワールド座標系でのY軸周りの角速度 [rad/s]
 * @param comp_x        (出力) 計算されたロール角 [deg]
 * @param comp_y        (出力) 計算されたピッチ角 [deg]
 * @param angle_gz      (入出力) 計算されたヨー角 [deg]
 */
void degree_calc(int16_t acceleration[], int16_t gyro[],
                 float& Vgx, float& Vgy,
                 float& comp_x, float& comp_y, float& angle_gz)
{
    // ---- 状態を保持する変数 ----
    static float Roll = 0.0f;  // 現在のロール角 [deg]
    static float Pitch = 0.0f; // 現在のピッチ角 [deg]
    
    static float prev_roll_estimate = 0.0f;
    static float prev_pitch_estimate = 0.0f;

    // ---- 時間差(dt)の計算 ----
    static uint32_t timestamp = 0;
    if (timestamp == 0) {
        timestamp = time_us_32();
        return;
    }
    uint32_t now = time_us_32();
    float dt = (now - timestamp) / 1000000.0f; // マイクロ秒を秒に変換
    timestamp = now;

    // ---- 定数 ----
    // gyro_se はこの関数の外側で定義されているグローバル定数と仮定
    const float k = 0.985f;
    const float DEG_TO_RAD = M_PI / 180.0f;
    const float RAD_TO_DEG = 180.0f / M_PI;

    // ---- センサー値の変換 ----
    float gx = gyro[0] * gyro_se;
    float gy = gyro[1] * gyro_se;
    float gz = gyro[2] * gyro_se;
    
    // ---- ジャイロによる角度変化の計算 ----
    // 機体の傾きを考慮して、ワールド座標系での角速度に変換
    float roll_rate  = gx + sin(Roll * DEG_TO_RAD) * tan(Pitch * DEG_TO_RAD) * gy + cos(Roll * DEG_TO_RAD) * tan(Pitch * DEG_TO_RAD) * gz;
    float pitch_rate = cos(Roll * DEG_TO_RAD) * gy - sin(Roll * DEG_TO_RAD) * gz;
    
    // 角度の変化量 [deg] を計算
    float delta_angle_roll  = roll_rate * dt;
    float delta_angle_pitch = pitch_rate * dt;

    // ---- 加速度センサーによる角度の計算 ----
    float accel_angle_roll  = -atan2(-acceleration[1], acceleration[2]) * RAD_TO_DEG;
    float accel_angle_pitch = -atan2(acceleration[0], sqrt(pow((float)acceleration[1], 2) + pow((float)acceleration[2], 2))) * RAD_TO_DEG;

    // ---- 相補フィルターによるロール・ピッチの統合 ----
    comp_x = k * (prev_roll_estimate + delta_angle_roll) + (1.0f - k) * accel_angle_roll;
    comp_y = k * (prev_pitch_estimate + delta_angle_pitch) + (1.0f - k) * accel_angle_pitch;

    // 計算した現在の角度を、次回の計算で使うための変数に反映させる
    Roll = comp_x;
    Pitch = comp_y;
    
    // 次回計算用に、今回の計算結果を保存する
    prev_roll_estimate = comp_x;
    prev_pitch_estimate = comp_y;

    // ---- ヨーの計算 (ご指定の単純積分) ----
    float delta_yaw = gz * dt;
    angle_gz += delta_yaw;
    
    // ヨー角を 0～360° の範囲に正規化
    angle_gz = fmodf(angle_gz, 360.0f);
    if (angle_gz < 0.0f) {
        angle_gz += 360.0f;
    }

    // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
    // ★★★ ここからがオフセット補正の追加箇所 ★★★
    // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★

    // ---- オフセット値の定義 ----
    // 事前に水平な場所に置いて測定した誤差の値を設定する
    const float roll_offset = 2.5f;  // 例：ロール角が4度ずれている場合
    const float pitch_offset = 0.5f; 

    // ---- 出力用の引数に値を代入 ----
    // フィルターで計算した値から、オフセット値を差し引く
    comp_x = comp_x - roll_offset;
    comp_y = comp_y - pitch_offset;
    
    // angle_gz は既に関数内で更新済み
    Vgx = roll_rate * DEG_TO_RAD;
    Vgy = pitch_rate * DEG_TO_RAD;
}

// --- 【置き換え】オドメトリ関数（中身を単純化） ---
// 以前の sphere_speed は削除し、これを代わりに使ってください。
// 指令値(command_Y, command_X)をそのまま速度計算に使います。
void sphere_speed(float command_Y, float command_X, float Vgy, float Vgx, float& Vsy, float& Vsx)
{
    // 指令値を回転角速度[deg/s]へ変換
    float Vry = command_Y * Step_convert * 1.8f;
    float Vrx = command_X * Step_convert * 1.8f; 
    
    // 速度[m/s]へと変換
    Vry = Vry * Rad * Wheel_R * milli;
    Vrx = Vrx * Rad * Wheel_R * milli;

    // 球体速度の合成
    Vsy = Vry + (Sphere_R * milli * Vgx);
    Vsx = Vrx + (Sphere_R * milli * Vgy);
}

/* ============================ Controller functions ============================ */

/**
 * @brief DualShock 4のスティック状態を保持する構造体  
 * Structure to hold the state of the DualShock 4 analog sticks.
 * 
 * @details  
 * L3スティックとR3スティックのX/Y軸の値（正規化済み）を保持します。  
 * Holds normalized values (-1.0 to 1.0) for the X and Y axes of the L3 and R3 sticks.
 */
struct StickState 
{
    float l3_x;
    float l3_y;
    float r3_x;
    float r3_y;
};

/**
 * @brief Normalize the value to a range of -1.0 to 1.0.
 * @param value The value to normalize.
 * @return float Normalized value.
 */
float normalize_stick_axis(int value) 
{
    return (value - 127.5f) / 127.5f;
}

/**
 * @brief 
 * DualShock 4のスティック入力を正規化して返します。
 * 与えられた `DualShock4_state` 構造体の各スティック軸（L3およびR3）を-1.0 ～ 1.0 の範囲に正規化し、`StickState` 構造体として返します。
 * 
 * Normalize DualShock 4 stick input values and return them.
 * Normalize each stick axis (L3 and R3) from the given `DualShock4_state`  
 * to the range -1.0 to 1.0 and return as a `StickState` structure.
 * 
 * @param state 
 * DualShock 4のスティック生データ（整数）Raw stick input values from DualShock 4 (integers)
 * 
 * @return 
 * tickState 正規化されたスティックの状態（float） Normalized stick state as floating point values
 */
StickState normalize_sticks(const DualShock4_state& state) 
{
    return 
    {
        .l3_x = normalize_stick_axis(state.l3_x),
        .l3_y = normalize_stick_axis(state.l3_y),
        .r3_x = normalize_stick_axis(state.r3_x),
        .r3_y = normalize_stick_axis(state.r3_y),
    };
}

float Calc_target_yaw_from_rstick(float r3_x, float r3_y)
{
    // 右スティックのx/yから角度を算出（前が0°）
    float angle = atan2f(-r3_x, -r3_y) * (180.0f / M_PI); // Y軸前方向を0°に
    if (angle < 0) angle += 360.0f; // 0〜360° に正規化
    return angle;
}

/* ============================================================================== */
    

int main()
{
    // 状態管理と距離計算のための変数
    float total_distance_x = 0.0f;
    float total_distance_y = 0.0f;
    uint32_t loop_timer = 0;

    // 制御関連の変数
    float comp_x = 0, comp_y = 0;
    float Vgx = 0, Vgy = 0;
    float Vsy = 0, Vsx = 0;
    float Vst_y = 0, Vst_x = 0;

    // センサー関連の変数
    int32_t gy_offset[3] = {0}; // ゼロで初期化する
    int16_t acceleration[3] = {0};
    int16_t gyro[3] = {0};
    int16_t temp = 0;
    int16_t acceleration2[3] = {0};
    int16_t gyro2[3] = {0};
    int16_t temp2;

    // PID制御関連の変数
    float cor_P_Y = 0, cor_D_Y = 0;
    float cor_P_X = 0, cor_D_X = 0;
    float cor_PD_Y = 0, cor_PD_X = 0;
    float Kp_value = 15.0f;
    float Kd_value = 0.00165f;

    // モーター出力関連の変数
    int synt_vec_Y = 0, synt_vec_X = 0;
    float target_yaw = 0;
    float Yaw = 0;
    int yaw_output = 0;
    int PD_w1 = 0, PD_w2 = 0, PD_w3 = 0; 
    int w1_Y = 0, w2_Y = 0, w1_X = 0, w2_X = 0, w3_X = 0; 

    uint32_t timestamp = 0;

    // 計測の状態を管理する
    enum MeasurementState {
        NOT_MEASURING,  // 0: 計測待機中
        MEASURING,      // 1: 10秒間計測中
        MEASUREMENT_DONE // 2: 計測完了
    };

    MeasurementState measurement_state = NOT_MEASURING; // 現在の計測状態
    uint32_t measurement_start_time = 0;    // 計測開始時刻 (us)
    float measurement_start_x = 0.0f;       // 計測開始時のX距離
    float measurement_start_y = 0.0f;       // 計測開始時のY距離

    // 計測の条件
    const float TARGET_VELOCITY_FOR_MEASUREMENT = VST_W_MAX; // この速度に達したら計測開始
    const float MEASUREMENT_DURATION_S = 10.0f;              // 10秒間計測

    MoveState move_state = IDLE;
    float start_pos_y_w = 0.0f;

    ////////////////////////////////////////////
    // SETUP
    ////////////////////////////////////////////

    stdio_init_all();
    sleep_ms(5000);
    printf("======================\n[SETUP] PicoW Project\n======================\n");
    
    uart_init(uart0,115200);
    gpio_set_function(0,GPIO_FUNC_UART);
    gpio_set_function(1,GPIO_FUNC_UART);
    uart_puts(uart0, "Hello, MPU9250! Reading raw data from registers via SPI...\n");

    spi_init(SPI_PORT_MPU, 1000 * 1000);
    gpio_set_function(PIN_MISO_MPU, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK_MPU, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI_MPU, GPIO_FUNC_SPI);
    bi_decl(bi_3pins_with_func(PIN_MISO_MPU, PIN_MOSI_MPU, PIN_SCK_MPU, GPIO_FUNC_SPI));

    gpio_init(PIN_CS_MPU);
    gpio_set_dir(PIN_CS_MPU, GPIO_OUT);
    gpio_put(PIN_CS_MPU, 1);
    bi_decl(bi_1pin_with_name(PIN_CS_MPU, "SPI CS1"));

    spi_init(SPI_PORT_L6470, 4000*1000);
    gpio_set_function(PIN_MISO_L6470, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK_L6470, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI_L6470, GPIO_FUNC_SPI);
    bi_decl(bi_3pins_with_func(PIN_MISO_L6470, PIN_MOSI_L6470, PIN_SCK_L6470, GPIO_FUNC_SPI));

    gpio_init(PIN_CS_L6470);
    gpio_set_dir(PIN_CS_L6470, GPIO_OUT);
    gpio_put(PIN_CS_L6470, 1);
    bi_decl(bi_1pin_with_name(PIN_CS_L6470, "SPI CS0"));

    mpu9250_reset();
    L6470_reset();
    mpu9250_calibration(gy_offset, acceleration2, gyro2, &temp2);

    uint8_t id;
    read_registers(0x75, &id, 1);
    printf("I2C address is 0x%x\n", id);

    gpio_init(16);
    gpio_set_dir(16,GPIO_IN);
    gpio_pull_up(16);
    gpio_init(17);
    gpio_set_dir(17,GPIO_OUT);
    gpio_put(17,1);
    sleep_ms(100);
    
    uart_puts(uart0, "AccX, Y, Z, GyroX, Y, Z, Temp\n");

    while (1) 
    {
        if(gpio_get(16) == true)
        {
            gpio_put(17,1);
            sleep_ms(500);
            gpio_put(17,0);
            sleep_ms(500);
            uint8_t buf[] = {Stop_BIT, Stop_BIT, Stop_BIT};
            cs_select_L6470();
            sleep_us(1);
            spi_write_blocking(SPI_PORT_L6470, buf, 3);
            cs_deselect_L6470();
        }
        else
        {
            gpio_put(17, 1);
            
            uint32_t now = time_us_32();
            float deltaT = (now - loop_timer) / 1e6f;
            loop_timer = now;
            
            // 1. キー入力のチェック (ノンブロッキング)
            int c = getchar_timeout_us(0);

            // 2. 状態に応じて動作を決定
            switch (move_state) {
                case IDLE:
                    // 待機中に'w'が押されたら、移動開始
                    if (c == 'w') {
                        move_state = MOVING_W;
                        start_pos_y_w = total_distance_y; // 開始時のY座標を記録
                        printf("Wキー入力\n");
                    } else {
                        // 待機中は停止
                        Vst_y = 0.0f;
                        Vst_x = 0.0f;

                        // 停止したら、次の計測に備えてリセット
                        measurement_state = NOT_MEASURING;
                    }
                    break;

                // main関数内の switch(move_state) の中
                case MOVING_W:
                    // とにかく最大速度で走り続ける
                    Vst_y = VST;

                    // 現在の計算上の移動距離を表示するだけ
                    float distance_traveled = fabsf(total_distance_y - start_pos_y_w);
                    
                    // printfで現在の計算上の距離を確認する
                    // 例: printf("Calculated Distance: %.3f m\n", distance_traveled);
                    if (distance_traveled >= 2.0)
                    {
                        Vst_y = 0.0f;
                        move_state = IDLE;
                        
                        // 手動停止でも、次の計測に備えてリセット
                        measurement_state = NOT_MEASURING;
                    }
                    
                    
                    // 's'キーが押されたら止まる（手動停止用）
                    else if (c == 's') {
                        Vst_y = 0.0f;
                        move_state = IDLE;
                        
                        // 手動停止でも、次の計測に備えてリセット
                        measurement_state = NOT_MEASURING;
                    }
                    break;
            }
            
            // 目標の向きは現在の向きを維持
            target_yaw = Yaw;

            P_ctrl_Y(Kp_value, Vst_y, Vsy, cor_P_Y);
            D_ctrl_Y(Kd_value, Vst_y, Vsy, cor_D_Y);
            P_ctrl_X(Kp_value, Vst_x, Vsx, cor_P_X);
            D_ctrl_X(Kd_value, Vst_x, Vsx, cor_D_X);
            cor_tot(cor_P_Y, cor_D_Y, cor_P_X, cor_D_X, cor_PD_Y, cor_PD_X);

            // ========================================================
            // ★ここから修正：古い関数を消して、新しいロジックにする
            // ========================================================

            // 1. 振動制御量の計算 (上で定義した新しい関数を使用)
            // Y軸制御の振動は comp_x (Roll) に依存
            float vib_val_Y = get_vib_Y(comp_x);
            // X軸制御の振動は comp_y (Pitch) に依存
            float vib_val_X = get_vib_X(comp_y);

            // 2. 全指令値の合成 (PID + 振動)
            // これがロボットが動こうとする「真のX, Y方向の強さ」です
            float Total_Command_Y = cor_PD_Y + vib_val_Y;
            float Total_Command_X = cor_PD_X + vib_val_X;

            // 3. Yaw制御 (必要ならコメントイン)
            // yaw_angle_control(target_yaw, Yaw, 1000, 0.1, yaw_output);

            // 4. 【重要】最後にまとめてモーターへ分配
            // ここで初めて3つのタイヤの値(PD_w1~3)が決まります
            distribute_to_motors(Total_Command_X, Total_Command_Y, yaw_output, PD_w1, PD_w2, PD_w3);

            // 5. モータードライバへ送信
            L6470_move(PD_w1, PD_w2, PD_w3);
            
            // 6. センサー読み取り & 姿勢角計算
            mpu9250_read_raw2(gy_offset, acceleration, gyro, temp);
            degree_calc(acceleration, gyro, Vgx, Vgy, comp_x, comp_y, Yaw);

            // 7. オドメトリ (速度算出)
            // 古い synt_vec_Y ではなく、合成した指令値(Total_Command)を使います
            sphere_speed(-Total_Command_Y, -Total_Command_X, Vgy, Vgx, Vsy, Vsx);
            
            // 距離積算
            total_distance_x += Vsx * deltaT;
            total_distance_y += Vsy * deltaT;


            if (time_us_32() - timestamp > 100000)
            {
                //printf("comp_x = %2f, ", comp_x);
                //printf("comp_y = %2f, ", comp_y);
                //printf("Yaw_T = %f, ", target_yaw);
                //printf("Yaw = %f, ", Yaw);
                //printf("Dist_X:%.2fm, Dist_Y:%.2fm", total_distance_x, total_distance_y);
                //printf("Vsy:%.2fm/s, Vsx:%.2fm/s,", Vsy, Vsx);
                //printf("Vst_y:%.2fm/s, Vst_x:%.2fm/s\n", Vst_y, Vst_x);
                //printf("Vgx = %f, ", Vgx);
                //printf("Vgy = %f,", Vgy);
                //printf("synt_vec_Y = %f\n", synt_vec_Y);
                printf("dist = %f,", fabsf(total_distance_y - start_pos_y_w));
                printf("\n");
                timestamp = time_us_32();
            }
        }
    }
    return 0;
}