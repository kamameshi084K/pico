#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

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
#define Rad          0.0174532  // for RAD conversion
#define Step_convert 0.0149011  // For converting the input value to the motor to step/s
#define feedback     0.1        // for feedback
#define feed_step    71205      // 内部速度から時間あたりのステップ数??への変換用

#define MAX_SPEED    14000      // Max motor output
#define START_SPEED  2000       // min motor output
#define VST          0.3        // target velocity of the sphere(m/s)
#define Wheel_R      30         // Enter wheel radius in centimeters(ホイールの半径、センチで入力)
#define Sphere_R     150        // Enter sphere radius in centimeters(球体の半径、センチで入力)
#define milli        0.001      // convert centimeters to meters
/* ----------------------------------------- */

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
void L6470_setting(u_int8_t buf[3],u_int8_t value[3])
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
    u_int8_t buf[3]={0xC0,0xC0,0xC0};//レジスタアドレス用
    u_int8_t value[3]={0x00,0x00,0x00};//レジスタ値入力用

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
        value[i]=0x50;
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
        value[i]=0x07;
    }
    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//過電流のしきい値の設定
        buf[i]=0x13;
        value[i]=0x0F;
    }

    L6470_setting(buf,value);

    for(int i = 0; i < 3; i++){//失速（脱調）のしきい値の設定
        buf[i]=L6470_STALL_TH;
        value[i]=0x7F;
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
void L6470_move(short w1,short w2,short w3) 
{
    u_int8_t rotate[]={Stop_BIT,Stop_BIT,Stop_BIT};//モーターの正転、逆転を管理する行列
    u_int8_t nullsp[]={0,0,0};//スピード送信時にはじめに送る空の行列
    u_int8_t firstsp[]={0,0,0};//スピード送信用の行列 始めの8bit
    u_int8_t lastsp[]={0,0,0};//後ろの8bit 合計16bit
    u_int16_t speed[]={0,0,0};//回転のスピード,最大値65535まで
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
    u_int8_t buf[] = {Stop_BIT, Stop_BIT, Stop_BIT};
    cs_select_L6470();
    sleep_us(1);
    spi_write_blocking(SPI_PORT_L6470, buf, 3);
    cs_deselect_L6470();

    Xpulse_num = 0;
    Ypulse_num = 0;
}

//球体を揺らさず止まる関数
void L6470_stop(float comp_x, float comp_y, int& Ypulse_num, int& Xpulse_num)
{
    int numx = 0,numy = 0;
    float stop_x = 0, stop_y = 0;
    if(comp_x > 24){//前後の傾き(許容角度オーバー)を判定
        stop_y = 1.0;
        
    }else if(comp_x < -24){
        stop_y = 1.0;
        stop_y = -stop_y ;
        
    }else if(comp_x > 2){//前後の傾き(許容角度内)を判定
        stop_y = ((abs(comp_x) - 2) / 24) * 12 / 11;
        //motor_ctrl_Y(10000*stop_y );
    }else if(comp_x < -2){
        stop_y = ((abs(comp_x) - 2) / 24) * 12 / 11;
        stop_y = -stop_y ;
        
    }else{
        stop_y = 0;
    }

    if(comp_y > 24){//左右の傾き(許容角度オーバー)を判定
        stop_x = 1.0;
        
    }else if(comp_y < -24){
        stop_x = 1.0;
        stop_x = -stop_x ;
        
    }else if(comp_y>2){//左右の傾き(許容角度内)を判定
        stop_x = ((abs(comp_y) - 2) / 24)* 12 / 11;
        //motor_ctrl_X(10000*stop_x );
    }else if(comp_y < -2){
        stop_x = ((abs(comp_y) - 2) / 24)* 12 / 11;
        stop_x = -stop_x ;
        
    }else{
        stop_x = 0;
    }

    numy = 10000 * stop_y;//(-numy,numy,0);
    numx = 10000 * stop_x;//(numx*1/2,numx*1/2,-numx);

    if((numx == 0) && (numy == 0))
    {
        L6470_manualstop(Ypulse_num, Xpulse_num); 
    }else
    {
        L6470_move( -numy + numx * (1.0 / 2.0), numy + numx * (1.0 / 2.0), -numx);//ベクトル合成
    }
}

void calculate_XY_Velocity(float direction, float Vst, float& Vst_y, float& Vst_x)
{
    // convert angle to radians(角度をラジアンに変換)
    float angle_in_radians = direction * M_PI / 180.0;

    // calculate the components of X and Y(XとYの成分を計算)
    Vst_x = Vst * cos(angle_in_radians);
    Vst_y = Vst * sin(angle_in_radians);
}

void P_ctrl_Y(int cnt, float Vst_Y, float Vsy, float& cor_P)
{
    const float Kp = 20;   // Proportional gain (比例ゲイン)
    float Vdiff_Y  = 0;    // deviation (偏差)
    // Captures keystrokes and resets information immediately after pressing.(キー入力を取得し、押した直後に情報をリセット)
    if (cnt == 0)
    {
        Vdiff_Y = 0;
        cor_P   = 0;
    }
    // Further processing(その後の処理)
    else
    {
        Vdiff_Y = Vsy - Vst_Y;  // Deviation between the target speed and current speed of the sphere (球体の目標速度と現在速度の偏差)
        cor_P   = Kp * Vdiff_Y; // Determine the amount of operation correction (操作補正量の決定)
    }
    //printf("cor_P = %3f\n", cor_P);   
}
void P_ctrl_X(int cnt, float Vst_X, float Vsx, float& cor_P)
{
    const float Kp = 25;   // Proportional gain (比例ゲイン)
    float Vdiff_X  = 0;    // deviation (偏差)
    // Captures keystrokes and resets information immediately after pressing.(キー入力を取得し、押した直後に情報をリセット)
    if (cnt == 0)
    {
        Vdiff_X = 0;
        cor_P   = 0;
    }
    // Further processing(その後の処理)
    else
    {
        Vdiff_X = Vsx - Vst_X;  // Deviation between the target speed and current speed of the sphere (球体の目標速度と現在速度の偏差)
        cor_P   = Kp * Vdiff_X; // Determine the amount of operation correction (操作補正量の決定)
    }
    //printf("cor_P = %3f\n", cor_P);   
}

void D_ctrl_Y(int cnt, float Vst_Y, float Vsy, float& cor_D)
{ 
    const float Kd = 0.00165;    //differential gain(微分ゲイン)
    const float deltaT = 450 * pow(10.0, -9.0); //1 cycle of program[μs](プログラムの1周期)
    static float Vdiff_Y_before = 0;
    float Vdiff_Y = 0; //偏差
    float Vdd_Y = 0;
    if (cnt == 0)
    {
        Vdiff_Y_before = 0;
        Vdiff_Y = 0;
        Vdd_Y = 0;
        cor_D = 0;
    }
    else
    {
        Vdiff_Y = Vsy - Vst_Y; //偏差
        Vdd_Y = Vdiff_Y - Vdiff_Y_before;
        //printf("Vdiff_Y = ,%f,", Vdiff_Y);
        //printf("Vdiff_Y_before = ,%f,", Vdiff_Y_before);
        //printf("Vdiff_Y - Vdiff_Y_before =, %f", Vdd_Y);
        cor_D = Kd * ((Vdd_Y) / deltaT);
        Vdiff_Y_before = Vdiff_Y;
        //printf("cor = ,%f,", cor);
        //printf("\n");
    }
}
void D_ctrl_X(int cnt, float Vst_X, float Vsx, float& cor_D)
{ 
    const float Kd = 0.00165;    //differential gain(微分ゲイン)
    const float deltaT = 450 * pow(10.0, -9.0); //1 cycle of program[μs](プログラムの1周期)
    static float Vdiff_X_before = 0;
    float Vdiff_X = 0; //偏差
    float Vdd_X = 0;
    if (cnt == 0)
    {
        Vdiff_X_before = 0;
        Vdiff_X = 0;
        Vdd_X = 0;
        cor_D = 0;
    }
    else
    {
        Vdiff_X = Vsx - Vst_X; //偏差
        Vdd_X = Vdiff_X - Vdiff_X_before;
        cor_D = Kd * ((Vdd_X) / deltaT);
        Vdiff_X_before = Vdiff_X;
    }
}

void cor_tot(int cnt, float cor_P_Y, float cor_D_Y, float cor_P_X, float cor_D_X, float& cor_PD_Y, float& cor_PD_X)
{
    // Initial value setting(初期値設定)
    static float previous_cor_PD_Y = 0;//(前回のcor_tot)
    static float previous_cor_PD_X = 0;//(前回のcor_tot)
    //cnt judgment(cnt判定)
    if (cnt == 0)//Initialize when cnt is 0(cntが0の時、初期化)
    {
        previous_cor_PD_Y = 0;
        previous_cor_PD_X = 0;
        cor_PD_Y = 0;
        cor_PD_X = 0;
    }
    else
    {
        cor_PD_Y = cor_P_Y + cor_D_Y + previous_cor_PD_Y;
        cor_PD_X = cor_P_X + cor_D_X + previous_cor_PD_X;
        previous_cor_PD_Y = cor_PD_Y;
        previous_cor_PD_X = cor_PD_X;
    }
    

}

void motor_calc(float cor_PD_Y, float cor_PD_X, short& PD_w1, short& PD_w2, short& PD_w3, short& synt_vec_Y, short& synt_vec_X)
{
    short SAC_fin = -cor_PD_Y + (-cor_PD_X);

    //おいらが調べたやつ、マイナス方向の制御に難あり
    //PD_w1 = (SAC_fin * (cos(direction) * (1.0 / 2.0) + sin(direction) * (sqrt(3.0) / 2.0)));
    //PD_w2 = (SAC_fin * (cos(direction) * (1.0 / 2.0) - sin(direction) * (sqrt(3.0) / 2.0)));
    //PD_w3 = (-SAC_fin * cos(direction));

    //富田氏の球体移動機構の制御式（修論本体:p25.(3.4.3式参照)）
    PD_w1 = -cor_PD_X * (1.0 / 2.0) + (-cor_PD_Y) * (sqrt(3.0) / 2.0);
    PD_w2 = -cor_PD_X * (1.0 / 2.0) - (-cor_PD_Y) * (sqrt(3.0) / 2.0); 
    PD_w3 = cor_PD_X;
    return;
}

void vib_ctrl_Y(float degree, short& w1, short& w2)
{   
    if (-2 < degree && degree < 2)
    {
        w1 = 0;
        w2 = 0;
    }
    else
    {
        w1 = -2000 * 9.8 * sin(degree * Rad) * (sqrt(3.0/2.0));
        w2 = -w1;
    }
}
void vib_ctrl_X(float degree, short& w1, short& w2, short& w3)
{   
    if (-2 < degree && degree < 2)
    {
        w1 = 0;
        w2 = 0;
        w3 = 0;
    }
    else
    {
        w1 = 2000 * 9.8 * sin(degree * Rad) * (1.0 / 2.0);
        w2 = w1;
        w3 = -2000 * 9.8 * sin(degree * Rad);
    }
}

void motor_ctrl(short PD_w1, short PD_w2, short PD_w3, short w1, short w2, short w3, short& synt_vec_Y, short& synt_vec_X)
{
    short fin_w1 = PD_w1 + w1;
    short fin_w2 = PD_w2 + w2;
    short fin_w3 = PD_w3 + w3;
    L6470_move(fin_w1, fin_w2, fin_w3);
    synt_vec_Y = fin_w1 + (-fin_w2);
    synt_vec_X = fin_w1 + fin_w2 + (-fin_w3);
}

void degree_calc(int16_t acceleration[], int16_t gyro[], float& Vgx, float& Vgy, float& comp_x, float& comp_y, float& angle_gz)
{
    uint32_t dt;
    static uint32_t timestamp;
    static float Roll = 0, Pitch = 0, Yaw = 0;
    static float previous_comp_x = 0, previous_comp_y = 0;
    static float angle_gx;
    static float angle_gy;
    //static float angle_gz;
    const float k = 0.985;
    float ax = atan2(-acceleration[1],acceleration[2]);//まだrad
    float ay = atan2(acceleration[0],sqrt(acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]));
    ax = -ax * 57.29577;//*180/piをしてdegに戻す
    ay = -ay * 57.29577;

    //ジャイロから角度算出
    dt = time_us_32() - timestamp;
    timestamp = time_us_32();
    /*3軸ジャイロセンサのデータは内部機構を中心とした座標軸の角速度を表している。
    しかし、内部機構の座標軸と基準としている座標軸がずれると、それは正しいデータとは言えなくなる。
    そのため、回転行列を用いて座標軸を変換してジャイロ値を補正する*/
    static float gx = gyro[0];
    static float gy = gyro[1];
    static float gz = gyro[2];
    gx = gx*gyro_se;
    gy = gy*gyro_se;
    gz = gz*gyro_se;
    //角度変化量を計算
    float gx_num = (gx + (sin(Roll) * tan(Pitch) * gy) + (cos(Roll) * tan(Pitch) * gz));
    float gy_num = ((cos(Roll) * gy) + (-sin(Roll) * gz));
    float gz_num = ((sin(Roll) / cos(Pitch) * gy)+(cos(Roll) / cos(Pitch) * gz)) * dt * micro;

    Vgx = gx_num * Rad;
    Vgy = gy_num * Rad;

    gx_num = gx_num * dt * micro;
    gy_num = gy_num * dt * micro;
    //積分
    angle_gx += gx_num;
    angle_gy += gy_num;
    angle_gz += gz_num;
    //加速度にローパスフィルタ処理
    //(k=0.99 fc=0.2364hz)(k=0.0159 fc=10hz)(k=0.0228 fc=1000hz)

    /*k=0.99;//富田先輩のローパスフィルター 
    lpf_ax=(k*lpf_ax)+((1-k)*ax);
    lpf_ay=(k*lpf_ay)+((1-k)*ay);*/

    //加速度、ジャイロに相補フィルター処理
    comp_x = k * (previous_comp_x + gx_num) + ((1 - k) * ax);
    comp_y = k * (previous_comp_y + gy_num) + ((1 - k) * ay);

    previous_comp_x = comp_x;
    previous_comp_y = comp_y;

    //現在姿勢の更新
    Roll  = comp_x * Rad;
    Pitch = comp_y * Rad;
    Yaw   = angle_gz * Rad;
}

void sphere_speed(int synt_vec_Y, int synt_vec_X, float Vgy, float Vgx, float& Vsy, float& Vsx)
{
    /*ただの入力からstep数へ変換したあと、1stepあたりの角度である1.8を掛けて
    度へと変える*/
    float Vry = synt_vec_Y * Step_convert * 1.8;
    float Vrx = synt_vec_X * Step_convert * 1.8; 
    //タイヤの半径を使って速度[m/s]へと変換
    Vry = Vry * Rad * Wheel_R * milli;
    Vrx = Vrx * Rad * Wheel_R * milli;
    //ジャイロセンサの出力値は[°/s]なので[rad/s]に変換
    Vgy = Vgy * Rad;
    Vgx = Vgx * Rad;
    Vsy = Vry + (Sphere_R * milli * Vgx);
    Vsx = Vrx + (Sphere_R * milli * Vgy);
}

/* ==================== Tanaka control function ==================== */

void speed_ctrl(uint32_t t, float& step)
{
    if(t<2000000){
        step=START_SPEED;
    }else{
        step=10000*(1.0/5.0)*(5.0-4.0*exp((-1.0/450000.0)*(t-2000000)));
        //時定数450000で一次遅れの式にステップ信号を入れ、逆ラプラス変換したもの
    }
}

/* ================================================================= */


int main()
{
    float comp_x = 0, comp_y = 0;
    float Vgx = 0, Vgy = 0;
    float Vsy = 0, Vsx = 0;
    int Ypulse_num = 0, Xpulse_num = 0;
    float Vst_y = 0,Vst_x = 0;

    int32_t gy_offset[3];
    int16_t acceleration[3];
    int16_t gyro[3];
    int16_t temp;

    int16_t acceleration2[3];
    int16_t gyro2[3];
    int16_t temp2;


    float vib_cor_Y = 0; //操作補正量
    float vib_cor_X = 0; //操作補正量
    float cor_P_Y = 0;  // P control operation correction amount(P制御の操作補正量)
    float cor_D_Y = 0;  // D control operation correction amount(D制御の操作補正量)PD制御の操作補正量

    float cor_P_X = 0;  // P control operation correction amount(P制御の操作補正量)
    float cor_D_X = 0;  // D control operation correction amount(D制御の操作補正量)

    float cor_PD_Y = 0; //PD control Operation correction amount(PD制御の操作補正量)
    float cor_PD_X = 0; //PD control Operation correction amount(PD制御の操作補正量)

    short synt_vec_Y = 0, synt_vec_X = 0; //synthetic vector(合成ベクトル)

    float Yaw = 0;

    //Motor rotation speed in the direction of travel based on PD control correction amount
    //(PD制御の補正量から進行方向に対してのモータの回転数)
    short PD_w1 = 0, PD_w2 = 0, PD_w3 = 0; 

    //Motor control value determined by vibration damping control
    //(制振制御によって決められたモータの制御値)
    short w1_Y = 0, w2_Y = 0, w1_X = 0, w2_X = 0, w3_X = 0; 

    u_int32_t timestamp = 0;
    u_int32_t timer=0;

    //u_int32_t time=0,cycle=0,number=0;//周期確認用

    int DMC = 0;    //キー入力なしの時のカウンタ

    int w=0,a=0,s=0,d=0;    //ループ回数確認用変数

        //Tc=1/2πfc fcはカットオフ周波数　Ts=サンプリングタイム
    
    stdio_init_all();
    uart_init(uart0,115200);//シリアル通信の設定 GP0,GP1ピンを使用 詳しくはpicoのピン配置図を参照
    gpio_set_function(0,GPIO_FUNC_UART);
    gpio_set_function(1,GPIO_FUNC_UART);

    uart_puts(uart0, "Hello, MPU9250! Reading raw data from registers via SPI...\n");

    // This example will use SPI0 at 0.5MHz.
    spi_init(SPI_PORT_MPU, 1000 * 1000);//SPI通信の設定 SPI1を使用 GP8,10,11ピンの設定,センサ用 1Mhzが最大
    gpio_set_function(PIN_MISO_MPU, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK_MPU, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI_MPU, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PIN_MISO_MPU, PIN_MOSI_MPU, PIN_SCK_MPU, GPIO_FUNC_SPI));

    gpio_init(PIN_CS_MPU);//セルセレクトのためのGP9ピンの設定
    gpio_set_dir(PIN_CS_MPU, GPIO_OUT);
    gpio_put(PIN_CS_MPU, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PIN_CS_MPU, "SPI CS1"));

    spi_init(SPI_PORT_L6470, 4000*1000);//SPI0を使用,5Mhzが最大,モタドラ用
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
    gpio_set_dir(16,GPIO_IN);//16pinを入力に設定
    gpio_pull_up(16);//16ピンをプルアップ
    gpio_init(17);
    gpio_set_dir(17,GPIO_OUT);//17pinを出力に設定
    gpio_put(17,1);
    sleep_ms(100);
    
    //t=time_us_32();

    uart_puts(uart0, "AccX, Y, Z, GyroX, Y, Z, Temp\n");

    while (1) 
    {
        if(gpio_get(16) == true)
        {
            gpio_put(17,1);//LED点滅
            sleep_ms(500);
            gpio_put(17,0);
            sleep_ms(500);
            L6470_manualstop(Ypulse_num, Xpulse_num);
            timestamp = time_us_32();
        }
        else
        {
            gpio_put(17, 1);//LED常時点灯

            //シリアルポートにデータが来ているか判別
            if (uart_is_readable(uart0))
            {
                char Rx = uart_getc(uart0);//シリアルポートからデータを取得
                switch (Rx)
                {
                uint32_t start_time;
                case 'w':
                    if (w == 0)
                    {
                        Vst_y = 0.3;
                        Vst_x = 0.0;
                        w++;
                    }
                    break;

                default:
                    break;
                }
                DMC = 0;
            }
            else
            {
                DMC++;
            }
            if (DMC > 1000)
                {
                    Vst_y = 0.0;
                    Vst_x = 0.0;
                    w = 0;
                }
            P_ctrl_Y(w, Vst_y, Vsy, cor_P_Y);
            D_ctrl_Y(w, Vst_y, Vsy, cor_D_Y);

            P_ctrl_X(d, Vst_x, Vsx, cor_P_X);
            D_ctrl_X(d, Vst_x, Vsx, cor_D_X);

            cor_tot(w, cor_P_Y, cor_D_Y, cor_P_X, cor_D_X, cor_PD_Y, cor_PD_X);
            motor_calc(cor_PD_Y, cor_PD_X, PD_w1, PD_w2, PD_w3, synt_vec_Y, synt_vec_X);

            vib_ctrl_Y(comp_x, w1_Y, w2_Y);
            vib_ctrl_X(comp_y, w1_X, w2_X, w3_X);

            motor_ctrl(PD_w1, PD_w2, PD_w3, (w1_Y + w1_X), (w2_Y + w2_X), w3_X, synt_vec_Y, synt_vec_X);
            
            mpu9250_read_raw2(gy_offset, acceleration, gyro, temp);
            degree_calc(acceleration, gyro, Vgx, Vgy,comp_x, comp_y, Yaw);
            sphere_speed(synt_vec_Y, synt_vec_X, Vgy, Vgx, Vsy, Vsx);
            if (time_us_32() - timestamp > 100000)
            {
                printf("Vst_y = %f\n", Vst_y);
                //printf("DMC = %d\n", DMC);
                //printf("w = %d\n", w);
                //printf("Vsx = %f\n", Vsy);
                //printf("comp_x = %f\n", comp_x);
                //printf("comp_y = %f\n", comp_y);
                //printf("synt_vec_Y = %d\n", synt_vec_Y);
                //printf("synt_vec_X = %d\n", synt_vec_X);
                //printf("Vgx = %f\n", Vgx);
                //printf("Vgy = %f\n", Vgy);
                //printf("Yaw = %f\n", Yaw);
                //printf("cor_P_Y = %f\n", cor_P_Y);
                //printf("cor_D_Y = %f\n", cor_D_Y);
                //printf("cor_PD_Y = %f\n", cor_PD_Y);
                //printf("cor_P_X = %f\n", cor_P_X);
                //printf("cor_D_X = %f\n", cor_D_X);
                //printf("cor_PD_X = %f\n", cor_PD_X);
                timestamp = time_us_32();
            }
        }
    }
    return 0;
}


