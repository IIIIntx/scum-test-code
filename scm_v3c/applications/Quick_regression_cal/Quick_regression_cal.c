/***************************/
//Project: Quick Regression Calibration 
//Author：Yiming YUAN(yyuan861@connect.hkust-gz.edu.cn)
//Discript：Quick Calculate the <coarse> and <mid> <fine> value for transmit
/**************************/

#include <string.h>
#include <math.h>

#include "scm3c_hw_interface.h"
#include "memory_map.h"
#include "rftimer.h"
#include "radio.h"
#include "ble.h"
#include "optical.h"

//=========================== defines =========================================

#define CRC_VALUE           (*((unsigned int *) 0x0000FFFC))
#define CODE_LENGTH         (*((unsigned int *) 0x0000FFF8))

#define TXPOWER             0xD8    // used for ibeacon pkt

#define NUMPKT_PER_CFG      100
#define STEPS_PER_CONFIG    32
#define TIMER_PERIOD        500  // 500 = 1ms@500kHz
#define TIMER_PERIOD_BLE    2000

#define CHANNEL             0       // ble channel
#define Freq_target         1.252 //(Target_channel/960)/2000
// #define RC_count_target     2000

// only this coarse settings are swept, 
// channel 37 and 0 are known within the setting scope of coarse=24

#define NUM_SAMPLES         10
#define RANGE               8


//=========================== variables =======================================

typedef struct {
                    uint8_t         tx_coarse_p1;
                    uint8_t         tx_mid_p1;
                    uint8_t         tx_fine_p1;
                    uint8_t         tx_coarse_p2;
                    uint8_t         tx_mid_p2;
                    uint8_t         tx_fine_p2;
                    //record the cb_timer num to compensite 4ms for ble transmitte
                    uint8_t			cb_timer_num;
        volatile    uint8_t         sample_index;
                    uint32_t        samples[NUM_SAMPLES];

                    bool            sendDone;
                    
                    uint8_t         pdu[PDU_LENGTH+CRC_LENGTH]; // protocol data unit
                    uint8_t         pdu_len;
                
                    uint32_t 				avg_sample;
                    uint32_t 				count_2M;
                    uint32_t 				count_LC;
                    uint32_t 				count_adc;
                    bool 				    freq_setFinish_flag;
                    uint8_t 				freq_setTimes_flag;
                    bool                    calculate_flag;             //give time for calculating
                    uint16_t 				calculate_times_flag;       //give time for calculating
                    bool                    RC_count_flag;
                    bool                    freq_setFinish_flag2;        //flag of finish estimate setting

                    int 				    RC_count;                    //count the number of 2M
                    double                  Comp_coff;
                    int                     freq_abs;
                    //to decrease the time, save the rough estimate parameter
                    double                  para_matrix[2][1];
                    double                  Inv_equ_c;  
} app_vars_t;

app_vars_t app_vars;

double freqTargetList[40] = {1.252, 1.253, 1.254, 1.255, 1.256, 1.257, 1.258, 1.259, 1.260, 1.261, 1.263, 1.265, 1.266, 1.267, 1.268, 1.269, 1.270, 1.271, 1.272, 1.273, 1.274, 1.275, 1.276, 1.277, 1.278, 1.279, 1.280, 1.281, 1.282, 1.283, 1.284, 1.285, 1.286, 1.288, 1.289, 1.290, 1.291, 1.251, 1.264, 1.292};


//=========================== prototypes ======================================

void        cb_endFrame_tx(uint32_t timestamp);
void        cb_timer(void);
uint8_t prepare_freq_setting_pdu(uint8_t coarse, uint8_t mid, uint8_t fine);

void        delay_tx(void);
void        delay_lc_setup(void);
void        course_estimate(uint16_t channelTarget);
void        Gaussian_elimination(int x_matrix_raw[2][2], double x_matrix_inverse[2][2]);
void        matrixMultiplication(double x_matrix_inverse[][2], int y_matrix[][1], double para_matrix[][1]);
void        Solve_Equation(double para_matrix[][1], double Inv_equ_c, double x_matrix_inverse[][2]);
void         __TransmitPacket (uint16_t channelTarget);
void        __PresiseEstimate(void);

//=========================== main ============================================

int main(void) {

    uint32_t calc_crc;
    uint16_t channelTarget;

    memset(&app_vars, 0, sizeof(app_vars_t));

    printf("Initializing...");

    // Set up mote configuration
    // This function handles all the analog scan chain setup
    initialize_mote();
    ble_init_tx();

    radio_setEndFrameTxCb(cb_endFrame_tx);
    rftimer_set_callback(cb_timer);
    
    // Disable interrupts for the radio and rftimer
    radio_disable_interrupts();
    rftimer_disable_interrupts();

    // Check CRC to ensure there were no errors during optical programming
    printf("\r\n-------------------\r\n");
    printf("Validating program integrity...");

    calc_crc = crc32c(0x0000,CODE_LENGTH);

    if (calc_crc == CRC_VALUE) {
        printf("CRC OK\r\n");
    } else {
        printf("\r\nProgramming Error - CRC DOES NOT MATCH - Halting Execution\r\n");
        while(1);
    }

    // After bootloading the next thing that happens is frequency calibration using optical
    printf("Calibrating frequencies...\r\n");

    // Initial frequency calibration will tune the frequencies for HCLK, the RX/TX chip clocks, and the LO
    
    optical_enableLCCalibration();

    // Turn on LO, DIV, PA, and IF
    ANALOG_CFG_REG__10 = 0x78;

    // Turn off polyphase and disable mixer
    ANALOG_CFG_REG__16 = 0x6;


    optical_enable();

    // Wait for optical cal to finish
    while(!optical_getCalibrationFinished());

    printf("Cal complete\r\n");
    
    // Enable interrupts for the radio FSM (Not working for ble)
    radio_enable_interrupts();

    radio_rfOff();
	radio_txEnable();
    
    //set a initial channel
    channelTarget = CHANNEL;

    while (1) {
        
        __TransmitPacket(channelTarget);

    }
}

//=========================== public ==========================================

//=========================== private =========================================

//==== callback
uint32_t     average_sample(void){
    uint8_t i;
    uint32_t avg;
    
    avg = 0;
    for (i=0;i<NUM_SAMPLES;i++) {
        avg += app_vars.samples[i];
    }
    avg = avg/NUM_SAMPLES;
    return avg;
}

/**************************/
//Function: As a timer, the timer source is RF timer 500kHz
//Parameter: None
//Return: None
//Call: After 'rftimer_setCompareIn'
//Description: This function is called when the timer is triggered, seperated to two parts, one is for estimate process, and another is for transmit process.
//Date: 31.May.2024
/*************************/
void cb_timer(void) {
    if(app_vars.freq_setFinish_flag2 == 0){
        rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
        read_counters_3B(&app_vars.count_2M,&app_vars.count_LC,&app_vars.count_adc);
        if (app_vars.RC_count_flag==0)
        {
            app_vars.samples[app_vars.sample_index] = app_vars.count_2M;
            app_vars.sample_index++;
            if (app_vars.sample_index==NUM_SAMPLES) {
                app_vars.sample_index = 0;
                app_vars.RC_count = average_sample();
                app_vars.cb_timer_num++;
            }
            if (app_vars.cb_timer_num==4)
            {
                app_vars.RC_count_flag = 1;
            }
            
            // printf("here");
        }
        if (app_vars.RC_count_flag==1){
            app_vars.samples[app_vars.sample_index] = app_vars.count_LC;
            app_vars.sample_index++;
            if (app_vars.sample_index==NUM_SAMPLES) {
                app_vars.sample_index = 0;
                app_vars.avg_sample = average_sample();
        //			printf("%d \r\n",app_vars.avg_sample);
                    
                if(app_vars.freq_setFinish_flag == 0 && app_vars.freq_setTimes_flag == 5){
                    app_vars.freq_setFinish_flag = 1;
                    app_vars.freq_setTimes_flag = 0;
                }
                app_vars.freq_setTimes_flag++;
            }
            if (app_vars.calculate_flag == 1)
            {
                app_vars.calculate_times_flag++;
                if (app_vars.calculate_times_flag==30)
                {
                    app_vars.calculate_times_flag=0;
                    app_vars.calculate_flag = 0;
                }
                
            }
        }
    }
    else if (app_vars.freq_setFinish_flag2 == 1)
    {
        app_vars.sendDone = true;
    }
}

void    cb_endFrame_tx(uint32_t timestamp){
    
    printf("this is end of tx \r\n");
	
}


//==== delay

// 0x07ff roughly corresponds to 2.8ms
#define TX_DELAY 0x07ff

void delay_tx(void) {
    uint16_t i;
    for (i=0;i<TX_DELAY;i++);
}

#define LC_SETUP_DELAY 0x02ff

void delay_lc_setup(void) {
    uint16_t i;
    for (i=0;i<LC_SETUP_DELAY;i++);
}

//==== regression
//0428 根据RC timer的值设置补充系数

/**************************/
//Function: Calculate the <coarse>,<mid>,<fine>
//Parameter: None
//Return: None (return transmit setting to appvars paraset)
//Call: Before the transmit
//Date: 31.May.2024
/*************************/
void course_estimate(uint16_t channelTarget){
   
    //用无符号数总会出现奇怪的问题
    int p1L_count, p1R_count, p2L_count, p2R_count, p3L_count, p3R_count;
    int p1x, p1y, p2x, p2y, p3x, p3y;
    int x_matrix[2][2];
    int y_matrix[2][1];
    double x_matrix_inverse[2][2];
    double para_matrix[2][1];
    double Inv_equ_c;
    // double test1, test2, test3;
    rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
    while (app_vars.RC_count_flag==0);
    
    LC_FREQCHANGE(1,31,31);
    rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
    while(app_vars.freq_setFinish_flag == 0);
    p1L_count = app_vars.avg_sample;
    while (p1L_count <=1000)       
    {
        rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
        while(app_vars.freq_setFinish_flag == 0);
        printf("cal error, waiting..");
        p1L_count = app_vars.avg_sample;
    }
    
    // printf("p1L_count=%d\r\n",p1L_count);

    LC_FREQCHANGE(2,0,0);
    rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
    app_vars.freq_setFinish_flag = 0;
    while(app_vars.freq_setFinish_flag == 0);
    p1R_count = app_vars.avg_sample;
    // printf("p1R_count=%d\r\n",p1R_count);
    
    LC_FREQCHANGE(15,31,31);
    rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
    app_vars.freq_setFinish_flag = 0;
    while(app_vars.freq_setFinish_flag == 0);
    p2L_count = app_vars.avg_sample;
    // printf("p2L_count=%d\r\n",p2L_count);
    
    LC_FREQCHANGE(16,0,0);
    rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
    app_vars.freq_setFinish_flag = 0;
    while(app_vars.freq_setFinish_flag == 0);
    p2R_count = app_vars.avg_sample;
    // printf("p2R_count=%d\r\n",p2R_count);
    
    LC_FREQCHANGE(30,31,31);
    rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
    app_vars.freq_setFinish_flag = 0;
    while(app_vars.freq_setFinish_flag == 0);
    p3L_count = app_vars.avg_sample;
    // printf("p3L_count=%d\r\n",p3L_count);
    
    LC_FREQCHANGE(31,0,0);
    rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
    app_vars.freq_setFinish_flag = 0;
    while(app_vars.freq_setFinish_flag == 0);
    p3R_count = app_vars.avg_sample;
    // printf("RC_count=%d\r\n",p3R_count);
    // Inv_equ_c = (double)app_vars.RC_count;
    // // printf("ah=%f",Inv_equ_c);
    // Comp_coff = Inv_equ_c/ 2000;

    //这一步为什么算不出来 直接赋值就可以算出来？
    app_vars.Comp_coff = freqTargetList[channelTarget];
    // printf("coff=%f\r\n",app_vars.Comp_coff);
    app_vars.freq_abs = (int)(freqTargetList[channelTarget]*app_vars.RC_count);
    printf("freq=%d\r\n",app_vars.freq_abs);
    printf("RC=%d\r\n",app_vars.RC_count);

    p1y = (p1L_count + p1R_count)/2;
    p2y = (p2L_count + p2R_count)/2;
    p3y = (p3L_count + p3R_count)/2;
    p1x = 2;
    p2x = 16;
    p3x = 31;

    // 修改 x_matrix
    x_matrix[0][0] = ((p2x * p2x) - (p1x * p1x));
    x_matrix[0][1] = (p2x - p1x);
    x_matrix[1][0] = ((p3x * p3x) - (p1x * p1x));
    x_matrix[1][1] = (p3x - p1x);
    // printf("xmatrix = %d,%d,%d,%d\r\n", x_matrix[0][0], x_matrix[0][1], x_matrix[1][0], x_matrix[1][1]);
    //0427 浮点运算算力太差，且使用标号为x会导致结果太大
    //能用整数尽可能用整数
    //尝试直接以x=2, x=16, x=31进行计算
    //反解出来只需要和.5比较就可以

    //0428 部分板子存在course code小的时候LC不准确的情况
    //尝试使用更大的course code来实现拟合

    // 修改 y_matrix
    y_matrix[0][0] = p2y - p1y;
    y_matrix[1][0] = p3y - p1y;
    // printf("y_matrix = %d,%d\r\n", y_matrix[0][0], y_matrix[1][0]);
    // printf("xmatrix = %d,%d,%d,%d\r\n", x_matrix[0][0], x_matrix[0][1], x_matrix[1][0], x_matrix[1][1]);
    Gaussian_elimination(x_matrix, x_matrix_inverse);
    matrixMultiplication(x_matrix_inverse, y_matrix, para_matrix);
    // printf("para_matrix = %f,%f\r\n", para_matrix[0][0], para_matrix[1][0]);
    //x_matrix_inverse不再使用，帮助计算存储中间值
    x_matrix_inverse[0][0] = para_matrix[0][0] * (p2x *p2x);
    x_matrix_inverse[0][1] = para_matrix[1][0] * p2x;
    x_matrix_inverse[1][0] = x_matrix_inverse[0][0] + x_matrix_inverse[0][1];
    Inv_equ_c = p2y - x_matrix_inverse[1][0];
    printf("Inv_equ_c=%f\r\n",Inv_equ_c);
    app_vars.Inv_equ_c = Inv_equ_c;
    app_vars.para_matrix[0][0] = para_matrix[0][0];
    app_vars.para_matrix[1][0] = para_matrix[1][0];
}

void __PresiseEstimate(void){
    int p1L_count, p1R_count, p2L_count, p2R_count, p3L_count, p3R_count;
    int p1x, p1y, p2x, p2y, p3x, p3y;
    double x_matrix_inverse[2][2];
    double assistMatrix[2][1];  //assist calculating, sometime directive calculating will stuck
    
    printf("begin precise stimate\r\n");
    Solve_Equation(app_vars.para_matrix, app_vars.Inv_equ_c,x_matrix_inverse);
    //开始分区，p1L_count, p1R_count, p2L_count, p2R_count, p3L_count, p3R_count释放，帮助计算
    if ((int)x_matrix_inverse[0][0]%10 <=5)
    {
        p1R_count = (int)x_matrix_inverse[1][0];
        printf("p1r=%d\r\n",p1R_count);
        p1L_count = p1R_count - 1;
        LC_FREQCHANGE(p1L_count,16,0);
        rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
        app_vars.freq_setFinish_flag = 0;
        while(app_vars.freq_setFinish_flag == 0);
        p1x = app_vars.avg_sample;

        LC_FREQCHANGE(p1L_count,31,31);
        rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
        app_vars.freq_setFinish_flag = 0;
        while(app_vars.freq_setFinish_flag == 0);
        p1y = app_vars.avg_sample;

        LC_FREQCHANGE(p1R_count,0,0);
        rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
        app_vars.freq_setFinish_flag = 0;
        while(app_vars.freq_setFinish_flag == 0);
        p2x = app_vars.avg_sample;

        LC_FREQCHANGE(p1R_count,15,31);
        rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
        app_vars.freq_setFinish_flag = 0;
        while(app_vars.freq_setFinish_flag == 0);
        p2y = app_vars.avg_sample;
        app_vars.tx_coarse_p1 = p1L_count;
        app_vars.tx_coarse_p2 = p1R_count;
    }
    else if ((int)x_matrix_inverse[0][0]%10 >5)
    {
        p1L_count = (int)x_matrix_inverse[1][0];
        printf("p1r=%d\r\n",p1R_count);
        p1R_count = p1L_count + 1;
        LC_FREQCHANGE(p1L_count,16,0);
        rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
        app_vars.freq_setFinish_flag = 0;
        while(app_vars.freq_setFinish_flag == 0);
        p1x = app_vars.avg_sample;

        LC_FREQCHANGE(p1L_count,31,31);
        rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
        app_vars.freq_setFinish_flag = 0;
        while(app_vars.freq_setFinish_flag == 0);
        p1y = app_vars.avg_sample;

        LC_FREQCHANGE(p1R_count,0,0);
        rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
        app_vars.freq_setFinish_flag = 0;
        while(app_vars.freq_setFinish_flag == 0);
        p2x = app_vars.avg_sample;

        LC_FREQCHANGE(p1R_count,15,31);
        rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
        app_vars.freq_setFinish_flag = 0;
        while(app_vars.freq_setFinish_flag == 0);
        p2y = app_vars.avg_sample;
        app_vars.tx_coarse_p1 = p1L_count;
        app_vars.tx_coarse_p2 = p1R_count;
    }
    //存在隐藏问题，如果比两个都小，待修正
    //可以优化，如果比其中一个大，顺延到下一个course的下区
    while ((app_vars.freq_abs > p1y && app_vars.freq_abs > p2y)||(app_vars.freq_abs < p1x && app_vars.freq_abs < p2x))
    {
        x_matrix_inverse[1][0] = x_matrix_inverse[1][0] + 1;
        if ((int)x_matrix_inverse[0][0]%10 <=5)
        {
            p1R_count = (int)x_matrix_inverse[1][0];
            printf("p1r=%d\r\n",p1R_count);
            p1L_count = p1R_count - 1;
            LC_FREQCHANGE(p1L_count,16,0);
            rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
            app_vars.freq_setFinish_flag = 0;
            while(app_vars.freq_setFinish_flag == 0);
            p1x = app_vars.avg_sample;

            LC_FREQCHANGE(p1L_count,31,31);
            rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
            app_vars.freq_setFinish_flag = 0;
            while(app_vars.freq_setFinish_flag == 0);
            p1y = app_vars.avg_sample;

            LC_FREQCHANGE(p1R_count,0,0);
            rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
            app_vars.freq_setFinish_flag = 0;
            while(app_vars.freq_setFinish_flag == 0);
            p2x = app_vars.avg_sample;

            LC_FREQCHANGE(p1R_count,15,31);
            rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
            app_vars.freq_setFinish_flag = 0;
            while(app_vars.freq_setFinish_flag == 0);
            p2y = app_vars.avg_sample;
            app_vars.tx_coarse_p1 = p1L_count;
            app_vars.tx_coarse_p2 = p1R_count;
        }
        else if ((int)x_matrix_inverse[0][0]%10 >5)
        {
            p1L_count = (int)x_matrix_inverse[1][0];
            printf("p1r=%d\r\n",p1R_count);
            p1R_count = p1L_count + 1;
            LC_FREQCHANGE(p1L_count,16,0);
            rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
            app_vars.freq_setFinish_flag = 0;
            while(app_vars.freq_setFinish_flag == 0);
            p1x = app_vars.avg_sample;

            LC_FREQCHANGE(p1L_count,31,31);
            rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
            app_vars.freq_setFinish_flag = 0;
            while(app_vars.freq_setFinish_flag == 0);
            p1y = app_vars.avg_sample;

            LC_FREQCHANGE(p1R_count,0,0);
            rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
            app_vars.freq_setFinish_flag = 0;
            while(app_vars.freq_setFinish_flag == 0);
            p2x = app_vars.avg_sample;

            LC_FREQCHANGE(p1R_count,15,31);
            rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
            app_vars.freq_setFinish_flag = 0;
            while(app_vars.freq_setFinish_flag == 0);
            p2y = app_vars.avg_sample;

            app_vars.tx_coarse_p1 = p1L_count;
            app_vars.tx_coarse_p2 = p1R_count;
        }
        printf("p1x=%d, p1y=%d, p2x=%d, p2y=%d\r\n",p1x, p1y, p2x, p2y);
    }
    
    //打印获取的数据
    printf("p1x=%d, p1y=%d, p2x=%d, p2y=%d\r\n",p1x, p1y, p2x, p2y);
    //都围绕在course的半区内，p1区使用16-32，p2区使用0-16
    // p1区
    p2L_count = p1y - p1x;
    p2R_count = 32 - 16;
    x_matrix_inverse[0][0] = (double)p2L_count/p2R_count; //k
    x_matrix_inverse[0][1] = x_matrix_inverse[0][0]*16; //k*x
    x_matrix_inverse[1][0] = p1x - x_matrix_inverse[0][1]; //b=y-k*x
    x_matrix_inverse[0][1] = app_vars.freq_abs - x_matrix_inverse[1][0]; //y-b
    assistMatrix[0][0] = x_matrix_inverse[0][1]/x_matrix_inverse[0][0]; //p1 set = (y-b)/k

    // p2区
    p3L_count = p2y - p2x;
    p3R_count = 16 - 0;
    x_matrix_inverse[0][0] = (double)p3L_count/p3R_count; //k
    x_matrix_inverse[0][1] = x_matrix_inverse[0][0]*16; //k*x
    x_matrix_inverse[1][0] = p2y - x_matrix_inverse[0][1];// b = y-kx
    x_matrix_inverse[0][1] = app_vars.freq_abs - x_matrix_inverse[1][0]; //y-b
    assistMatrix[1][0] = x_matrix_inverse[0][1]/x_matrix_inverse[0][0]; //p2 set = (y-b)/k

    //下面需要验证是否在该范围中
    //可能遇到的问题是LC_count与发射频率的误差

    //输出结果
    if ((int)assistMatrix[0][0]>=16 && (int)assistMatrix[0][0]<=32)
    {
        p1x = (int)assistMatrix[0][0];
        x_matrix_inverse[0][1] = assistMatrix[0][0] - p1x;
        p1y = (int)(x_matrix_inverse[0][1]*31);
    }
    else
    {
        p1x = 0;
        p1y = 0;
    }
    if ((int)assistMatrix[1][0]>=0 && (int)assistMatrix[1][0]<=16)
    {
        p2x = (int)assistMatrix[1][0];
        x_matrix_inverse[1][1] = assistMatrix[1][0] - p2x;
        p2y = (int)(x_matrix_inverse[1][1]*31);
    }
    else
    {
        p2x = 0;
        p2y = 0;
    }
    printf("set1=%d,%d\r\n",p1x,p1y);
    printf("set2=%d.%d\r\n",p2x,p2y); 
    app_vars.tx_mid_p1 = p1x;
    app_vars.tx_fine_p1 = p1y;
    app_vars.tx_mid_p2 = p2x;
    app_vars.tx_fine_p2 = p2y;
    app_vars.freq_setFinish_flag2 = 1;
    
}

/**************************/
//Function: Use Gaussian elimination to calculate the inverse of the matrix for transmit setting
//Parameter: x_matrix_raw[][],  x_matrix_inversep[][]
//Return: None (return inverse parameter directively)
//Call: course_estimate 
//Date: 31.May.2024
/*************************/
//使用高斯消元法对矩阵进行求逆
void Gaussian_elimination(int x_matrix_raw[2][2], double x_matrix_inverse[2][2]) {
    // 计算矩阵的行列式
    double determinant;

    determinant = x_matrix_raw[0][0] * x_matrix_raw[1][1] - x_matrix_raw[0][1] * x_matrix_raw[1][0];
    
    // 检查行列式是否为0，若为0则矩阵不可逆
    if (determinant == 0) {
        printf("Matrix is not invertible.\n");
        return;
    }

    // 计算逆矩阵的每个元素
    x_matrix_inverse[0][0] = x_matrix_raw[1][1] / determinant;
    x_matrix_inverse[0][1] = -x_matrix_raw[0][1] / determinant;
    x_matrix_inverse[1][0] = -x_matrix_raw[1][0] / determinant;
    x_matrix_inverse[1][1] = x_matrix_raw[0][0] / determinant;
    app_vars.calculate_flag = 1;
    while(app_vars.calculate_flag == 1){
    }; 
}

/**************************/
//Function: multiple two matrix
//Parameter: x_matrix_raw[][],  x_matrix_inversep[][]
//Return: None (return matrix directively)
//Call: course_estimate 
//Date: 31.May.2024
/*************************/
void matrixMultiplication(double x_matrix_inverse[][2], int y_matrix[][1], double para_matrix[][1]) {
    int i, j, k;
    
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 1; j++) {
            para_matrix[i][j] = 0;
            for (k = 0; k < 2; k++) {
                para_matrix[i][j] += x_matrix_inverse[i][k] * y_matrix[k][j];
            }
        }
    }
}

void Solve_Equation(double para_matrix[][1], double Inv_equ_c, double x_matrix_inverse[][2]) {
    Inv_equ_c = Inv_equ_c - app_vars.freq_abs;
    //同样使用x_matrix_inverse帮助计算存储中间值，以减少内存
    x_matrix_inverse[0][0] = para_matrix[1][0]*para_matrix[1][0];
    x_matrix_inverse[0][1] = 4 * para_matrix[0][0]*Inv_equ_c;
    x_matrix_inverse[1][0] = x_matrix_inverse[0][0] - x_matrix_inverse[0][1];
    x_matrix_inverse[1][1] = sqrt(x_matrix_inverse[1][0]);

    //x_matrix_inverse[0][0],[0][1],[1][0]重新释放 
    x_matrix_inverse[0][0] = 2*para_matrix[0][0];
    x_matrix_inverse[0][1] = 1/x_matrix_inverse[0][0];
    x_matrix_inverse[1][0] = x_matrix_inverse[0][1] * (-para_matrix[1][0]+x_matrix_inverse[1][1]);
    //只需要x大于0的根
    //在此处直接乘10，看除10的余数大于5还是小于5
    x_matrix_inverse[0][0] = x_matrix_inverse[1][0] * 10;
    printf("solve = %f\r\n", x_matrix_inverse[1][0]);
}

//==== pdu related
uint8_t prepare_freq_setting_pdu(uint8_t coarse, uint8_t mid, uint8_t fine) {
    
    uint8_t i;
    
    uint8_t field_len;
    
    memset(app_vars.pdu, 0, sizeof(app_vars.pdu));
    
    // adv head (to be explained)
    i = 0;
    field_len = 0;
    
    app_vars.pdu[i++] = flipChar(0x42);
    app_vars.pdu[i++] = flipChar(0x09);
	
		app_vars.pdu[i++] = flipChar(0xCC);
		app_vars.pdu[i++] = flipChar(0xBB);
		app_vars.pdu[i++] = flipChar(0xAA);
//		app_vars.pdu[i++] = 0x20;
//    app_vars.pdu[i++] = 0x03;
	
    app_vars.pdu[i++] = flipChar(coarse);
    app_vars.pdu[i++] = flipChar(mid);
    app_vars.pdu[i++] = flipChar(fine);
//		app_vars.pdu[i++] = flipChar(0xAB);
//    app_vars.pdu[i++] = flipChar(0xAB);
//    app_vars.pdu[i++] = flipChar(0xAB);
    
    field_len += 8;
    
    return field_len;
}

//==== transmit process
/**************************/
//Function: transmit pdu packet on selected channel
//Parameter: channelTarget is listed, the channelTarget as a index to realize channel hopping
//Return: None
//Call: main 
//Date: 31.May.2024
/*************************/
void __TransmitPacket (uint16_t channelTarget){
    int i;
    int offset;
    uint8_t cfg_course;
    uint8_t cfg_mid;
    int cfg_fine;

    ble_set_channel(channelTarget);
	course_estimate(channelTarget);
    __PresiseEstimate();

    //p1
    cfg_course = app_vars.tx_coarse_p1;
    for(offset = -RANGE; offset <= RANGE; offset++){
        cfg_fine = app_vars.tx_fine_p1 + offset;
        cfg_mid = app_vars.tx_mid_p1;
        if(cfg_fine<0){
            cfg_fine = 31+offset;
            cfg_mid -= 1;
        }else if(cfg_fine>32){
            cfg_fine = offset;
            cfg_mid += 1;
        }   
        printf(
            "coarse=%d, middle=%d, fine=%d\r\n", 
            cfg_course,cfg_mid,cfg_fine
        );
        
        for (i=0;i<NUMPKT_PER_CFG;i++) {
            
            radio_rfOff();
            
            app_vars.pdu_len = prepare_freq_setting_pdu(cfg_course, cfg_mid, cfg_fine);
            ble_prepare_packt(&app_vars.pdu[0], app_vars.pdu_len);
            
            LC_FREQCHANGE(cfg_course, cfg_mid, cfg_fine);
            
            delay_lc_setup();
            
            ble_load_tx_arb_fifo();
            radio_txEnable();
            
            delay_tx();
            
            ble_txNow_tx_arb_fifo();
            
            // need to make sure the tx is done before 
            // starting a new transmission
            
            rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
            app_vars.sendDone = false;
            while (app_vars.sendDone==false);
        }
        
    }
    //p2
    cfg_course = app_vars.tx_coarse_p2;
    for(offset = -RANGE; offset <= RANGE; offset++){
        cfg_fine = app_vars.tx_fine_p2 + offset;
        cfg_mid = app_vars.tx_mid_p2;
        if(cfg_fine<0){
            cfg_fine = 31+offset;
            cfg_mid -= 1;
        }else if(cfg_fine>32){
            cfg_fine = offset;
            cfg_mid += 1;
        }   
        printf(
            "coarse=%d, middle=%d, fine=%d\r\n", 
            cfg_course,cfg_mid,cfg_fine
        );
        
        for (i=0;i<NUMPKT_PER_CFG;i++) {
            
            radio_rfOff();
            
            app_vars.pdu_len = prepare_freq_setting_pdu(cfg_course, cfg_mid, cfg_fine);
            ble_prepare_packt(&app_vars.pdu[0], app_vars.pdu_len);
            
            LC_FREQCHANGE(cfg_course, cfg_mid, cfg_fine);
            
            delay_lc_setup();
            
            ble_load_tx_arb_fifo();
            radio_txEnable();
            
            delay_tx();
            
            ble_txNow_tx_arb_fifo();
            
            // need to make sure the tx is done before 
            // starting a new transmission
            
            rftimer_setCompareIn(rftimer_readCounter()+TIMER_PERIOD);
            app_vars.sendDone = false;
            while (app_vars.sendDone==false);
        }
        
    }
}