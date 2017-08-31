/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "math.h"
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "ysPWM.h"

/******************************************************************************
| defines
|----------------------------------------------------------------------------*/
/* IM parameters */
#define Rs 5  //4.4717
#define Rr 3.1407
#define Lm 0.42735
#define Lls 0.0173
#define Ls 0.44465
#define Llr 0.0173
#define Lr 0.44465
#define Tr 0.1415767
#define np 2

/* control period */
#define Ts 5e-5

/* lamdar最小值限制 */
#define lamdarlimit_L 0.01

/* PI parameters */
  // 速度闭环
//#define Kp_Te 2
//#define Ki_Te 0
#define Tecmdlimit_H 4
#define Tecmdlimit_L -4

/* speed ramp */
#define spdramp 200  // 斜率
#define spdlimit_H 1000  // 转速上限
#define spdlimit_L 0  // 转速下限

/* V/spd curve */
//#define VSpdramp 0.38386  // 斜率
#define VSpdramp 0.17  // 斜率
#define Voltlimit_H 100  // 电压上限
#define Voltlimit_L 10  // 电压下限

/* DTC */
#define band_lambdas 0.02
#define band_Te 0.05

//#define VSpdramp 0.38386  // 斜率
#define VSpdramp 0.17  // 斜率
#define Voltlimit_H 100  // 电压上限
#define Voltlimit_L 10  // 电压下限

/* auxiliary */
#define pi 3.1415926
#define Z 1024  // 光电码盘线数
#define digit 1e6  // roundn参数

/******************************************************************************
| types
|----------------------------------------------------------------------------*/
typedef struct
{
  double a, b, c;
} PHASE_ABC;

typedef struct
{
  double al, be;
} PHASE_ALBE;

typedef struct
{
  double d,q;
} PHASE_DQ;

/******************************************************************************
| global variables
|----------------------------------------------------------------------------*/
/* 观测值 */
  // 电压
extern double Udc_IN;
extern PHASE_ABC uabc;
extern PHASE_ALBE ualbe;
extern PHASE_DQ udq;
  // 电流
extern PHASE_ABC iabc;
extern PHASE_ALBE ialbe;
extern PHASE_DQ idq;
  // 磁链
extern double lambdar;
extern PHASE_ALBE lambdaralbe;
extern double lambdas;  // lambdas^2
extern PHASE_ALBE lambdasalbe;
extern double theta;
  // 转矩
extern double Te;
  // 转速
extern double speed;
extern double wr;

/* 给定值 */
  // 电压
extern double u_cmd;
extern PHASE_ALBE ualbe_cmd;
extern PHASE_DQ udq_cmd;
  // 电流
extern PHASE_DQ idq_cmd;
  // 磁链
extern double lambdas_cmd;
  // 转矩
extern double Te_cmd;
  // 占空比
extern PHASE_ABC Dabc;
  // 转速
extern double spd_cmd;  // 转速给定
extern double spd_req;  // 转速设定

/* PI 变量 */
extern double Kp_Tecmd;
extern double Ki_Tecmd;
extern double intgrt_Tecmd;

/******************************************************************************
| local functions prototypes
|----------------------------------------------------------------------------*/
double roundn(double input, int _digit);

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/
/* Forward conversion */  
extern void S3toR2(PHASE_ABC abc, PHASE_DQ *dq, double theta);
extern void S3toS2(PHASE_ABC abc, PHASE_ALBE *albe);
extern void S2toR2(PHASE_ALBE albe, PHASE_DQ *dq, double cosIn, double sinIn);

/* Backward conversion */  
extern void R2toS3(PHASE_DQ dq, PHASE_ABC *abc, double theta);
extern void S2toS3(PHASE_ALBE albe, PHASE_ABC *abc);
extern void R2toS2(PHASE_DQ dq, PHASE_ALBE *albe, double cosIn, double sinIn);

/* calculate lamdar */  
extern double lambdar_cal(double lambdar, double ism);
extern void lambdaralbe_cal(PHASE_ALBE ialbe, PHASE_ALBE *lambdaralbe, double wr);

/* calculate lamdar */
extern void lambdasalbe_voltage(PHASE_ALBE ualbe, PHASE_ALBE ialbe, PHASE_ALBE *lambdasalbe);
extern void lambdasalbe_current(PHASE_ALBE lambdaralbe, PHASE_ALBE *lambdasalbe, PHASE_ALBE ialbe);

/* PI module */  
extern double PImodule(double Kp, double Ki, double err, double *intgrt, double Uplim, double Downlim, double _Ts);
extern double Integrator(double paramin, double sum, double ts);
extern double LPfilter1(double paramin, double lasty, double wc, double ts);
extern double LPfilter2(double paramin, double lasty, double wc, double ts);

/* Relay */
extern int Relay_2Level(double input, double Uplim, double Downlim, int lastout);
extern int Relay_3Level(double input, double Uplim, double Downlim, int lastout);

/* Sector */
extern int sector1(double alpha, double beta);
extern int sector2(double alpha, double beta);

/* SVM */  
extern void positionSVM();
extern void ualbeSVM(double Ual, double Ube, double Ud, PHASE_ABC *Dabc);

/* Auxiliary Function */
extern double RAMP(double ramp, double initial, double increment, double Hlimit, double Llimit);
extern double roundn(double input, int _digit);
