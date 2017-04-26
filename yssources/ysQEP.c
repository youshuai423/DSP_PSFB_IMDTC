/******************************************************************************
| includes
|-------------------------------------------------------------------------------------------*/
#include "ysQEP.h"

/******************************************************************************
| variables
|--------------------------------------------------------------------------------------------*/

/******************************************************************************
| functions
|-------------------------------------------------------------------------------------------*/
void InitQEP1(void)
{
	/**********************QDU1正交解码单元********************/
	EQep1Regs.QDECCTL.bit.QSRC = 0;	//00=正交计数 01=直接计数 10=向上计数 11=向下计数
	EQep1Regs.QDECCTL.bit.SOEN = 0;  // 禁止同步比较输出
	//EQep1Regs.QDECCTL.bit.SPSEL = 0;  // index引脚用于输出（默认）
	EQep1Regs.QDECCTL.bit.XCR = 0;  // 上下边沿计数
	EQep1Regs.QDECCTL.bit.SWAP = 0;  // 正交时钟交换禁止
	EQep1Regs.QDECCTL.bit.IGATE = 0;  //0=禁止索引事件 1=允许索引事件
	EQep1Regs.QDECCTL.bit.QAP = 0;	//0=直接输入 1=反向输入
	EQep1Regs.QDECCTL.bit.QBP = 0;	//0=直接输入 1=反向输入
	EQep1Regs.QDECCTL.bit.QIP = 0;	//0=直接输入 1=反向输入
	EQep1Regs.QDECCTL.bit.QSP = 0;	//0=直接输入 1=反向输入

	//******************PCCU1位置计数器&计数控制单元************////////////
	EQep1Regs.QEPCTL.bit.FREE_SOFT = 0;  //00=立刻停止 01=继续运行  1x=不受影响
	EQep1Regs.QEPCTL.bit.PCRM = 1;  //00=索引事件 01=最大位置 10=首次索引 11=单位时间事件
	EQep1Regs.QEPCTL.bit.SEI = 0;  // 索引事件不初始化位置计数器
	EQep1Regs.QEPCTL.bit.IEI = 0;  //00,01=不动作 10=QEPI1上升沿计数器初始化为QPOSINIT 11=QEPI1下降沿计数器初始化为QPOSINIT
	//EQep1Regs.QEPCTL.bit.SWI = 0;  // 软件初始化位置计数
	EQep1Regs.QEPCTL.bit.IEL= 00;		//00==保留 01=QEPI1上升沿计数器值锁存到QPOSILAT 10=QEPI1下升沿计数器值锁存到QPOSILAT 11=软件索引标记
	EQep1Regs.QEPCTL.bit.QPEN= 1;	//0=软件复位QEP 1=使能
	EQep1Regs.QEPCTL.bit.QCLM = 0;  // CPU读数据锁存数据
	EQep1Regs.QEPCTL.bit.UTE = 0;  // 禁止单位定时器
	EQep1Regs.QEPCTL.bit.WDE = 0;  // 禁止看门狗

	EQep1Regs.QPOSINIT=0;  //初始化位置计数器
	EQep1Regs.QPOSMAX=65000;  // 最大计数
	EQep1Regs.QEPSTS.bit.QDF= 0;
	EQep1Regs.QCAPCTL.bit.CEN = 1;  // 使能QEP捕捉单元
}
