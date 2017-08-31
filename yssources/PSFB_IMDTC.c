/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "PSFB_IMDTC.h"

/******************************************************************************
| local variable definitions
|----------------------------------------------------------------------------*/
int code_start = 0;
int indexDA = 0;
int variables_flag = 0;  // 0——控制启动， 1——DA选择
int received_data = 0;  // RXFIFO数据数

double mSample[5];
Uint32 spd_reg = 0;
int index_lambdas = 0;
int index_Te = 0;
int index_column = 0;
int index_row = 0;
int vector = 0;
//int SwitchTable[6][6] = {
//	5, 6, 1, 2, 3, 4,
//	0, 7, 0, 7, 0, 7,
//	3, 4, 5, 6, 1, 2,
//	6, 1, 2, 3, 4, 5,
//	7, 0, 7, 0, 7, 0,
//	2, 3, 4, 5, 6, 1
//};
int SwitchTable[4][6] = {
	0, 7, 0, 7, 0, 7,
	3, 4, 5, 6, 1, 2,
	7, 0, 7, 0, 7, 0,
	2, 3, 4, 5, 6, 1
};
int switchstate[3][8] = {
	0, 1, 1, 0, 0, 0, 1, 1,
	0, 0, 1, 1, 1, 0, 0, 1,
	0, 0, 0, 0, 1, 1, 1, 1
};
PHASE_ALBE lambdastemp1 = {0, 0};
PHASE_ALBE lambdastemp2 = {0, 0};
PHASE_ALBE lambdasalbe_c = {0, 0};

/******************************************************************************
@brief  Main
******************************************************************************/
void main()
{
   InitSysCtrl();

   DINT;

   InitPieCtrl();

   IER = 0x0000;
   IFR = 0x0000;

   InitPieVectTable();

   EALLOW;
   PieVectTable.EPWM1_INT = &epwm1_timer_isr;  // ePWM1中断入口
   PieVectTable.TINT0 = &ISRTimer0;
   EDIS;

   InitPORT();
   InitPWM();
   InitECAP();
   InitADC();
   InitSCIB();
   InitQEP1();
   InitCpuTimers();  // 计算转速和转速给定值

   ConfigCpuTimer(&CpuTimer0, 150, 10000);  // 10ms
   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0

   IER |= M_INT3;  // enable ePWM CPU_interrupt
   IER |= M_INT1;  // CpuTimer
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // enable ePWM1 pie_interrupt
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
//   while(1)
//   {
//		if(code_start != 0) break;
//		EPwm1Regs.ETCLR.bit.INT = 1;
//		CpuTimer0Regs.TCR.bit.TIF = 1;
//		CpuTimer0Regs.TCR.bit.TRB = 1;
//   }
   EINT;   // 总中断 INTM 使能
   ERTM;   // Enable Global realtime interrupt DBGM

   while(1){}

}

interrupt void epwm1_timer_isr(void)
{
	double Amplambdas = 0;
	double temp = 0, cosphi = 0, sinphi = 0;

    EPwm1Regs.ETCLR.bit.INT = 1;

	// ----------------电压电流采样---------------------
	ParallelRD(mSample, 5);
	iabc.a = LPfilter1(mSample[0] * HallRatioIa, iabc.a, wc_sample, Ts);
	//Ug = LPfilter1(mSample[1] * HallRatioVg, Ug, wc_sample, Ts);  // 采样方向和正方向相同
	iabc.b = LPfilter1(mSample[2] * HallRatioIb, iabc.b, wc_sample, Ts);
	//Ig = LPfilter1(mSample[3] * HallRatioIg, Ig, wc_sample, Ts);
	Udc_IN = LPfilter1(mSample[4] * HallRatioVg, Udc_IN, wc_sample, Ts);
	iabc.c = - iabc.b - iabc.a;  // ??合不合适
   	uabc.a = 2.52 * (Udc_IN - 2) * (Dabc.a - 0.5);  // 2.52 = (1 - Phasft / 180) * 3
   	uabc.b = 2.52 * (Udc_IN - 2) * (Dabc.b - 0.5);
   	uabc.c = 2.52 * (Udc_IN - 2) * (Dabc.c - 0.5);

   	double cosIn = cos(theta);
    double sinIn = sin(theta);

    double Ud = 0;
    Ud = 2.52 * (Udc_IN - 2);
    /* SVM开环计算 */
    u_cmd = RAMP(VSpdramp, 0, spd_cmd, Ud, Voltlimit_L);
    theta += 1.047197551196598e-05 * spd_cmd; // theta += 2 * pi * (spd_cmd / 30.0) * 5e-5;
    if (theta > 6.2831852)  // 2 * pi = 6.2831852
      theta -= 6.2831852;
    ualbe_cmd.al = u_cmd * cosIn;
    ualbe_cmd.be = u_cmd * sinIn;

    ualbeSVM(ualbe_cmd.al, ualbe_cmd.be, Ud, &Dabc);

//	/* 3s/2r coordinate transform */
//	S3toS2(iabc, &ialbe);
//	S3toS2(uabc, &ualbe);
//
//	/* Torque and stator flux calculation */
//	// 1. 低通滤波
//	//lambdasalbeCal(ualbe, ialbe, &lambdasalbe);
//	// 2. 改进积分
////	lambdastemp1.al = LPfilter2(ualbe.al - Rs * ialbe.al, lambdastemp1.al, wc_lambdascal, Ts);
////	lambdastemp1.be = LPfilter2(ualbe.be - Rs * ialbe.be, lambdastemp1.be, wc_lambdascal, Ts);
////	if(lambdas == 0)
////	{
////		cosphi = 1; sinphi = 0;
////	}
////	else
////	{
////		temp = 1 / lambdas;
////		cosphi = lambdasalbe.al * temp;
////		sinphi = lambdasalbe.be * temp;
////	}
////	if(lambdas > lambdas_cmd)  Amplambdas = lambdas_cmd;
////	else  Amplambdas = lambdas;
////	lambdastemp2.al = LPfilter1(Amplambdas * cosphi, lambdastemp2.al, wc_lambdascal, Ts);
////	lambdastemp2.be = LPfilter1(Amplambdas * sinphi, lambdastemp2.be, wc_lambdascal, Ts);
////	lambdasalbe.al = lambdastemp1.al + lambdastemp2.al;  // 先加还是后加？
////	lambdasalbe.be = lambdastemp1.be + lambdastemp2.be;
//
//	// 3.电流型磁链观测器
//	lambdaralbe_cal(ialbe, &lambdaralbe, wr);
//	lambdasalbe_current(lambdaralbe, &lambdasalbe, ialbe);
//
//	lambdas = sqrt(lambdasalbe.al * lambdasalbe.al + lambdasalbe.be * lambdasalbe.be);
//
//	Te = np * (lambdasalbe.al * ialbe.be - lambdasalbe.be * ialbe.al);
//
//	/* ASR */
//	Te_cmd = PImodule(Kp_Tecmd, Ki_Tecmd, spd_cmd - speed, &intgrt_Tecmd, Tecmdlimit_H, Tecmdlimit_L, Ts);
//
//	/* Relay */
//	index_lambdas = Relay_2Level(lambdas_cmd - lambdas, band_lambdas, -band_lambdas, index_lambdas);
//	//index_Te = Relay_3Level(Te_cmd - Te, band_Te, -band_Te, index_Te);  // =============
//	index_Te = Relay_2Level(Te_cmd - Te, band_Te, -band_Te, index_Te);
//
//	/* Switchstate selection */
//	//index_row = (int)(1.5*index_lambdas + index_Te + 2.5); // ==============
//	index_row = (int)(index_lambdas + 0.5 * index_Te + 1.5);
//	index_column = sector2(lambdasalbe.al, lambdasalbe.be) - 1;
//	//if (index_row >= 0 && index_row <= 5 && index_column >= 0 && index_column <= 5)  // ================
//	if (index_row >= 0 && index_row <= 3 && index_column >= 0 && index_column <= 5)
//	{
//		vector = SwitchTable[index_row][index_column];
//		Dabc.a = switchstate[0][vector];
//		Dabc.b = switchstate[1][vector];
//		Dabc.c = switchstate[2][vector];
//	}
//	else
//	{
//		Dabc.a = 0;
//		Dabc.b = 0;
//		Dabc.c = 0;
//	}
//
	if(code_start == 1)
	{
	   	EPwm4Regs.CMPA.half.CMPA = (int)(halfperiod * (1 - Dabc.a));  // AU, CU
	   	EPwm5Regs.CMPA.half.CMPA = (int)(halfperiod * (1 - Dabc.b));
	   	EPwm6Regs.CMPA.half.CMPA = (int)(halfperiod * (1 - Dabc.c));
	}
	else
	{
	   	EPwm4Regs.CMPA.half.CMPA = 0;  // AU, CU
	   	EPwm5Regs.CMPA.half.CMPA = 0;
	   	EPwm6Regs.CMPA.half.CMPA = 0;
	   	lambdasalbe.al = 0;
	   	lambdasalbe.be = 0;
	   	lambdaralbe.al = 0;
	   	lambdaralbe.be = 0;
	}

	switch(indexDA)
	{
		case 0:{DACout(0, ualbe.al * 0.03); DACout(2, ualbe.be * 0.03);break;}
		case 1:{DACout(0, ialbe.al); DACout(2, ialbe.be);break;}
		case 2:{DACout(0, lambdasalbe.al * 5); DACout(2, lambdasalbe.be * 5);break;}
		case 3:{DACout(0, Te); DACout(2, lambdas * 5);break;}
		case 4:{DACout(0, Te_cmd); DACout(2, Te);break;}
		case 5:{DACout(0, spd_cmd * 0.01); DACout(2, speed * 0.01);break;}
		case 6:{DACout(0, (speed) * 0.009); DACout(2, Te);break;}
		default:{DACout(0, 0); DACout(1, 0);}
	}

   // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

interrupt void ISRTimer0(void)
{
	int temp = 0;

	CpuTimer0.InterruptCount ++;
	if (CpuTimer0.InterruptCount  > 15) CpuTimer0.InterruptCount -= 16;

	if(EQep1Regs.QFLG.bit.PCO == 1 && EQep1Regs.QPOSCNT < spd_reg)
	{
		speed = (int)(EQep1Regs.QPOSCNT + 65000 - spd_reg) * (1.4648);  // 1024线，1.4648 = 1/(1024*4)*60 / 0.01
		EQep1Regs.QCLR.bit.PCO = 1;
	}
	else if(EQep1Regs.QFLG.bit.PCU == 1 && EQep1Regs.QPOSCNT > spd_reg)
	{
		speed = (int)(spd_reg + 65000 - EQep1Regs.QPOSCNT) * (-1.4648);
		EQep1Regs.QCLR.bit.PCU = 1;
	}
	else
		speed = ((int)EQep1Regs.QPOSCNT - (int)spd_reg) * (1.4648);

	wr = speed * 0.209439510239320;  // wr = speed / 60 * 2*pi * 2;

	spd_reg = EQep1Regs.QPOSCNT;

	if (spd_cmd < spd_req && code_start == 1)
	{
		spd_cmd = RAMP(spdramp, spd_cmd, 0.01, spdlimit_H, spdlimit_L);  // 转速给定值计算
	}

	if(ScibRegs.SCIFFRX.bit.RXFFST == 0)
		received_data = 0;
	else if(ScibRegs.SCIFFRX.bit.RXFFST == 2)
	{
		// 接受控制对象
		scib_rx(&temp);
		switch(temp)
		{
		case 0xff: variables_flag = 0; break;
		case 0xfe: variables_flag = 1; break;
		}
		// 接受控制参数
		scib_rx(&temp);
		switch(variables_flag)
		{
		case 0: code_start = temp; break;
		case 1: indexDA = temp; break;
		}

		//scib_tx(1);
	}
	else if(received_data == 1 && ScibRegs.SCIFFRX.bit.RXFFST == 1)
	{
		received_data = 0;
		ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;  // reset RXFIFO
		ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;  // reenable RXFIFO
	}
	else if(ScibRegs.SCIFFRX.bit.RXFFST == 1)
		received_data = 1;
	else
	{
		received_data = 0;
		ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;  // reset RXFIFO
		ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;  // reenable RXFIFO
	}

	// Acknowledge this interrupt to receive more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	CpuTimer0Regs.TCR.bit.TIF=1;
	CpuTimer0Regs.TCR.bit.TRB=1;
}

