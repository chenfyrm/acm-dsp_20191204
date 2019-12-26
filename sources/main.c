//################################################################################
// FILE:    Main.c
// TITLE:	NUGTCU_DSP2
// DESCRIPTION: NUGTCU,T=250uS
//################################################################################
/*
 * HISTORY
 *
 *
 *
 * */

/* INCLUDES */
#include "DSP28x_Project.h"
#include "math.h"
#include "acm_dsp.h"

/* EXTERNAL FUNCTION PROTOTYPES */

/* DESCRIPTION */

/* DEFINES */
//--------------------------------------------------------------------------------
/* TYPEDEFS */

/*STRUCTDEFS*/
//-----------------input-------------------------
//闂佹彃娲﹂悧閬嶅Υ娴ｇ懓鐦瑰ù鐙呮嫹
struct MCUFLAG1_BITS {
	Uint16 RstSa :1;
	Uint16 OvpFcTsAv :1;
	Uint16 CdAuLdCt :1;
	Uint16 RstFlt :1;
	Uint16 BtCpAv :1;
	Uint16 CvOp :1;
	Uint16 OvpCpAv :1;
	Uint16 TA7 :1;
	Uint16 TA8 :1;
	Uint16 TA9 :1;
	Uint16 TA10 :1;
	Uint16 TA11 :1;
	Uint16 TA12 :1;
	Uint16 TA13 :1;
	Uint16 TA14 :1;
	Uint16 TA15 :1;
};

union MCUFLAG1_REG {
	Uint16 all;
	struct MCUFLAG1_BITS bit;
};

struct PX_In {
	float32 XU_DcLk;   		    		// DC-link voltage, V
	float32 XI_DcLk;   				// DC current, A
	float32 XI_PhA;						// phase A current, A
	float32 XI_PhB;						// phase B current, A
	float32 XI_PhC;			  	  		// phase C current, A
	float32 XU_PhABGt;					// AB闁烩晛鎽滈崵搴ㄦ偨闂堟稑绔 V
	Uint16 NX_McuPlCn;				// MCU pulse(heartbeat) counter
	Uint16 NX_McuOpSt;				// MCU operation state
	Uint16 NX_McuVer;			// MCU version
	union MCUFLAG1_REG XX_McuFlag1;
};
volatile struct PX_In PX_In_Spf = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0x400,
		0x10, 0x00, };
//========================================================================
//濞ｅ洦绻冩慨銏ゅ磹閿
struct PX_InPr {
	float32 XU_DcLk_Max;   			// DC-link voltage, V
	float32 XU_DcLk_Min;   			// DC-link voltage, V
	Uint16 XU_DcLkOvCn;   			// DC-link over-voltage counter
	Uint16 XU_DcLkUnCn;   			// DC-link under-voltage counter
	float32 XI_DcLk_Max;   			// DC current, A
	Uint16 XI_DcLkOvCn;   			// DC-link over-current counter
	float32 XI_PhABC_Max;				// phase ABC maximum current, A
	Uint16 XI_PhAOvCn;				// phase A over current counter
	Uint16 XI_PhBOvCn;				// phase B over current counter
	Uint16 XI_PhCOvCn;				// phase C over current counter
	float32 XH_AmTp_Max;         		// ambient temperature, C
};
volatile struct PX_InPr PX_InPr_Spf = { 1800.0,         		//
		0.0,         		//
		0, 0, 10.0,         		//
		0, 300.0,         		//
		0, 0, 0, 0.0, };         		//
//-------------output-------------
//============================================================================================
struct WARN_BITS {
	Uint16 TA0 :1;		//
	Uint16 TA1 :1;		//
	Uint16 TA2 :1;		// Idc
	Uint16 TA3 :1;		// Udc
	Uint16 TA4 :1;		// DSP
	Uint16 TA5 :1;		//A
	Uint16 TA6 :1;		//B
	Uint16 TA7 :1;		//C
	Uint16 TA8 :1;		// Iac
	Uint16 TA9 :1;		// Ia
	Uint16 TA10 :1;		// Ib
	Uint16 TA11 :1;		// Ic
	Uint16 TA12 :1;		// Ia
	Uint16 TA13 :1;		// Ib
	Uint16 TA14 :1;		// Ic
	Uint16 TA15 :1;		// Iac
};
union WARN_REG {
	Uint16 all;
	struct WARN_BITS bit;
};

struct DSPFLAG1_BITS {
	Uint16 CdAuLdCt :1;
	Uint16 CvOp :1;
	Uint16 OvpFcTs :1;
	Uint16 OvpCpAv :1;
	Uint16 IPhClTrsAv :1;
	Uint16 OvMdAv :1;
	Uint16 BtCpAv :1;
	Uint16 TA7 :1;
	Uint16 TA8 :1;
	Uint16 TA9 :1;
	Uint16 TA10 :1;
	Uint16 TA11 :1;
	Uint16 TA12 :1;
	Uint16 TA13 :1;
	Uint16 TA14 :1;
	Uint16 TA15 :1;
};

union DSPFLAG1_REG {
	Uint16 all;
	struct DSPFLAG1_BITS bit;
};

struct DSPST_BITS {
	Uint16 CvSt :8;
	Uint16 OvpCp :4;
	Uint16 BtCp :4;
};

union DSPST_REG {
	Uint16 all;
	struct DSPST_BITS bit;
};

//閺夊牊鎸搁崵顓熺閵堝嫮闉嶉柛濠忔嫹
struct PX_Out {
	Uint16 XX_PwmMo;			// PWM mode  0x00  0x15
	Uint16 XT_PwmPdVv;   		// PWM period value,  for initialization
	Uint16 XX_Pwm1AVv;   		// PWM1A value
	Uint16 XX_Pwm2AVv;			// PWM2A value
	Uint16 XX_Pwm3AVv;			// PWM3A value
	Uint16 XX_Pwm4AVv;			// PWM4A value, chopper
	Uint16 XX_Pwm4BVv;			// PWM4B value, chopper
	float32 XU_PhAB_Rms;
	float32 XF_PhAB;
	float32 XI_PhA_Rms;		    // phase A current, RMS
	float32 XI_PhB_Rms;	    	// phase B current, RMS
	float32 XI_PhC_Rms;		    // phase C current, RMS
	float32 XP_Out;		    	// output power, kw
	float32 XQ_Out;
	float32 XI_DcLkEst;
	float32 XX_PhUbl;			// phase unbalance
	union WARN_REG XX_Flt1;			// 16
	Uint16 SX_Run;
	Uint16 NX_DspPlCn;			// DSP2 pulse(heartbeat) counter
	union DSPST_REG NX_DspOpSt;	// NX_Dsp2OpSt: DSP2 operation state 0x11:
	union DSPST_REG oldDspSt;
	Uint16 NX_DspVer;			// DSP2 version
	union DSPFLAG1_REG XX_DspFlag1;
};
volatile struct PX_Out PX_Out_Spf = { 21,
		6944,			//450Hz:6465 1350Hz:6944
		3472, 3472, 3472, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0000, 0, 0, 0x0000,
		0x0000, 0x10, 0x0000, };
//======================================================================
/* GLOBALS */

/* LOCALS */
//int16	*XintfZone0=(int16 *)0x004000;//Unused
//int16	*XintfZone6=(int16 *)0x100000;//Unused
int16 *XintfZone7 = (int16 *) 0x200000;			//DP RAM
//=======================================================================
Uint16 GPIO_Temp181, GPIO_Temp182;

//===========================================================================
Uint16 Cnt_Period = 0;
Uint16 Cnt_us = 0;
Uint16 Cnt_sec = 0;
Uint16 Cnt_min = 0;
Uint16 Hold = 0;
volatile Uint16 Cnt_B = 0;
volatile Uint16 Cnt_1ms = 0;
volatile Uint16 Cnt_4ms = 0;
volatile Uint16 Cnt_16ms = 0;
volatile float32 Ext_U = 0.0;
volatile float32 Ext_F = 0.0;
volatile Uint16 TestFlg = 0;

int16 DA[8] = { 0 };

/* LOCAL FUNCTIONS */
void DPRAM_RD(void);
void DPRAM_WR(void);
void NX_Pr(void);
void EN_GPIO30(void);
void DIS_GPIO30(void);
interrupt void DPRAM_isr(void);
interrupt void DPRAM_isr_Fix(void);
void DspStCl(void);
void OvpFcTsCp(void);
void OvpCp(void);
void BtCp(void);

//=================================================================================
/* MAIN */
void main(void) {

	InitSysCtrl();
	InitGpio();
	InitXintf();

	DELAY_US(10000L);

	DINT;
	IER = 0x0000;
	IFR = 0x0000;
	InitPieCtrl();
	InitPieVectTable();

	EALLOW;
	//	PieVectTable.XINT1 = &DPRAM_isr;
	PieVectTable.XINT1 = &DPRAM_isr_Fix;
	EDIS;

	InitCpuTimers();
	InitXInterrupt();
	IER = M_INT1;
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;

	EINT;
	ERTM;

	DspInit();
	McuInit();

	while (1) {
		//--------------------------------------------------------------------------------
		GPIO_Temp181 = GpioDataRegs.GPADAT.bit.GPIO18;			//
		DELAY_US(2L);
		//
		GPIO_Temp182 = GpioDataRegs.GPADAT.bit.GPIO18;
		if ((GPIO_Temp181 == 0) && (GPIO_Temp182 == 0)) {
			PX_In_Spf.NX_McuPlCn = *(XintfZone7 + 0x7FFF);			//
		}
	}
}
//==============================================================================
interrupt void DPRAM_isr(void) //after DSP1 has written to DPRAM, trigger the interrupt
{
	/*保护现场*/

	/**/
	PX_Out_Spf.NX_DspPlCn++;
	if (PX_Out_Spf.NX_DspPlCn > 32767)
		PX_Out_Spf.NX_DspPlCn = 0;

	DIS_GPIO30();

	DPRAM_RD(); //

	NX_Pr();

	DspStCl();



	if (!(TestFlg == 0x55)) {
		DspTask_B();

		Cnt_1ms++;
		if (Cnt_1ms >= 3) {

			DspTask_T2();

			Cnt_1ms = 0;
		}

		Cnt_4ms++;
		if (Cnt_4ms >= 11) {

			McuTask_4ms();

			Cnt_4ms = 0;
		}

		Cnt_16ms++;
		if (Cnt_16ms >= 44) {

			McuParam.PF_3PhNom = Limit(Ext_F, 40.0, 60.0);
			McuParam.PU_U3PhRef3 = Limit(Ext_U, 50.0, 380.0) * SQRT2bySQRT3;
			McuParam.PU_U3PhRef4 = McuParam.PU_U3PhRef3;

			McuTask_16ms();

			Cnt_16ms = 0;

		}
	} else {
		/*
		 *
		 ************************************************************* */
		PX_Out_Spf.XT_PwmPdVv = 6944;
		DspData.XX_Mode = !DspData.XX_Mode;
		DspData.XX_DutyA = 0.2;
		DspData.XX_DutyB = 0.3;
		DspData.XX_DutyC = 0.6;
		/*******************************************************************************/

	}

	/**/
	if (PX_Out_Spf.SX_Run == 1) {
		if (DspData.XX_Mode == 1) {
			PX_Out_Spf.XX_PwmMo = 21; //
			PX_Out_Spf.XX_Pwm1AVv = PX_Out_Spf.XT_PwmPdVv
					* (1.0 - DspData.XX_DutyA);
			PX_Out_Spf.XX_Pwm2AVv = PX_Out_Spf.XT_PwmPdVv
					* (1.0 - DspData.XX_DutyB);
			PX_Out_Spf.XX_Pwm3AVv = PX_Out_Spf.XT_PwmPdVv
					* (1.0 - DspData.XX_DutyC);
		}
		if (DspData.XX_Mode == 0) {
			PX_Out_Spf.XX_PwmMo = 0; //
			PX_Out_Spf.XX_Pwm1AVv = PX_Out_Spf.XT_PwmPdVv * DspData.XX_DutyA;
			PX_Out_Spf.XX_Pwm2AVv = PX_Out_Spf.XT_PwmPdVv * DspData.XX_DutyB;
			PX_Out_Spf.XX_Pwm3AVv = PX_Out_Spf.XT_PwmPdVv * DspData.XX_DutyC;
		}
	} else {
		PX_Out_Spf.XX_PwmMo = 0;
		PX_Out_Spf.XX_Pwm1AVv = PX_Out_Spf.XT_PwmPdVv * 0.5;
		PX_Out_Spf.XX_Pwm2AVv = PX_Out_Spf.XT_PwmPdVv * 0.5;
		PX_Out_Spf.XX_Pwm3AVv = PX_Out_Spf.XT_PwmPdVv * 0.5;
	}

	PX_Out_Spf.XU_PhAB_Rms = DspData.XU_3PhAbs / SQRT2;
	PX_Out_Spf.XF_PhAB = DspData.XF_U3Ph;
	PX_Out_Spf.XI_PhA_Rms = DspData.XI_Ph1Rms_Flt;
	PX_Out_Spf.XI_PhB_Rms = DspData.XI_Ph2Rms_Flt;
	PX_Out_Spf.XI_PhC_Rms = DspData.XI_Ph3Rms_Flt;
	PX_Out_Spf.XP_Out = DspData.XP_3Ph_Flt;
	PX_Out_Spf.XQ_Out = DspData.XQ_3Ph_Flt;
	PX_Out_Spf.XI_DcLkEst = DspData.XP_3Ph_Flt / DspData.XU_DcLkFlt;

	DPRAM_WR(); //

	EN_GPIO30();

	PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

//==============================================================================
interrupt void DPRAM_isr_Fix(void) //固定中断频率4.35kHz
{
	/*保护现场*/

	/**/
	PX_Out_Spf.NX_DspPlCn++;
	if (PX_Out_Spf.NX_DspPlCn > 32767)
		PX_Out_Spf.NX_DspPlCn = 0;

	DIS_GPIO30();

	//	PX_In_Spf.NX_McuPlCn = *(XintfZone7 + 0x7FFF); // MCU pulse(heartbeat) counter    (RAM 0x7FFF clear)
	//	DspData.XU_PhABLk = *(XintfZone7 + 0x7) * 0.1 * 2.0;		// AB
	//
	//	Cnt_Period++;
	//	if (Cnt_Period >= 2700) {
	//		Cnt_sec++;
	//		Cnt_Period = 0;
	//	}
	//	if(Cnt_sec>=60.0){
	//		Cnt_min++;
	//		Cnt_sec = 0;
	//	}



	//	Cnt_B++;
	//	if (Cnt_B >= 1) {
	DPRAM_RD(); //
	NX_Pr();
	DspStCl();
	//		if (!(TestFlg == 0x55)) {

	DspTask_185us();

	DspTask_B();

	Cnt_1ms++;
	if (Cnt_1ms >= 3) {
		DspTask_T2();
		Cnt_1ms = 0;
	}

	Cnt_4ms++;
	if (Cnt_4ms >= 11) {
		McuTask_4ms();
		Cnt_4ms = 0;
	}

	Cnt_16ms++;
	if (Cnt_16ms >= 44) {
		McuParam.PF_3PhNom = Limit(Ext_F, 40.0, 60.0);
		McuParam.PU_U3PhRef3 = Limit(Ext_U, 50.0, 380.0) * SQRT2bySQRT3;
		McuParam.PU_U3PhRef4 = McuParam.PU_U3PhRef3;
		McuTask_16ms();
		Cnt_16ms = 0;
	}
	//		} else {
		//			/*
		//			 *
	//			 ************************************************************* */
	//			PX_Out_Spf.XT_PwmPdVv = 6944;
	//			DspData.XX_Mode = !DspData.XX_Mode;
	//			DspData.XX_DutyA = 0.2;
	//			DspData.XX_DutyB = 0.3;
	//			DspData.XX_DutyC = 0.6;
	//			/*******************************************************************************/
	//		}

	/**/
	if (PX_Out_Spf.SX_Run == 1) {
		if (DspData.XX_Mode == 1) {
			PX_Out_Spf.XX_PwmMo = 21; //
			PX_Out_Spf.XX_Pwm1AVv = PX_Out_Spf.XT_PwmPdVv
					* (1.0 - DspData.XX_DutyA);
			PX_Out_Spf.XX_Pwm2AVv = PX_Out_Spf.XT_PwmPdVv
					* (1.0 - DspData.XX_DutyB);
			PX_Out_Spf.XX_Pwm3AVv = PX_Out_Spf.XT_PwmPdVv
					* (1.0 - DspData.XX_DutyC);
		}
		if (DspData.XX_Mode == 0) {
			PX_Out_Spf.XX_PwmMo = 0; //
			PX_Out_Spf.XX_Pwm1AVv = PX_Out_Spf.XT_PwmPdVv
					* DspData.XX_DutyA;
			PX_Out_Spf.XX_Pwm2AVv = PX_Out_Spf.XT_PwmPdVv
					* DspData.XX_DutyB;
			PX_Out_Spf.XX_Pwm3AVv = PX_Out_Spf.XT_PwmPdVv
					* DspData.XX_DutyC;
		}
	} else {
		PX_Out_Spf.XX_PwmMo = 0;
		PX_Out_Spf.XX_Pwm1AVv = PX_Out_Spf.XT_PwmPdVv * 0.5;
		PX_Out_Spf.XX_Pwm2AVv = PX_Out_Spf.XT_PwmPdVv * 0.5;
		PX_Out_Spf.XX_Pwm3AVv = PX_Out_Spf.XT_PwmPdVv * 0.5;
	}

	PX_Out_Spf.XU_PhAB_Rms = DspData.XU_3PhAbs / SQRT2;
	PX_Out_Spf.XF_PhAB = DspData.XF_U3Ph;
	PX_Out_Spf.XI_PhA_Rms = DspData.XI_Ph1Rms_Flt;
	PX_Out_Spf.XI_PhB_Rms = DspData.XI_Ph2Rms_Flt;
	PX_Out_Spf.XI_PhC_Rms = DspData.XI_Ph3Rms_Flt;
	PX_Out_Spf.XP_Out = DspData.XP_3Ph_Flt;
	PX_Out_Spf.XQ_Out = DspData.XQ_3Ph_Flt;
	PX_Out_Spf.XI_DcLkEst = DspData.XP_3Ph_Flt / DspData.XU_DcLkFlt;

	DPRAM_WR(); //
	//		Cnt_B = 0;
	//	}

	EN_GPIO30();
	PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}
//==============================================================================
void DPRAM_RD(void) //MCU-->DSP
{
	//----------------------------------------------------
	PX_In_Spf.NX_McuPlCn = *(XintfZone7 + 0x7FFF); // MCU pulse(heartbeat) counter    清除右侧中断标志
	//---------------------------------------------------------------------
	PX_In_Spf.NX_McuOpSt = *(XintfZone7 + 0x0001);		// MCU operation state
	//	PX_In_Spf.NX_McuVer = 0x10;
	PX_In_Spf.XX_McuFlag1.all = *(XintfZone7 + 0x0014);

	DspData.XU_DcLk = *(XintfZone7 + 0x6) * 0.1 * 2.0;	// DC-link voltage, V
	DspData.XI_DcLk = 0;		// DC-link current, V
	DspData.XI_PhA = *(XintfZone7 + 0x8) * 0.1 / 2.0 * (-1.0);// phase A current, A
	DspData.XI_PhB = *(XintfZone7 + 0xA) * 0.1 / 2.0 * (-1.0);// phase B current, A
	DspData.XI_PhC = *(XintfZone7 + 0x9) * 0.1 / 2.0 * (-1.0);// phase C current, A
	DspData.XU_PhABLk = *(XintfZone7 + 0x7) * 0.1 * 2.0;		// AB
	Ext_U = *(XintfZone7 + 0x11) * 10.0;		//电压
	Ext_F = *(XintfZone7 + 0x12) * 1.0;		//频率
	TestFlg = *(XintfZone7 + 0x1A);		//0x55为测试
}
//==============================================================================
void DPRAM_WR(void)			//DSP-->MCU
{

	/*
	 *
	 *
	 *
	 * */
	*(XintfZone7 + 0x2) = PX_Out_Spf.NX_DspOpSt.all; // NX_Dsp2OpSt: DSP2 operation state
	*(XintfZone7 + 0x3) = PX_Out_Spf.NX_DspVer;		// DSP version
	*(XintfZone7 + 0x15) = PX_Out_Spf.XX_PwmMo;     // PWM  mode configuration
	*(XintfZone7 + 0x16) = PX_Out_Spf.XT_PwmPdVv;			// PWM period value
	*(XintfZone7 + 0x17) = PX_Out_Spf.XX_Pwm1AVv;			// PWM1A value
	*(XintfZone7 + 0x18) = PX_Out_Spf.XX_Pwm2AVv;			// PWM2A value
	*(XintfZone7 + 0x19) = PX_Out_Spf.XX_Pwm3AVv;			// PWM3A value
	*(XintfZone7 + 0x1A) = PX_Out_Spf.XX_Pwm4AVv;		// PWM4A value, chopper
	*(XintfZone7 + 0x1B) = PX_Out_Spf.XX_Pwm4BVv;

	*(XintfZone7 + 0x1C) = PX_Out_Spf.XU_PhAB_Rms;
	*(XintfZone7 + 0x1D) = PX_Out_Spf.XF_PhAB;
	*(XintfZone7 + 0x1E) = PX_Out_Spf.XI_PhA_Rms;
	*(XintfZone7 + 0x1F) = PX_Out_Spf.XI_PhB_Rms;
	*(XintfZone7 + 0x20) = PX_Out_Spf.XI_PhC_Rms;
	*(XintfZone7 + 0x21) = PX_Out_Spf.XP_Out;
	*(XintfZone7 + 0x22) = PX_Out_Spf.XQ_Out;
	*(XintfZone7 + 0x23) = PX_Out_Spf.XI_DcLkEst;

	*(XintfZone7 + 0x24) = PX_Out_Spf.NX_DspOpSt.all;
	*(XintfZone7 + 0x25) = PX_Out_Spf.XX_DspFlag1.all;

	*(XintfZone7 + 0x26) = PX_Out_Spf.XX_Flt1.all;		// DSP


	*(XintfZone7 + 0x27) = fabs(McuData.WU_3PhDsp);
	*(XintfZone7 + 0x28) = fabs(DspData.WU_3PhAbs);
	*(XintfZone7 + 0x29) = fabs(DspData.XU_3PhAbs);
	*(XintfZone7 + 0x2A) = fabs(PI_U3PhCl.Out );


	/*
	 *
	 * */
	DA[3] = 0.0;
	DA[4] = 0.0;
	DA[5] = 0.0;
	DA[6] = 0.0;
	DA[7] = 0.0;

	if (DA[3] >= 4095)
		DA[3] = 4095;
	if (DA[3] <= -4095)
		DA[3] = -4095;
	if (DA[4] >= 4095)
		DA[4] = 4095;
	if (DA[4] <= -4095)
		DA[4] = -4095;
	if (DA[5] >= 4095)
		DA[5] = 4095;
	if (DA[5] <= -4095)
		DA[5] = -4095;
	if (DA[6] >= 4095)
		DA[6] = 4095;
	if (DA[6] <= -4095)
		DA[6] = -4095;
	if (DA[7] >= 4095)
		DA[7] = 4095;
	if (DA[7] <= -4095)
		DA[7] = -4095;
	*(XintfZone7 + 0x2B) = DA[3];
	*(XintfZone7 + 0x2C) = DA[4];
	*(XintfZone7 + 0x2D) = DA[5];
	*(XintfZone7 + 0x2E) = DA[6];
	*(XintfZone7 + 0x2F) = DA[7];

	//---------------------------------------------------
	*(XintfZone7 + 0x7FFE) = PX_Out_Spf.NX_DspPlCn;		// 右侧写7FFE，左侧产生中断
	//------------------------------------------------------------

}
//==============================================================================
/* */
void NX_Pr(void) {
	//-------------------Udc over-voltage--------------------
	if (PX_In_Spf.XU_DcLk > PX_InPr_Spf.XU_DcLk_Max) {
		PX_InPr_Spf.XU_DcLkOvCn++;
		if (PX_InPr_Spf.XU_DcLkOvCn > 3) {
			PX_Out_Spf.SX_Run = 0;
			PX_Out_Spf.XX_Flt1.bit.TA0 = 1;
			PX_InPr_Spf.XU_DcLkOvCn = 4;
		}
	} else
		PX_InPr_Spf.XU_DcLkOvCn = 0;

	//------------------Udc under-voltage---------------------
	if ((PX_In_Spf.XU_DcLk < PX_InPr_Spf.XU_DcLk_Min)
			&& PX_Out_Spf.SX_Run == 1) {
		PX_InPr_Spf.XU_DcLkUnCn++;
		if (PX_InPr_Spf.XU_DcLkUnCn > 3) {
			PX_Out_Spf.SX_Run = 0;
			PX_Out_Spf.XX_Flt1.bit.TA0 = 1;
			PX_InPr_Spf.XU_DcLkUnCn = 4;
		}
	} else
		PX_InPr_Spf.XU_DcLkUnCn = 0;

	//-----------------------DC-link over-current--------------------------------
	if (PX_In_Spf.XI_DcLk > PX_InPr_Spf.XI_DcLk_Max) {
		PX_InPr_Spf.XI_DcLkOvCn++;
		if (PX_InPr_Spf.XI_DcLkOvCn > 3) {
			PX_Out_Spf.SX_Run = 0;
			PX_Out_Spf.XX_Flt1.bit.TA1 = 1;
			PX_InPr_Spf.XI_DcLkOvCn = 4;
		}
	} else
		PX_InPr_Spf.XI_DcLkOvCn = 0;

	//-----------------------phase A over-current------------------------
	if ((PX_In_Spf.XI_PhA > PX_InPr_Spf.XI_PhABC_Max)
			|| (PX_In_Spf.XI_PhA < -PX_InPr_Spf.XI_PhABC_Max)) {
		PX_InPr_Spf.XI_PhAOvCn++;
		if (PX_InPr_Spf.XI_PhAOvCn > 3) {
			PX_Out_Spf.SX_Run = 0;
			PX_Out_Spf.XX_Flt1.bit.TA5 = 1;
			PX_InPr_Spf.XI_PhAOvCn = 4;
		}
	} else
		PX_InPr_Spf.XI_PhAOvCn = 0;

	//-----------------------phase B over-current------------------------
	if ((PX_In_Spf.XI_PhB > PX_InPr_Spf.XI_PhABC_Max)
			|| (PX_In_Spf.XI_PhB < -PX_InPr_Spf.XI_PhABC_Max)) {
		PX_InPr_Spf.XI_PhBOvCn++;
		if (PX_InPr_Spf.XI_PhBOvCn > 3) {
			PX_Out_Spf.SX_Run = 0;
			PX_Out_Spf.XX_Flt1.bit.TA6 = 1;
			PX_InPr_Spf.XI_PhBOvCn = 4;
		}
	} else
		PX_InPr_Spf.XI_PhBOvCn = 0;

	//-----------------------phase C over-current------------------------
	if ((PX_In_Spf.XI_PhC > PX_InPr_Spf.XI_PhABC_Max)
			|| (PX_In_Spf.XI_PhC < -PX_InPr_Spf.XI_PhABC_Max)) {
		PX_InPr_Spf.XI_PhCOvCn++;

		if (PX_InPr_Spf.XI_PhCOvCn > 3) {
			PX_Out_Spf.SX_Run = 0;
			PX_Out_Spf.XX_Flt1.bit.TA7 = 1;
			PX_InPr_Spf.XI_PhCOvCn = 4;
		}
	} else
		PX_InPr_Spf.XI_PhCOvCn = 0;

}
//==============================================================================
void EN_GPIO30(void) {
	EALLOW;
	GpioDataRegs.GPASET.bit.GPIO30 = 1;
	EDIS;
}
//==============================================================================
void DIS_GPIO30(void) {
	EALLOW;
	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
	EDIS;
}

//=================
void OvpCp(void) {

}
//=================
void BtCp(void) {

}
//==================
void DspStCl(void)
{

	if (PX_In_Spf.XX_McuFlag1.bit.RstSa == 1)
	{
		PX_Out_Spf.XX_DspFlag1.all = 0x0000;
		PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x0000;
	}

	if (PX_In_Spf.XX_McuFlag1.bit.RstFlt == 1)
	{
		PX_Out_Spf.XX_Flt1.all = 0x0000;
	}

	//PrBc
	if ((PX_In_Spf.NX_McuOpSt == 0x417) || (PX_In_Spf.NX_McuOpSt == 0x41C)
			|| (PX_In_Spf.NX_McuOpSt == 0x41A)
			|| (PX_In_Spf.NX_McuOpSt == 0x426)
			|| (PX_In_Spf.NX_McuOpSt == 0x41E))
	{
		if ((PX_In_Spf.XX_McuFlag1.bit.CdAuLdCt == 0)
				&& (PX_Out_Spf.NX_DspOpSt.bit.CvSt > 0x0030))
		{
			PX_Out_Spf.SX_Run = 0;
			PX_Out_Spf.XX_DspFlag1.all = 0x0000;
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x0020;
			if (PX_In_Spf.XX_McuFlag1.bit.OvpCpAv == 1)
			{
				PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x4;
			}
		}
	}

	//	//PrSd
	//	if (PX_In_Spf.NX_McuOpSt == 0x41C) {
	//		if ((PX_In_Spf.XX_McuFlag1.bit.CdAuLdCt == 0)
	//				&& (PX_Out_Spf.NX_DspOpSt.bit.CvSt > 0x0030)) {
	//			PX_Out_Spf.XX_DspFlag1.all = 0x0000;
	//			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x0020;
	//			if (PX_In_Spf.XX_McuFlag1.bit.OvpCpAv == 1) {
	//				PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x4;
	//			}
	//		}
	//
	//	}
	//
	//	//SfSd
	//	if (PX_In_Spf.NX_McuOpSt == 0x41A) {
	//		if ((PX_In_Spf.XX_McuFlag1.bit.CdAuLdCt == 0)
	//				&& (PX_Out_Spf.NX_DspOpSt.bit.CvSt > 0x0030)) {
	//			PX_Out_Spf.XX_DspFlag1.all = 0x0000;
	//			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x0020;
	//			if (PX_In_Spf.XX_McuFlag1.bit.OvpCpAv == 1) {
	//				PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x4;
	//			}
	//		}
	//	}
	//
	//	//FsSd
	//	if (PX_In_Spf.NX_McuOpSt == 0x426) {
	//		if ((PX_In_Spf.XX_McuFlag1.bit.CdAuLdCt == 0)
	//				&& (PX_Out_Spf.NX_DspOpSt.bit.CvSt > 0x0030)) {
	//			PX_Out_Spf.XX_DspFlag1.all = 0x0000;
	//			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x0020;
	//			if (PX_In_Spf.XX_McuFlag1.bit.OvpCpAv == 1) {
	//				PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x4;
	//			}
	//		}
	//	}
	//
	//	//Inso
	//	if (PX_In_Spf.NX_McuOpSt == 0x41E) {
	//		if (PX_In_Spf.XX_McuFlag1.bit.CdAuLdCt == 0) {
	//			PX_Out_Spf.XX_DspFlag1.all = 0x0000;
	//			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x0020;
	//		}
	//		if (PX_In_Spf.XX_McuFlag1.bit.OvpCpAv == 1) {
	//			PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x4;
	//		}
	//	}

	//-------------------------------------------
	//
	if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x00)
	{
		//
		//
		if (PX_In_Spf.NX_McuOpSt == 0x402)
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x10;
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x10)
	{
		//---
		//---
		if (PX_In_Spf.NX_McuOpSt == 0x403)
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x20;
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x20)
	{
		//
		DspInit();
		McuInit();
		PX_Out_Spf.XX_DspFlag1.all = 0x0000;
		PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x30;
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x30)
	{
		if ((PX_In_Spf.NX_McuOpSt == 0x408)
				&& (PX_In_Spf.XX_McuFlag1.bit.CvOp == 1))
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x40;

		}
	}

	//------------------------------------------------------------
	//
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x40)
	{
		if ((PX_In_Spf.NX_McuOpSt == 0x408)
				&& (PX_In_Spf.XX_McuFlag1.bit.CvOp == 1))
		{
			if (DspData.XU_3PhRms < 50.0)
			{
				PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x41;
			}
			if (DspData.XU_3PhRms > 370.0)
			{
				PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x42;
			}
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x41)
	{
		PX_Out_Spf.XX_DspFlag1.bit.CdAuLdCt = 1;
		if ((PX_In_Spf.NX_McuOpSt == 0x408)
				&& (PX_In_Spf.XX_McuFlag1.bit.CvOp == 1))
		{
			if (PX_In_Spf.XX_McuFlag1.bit.CdAuLdCt == 1)
				PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x42;
		}
		else
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x40;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x42)
	{
		if ((PX_In_Spf.NX_McuOpSt == 0x408)
				&& (PX_In_Spf.XX_McuFlag1.bit.CvOp == 1))
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x43;
		}
		else
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x40;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x43)
	{
		McuData.C_FRmp = TRUE;
		PX_Out_Spf.SX_Run = 1;
		PX_Out_Spf.XX_DspFlag1.bit.CvOp = 1;
		if ((PX_In_Spf.NX_McuOpSt == 0x408)
				&& (PX_In_Spf.XX_McuFlag1.bit.CvOp == 1))
		{
			if (PX_Out_Spf.SX_Run == 1)
				PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x44;
		}
		else
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x40;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x44)
	{
		if ((PX_In_Spf.NX_McuOpSt == 0x408)
				&& (PX_In_Spf.XX_McuFlag1.bit.CvOp == 1))
		{
			if (McuData.A_FNom == 1)
				PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x45;
		}
		else
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x40;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x45)
	{
		//
		//
		if ((PX_In_Spf.NX_McuOpSt == 0x408)
				&& (PX_In_Spf.XX_McuFlag1.bit.CvOp == 1))
		{
			if (PX_In_Spf.XX_McuFlag1.bit.CdAuLdCt == 1)
			{
				PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x48;

			}
			else
			{
				PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x46;

			}
		}
		else
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x40;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x46)
	{

		//
		McuData.C_AuSz = 1;
		if ((PX_In_Spf.NX_McuOpSt == 0x408)
				&& (PX_In_Spf.XX_McuFlag1.bit.CvOp == 1))
		{

			if (McuData.A_AuSz == 1)
			{
				PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x47;

			}
		}
		else
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x40;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x47)
	{
		//
		PX_Out_Spf.XX_DspFlag1.bit.CdAuLdCt = 1;
		if ((PX_In_Spf.NX_McuOpSt == 0x408)
				&& (PX_In_Spf.XX_McuFlag1.bit.CvOp == 1))
		{
			if (PX_In_Spf.XX_McuFlag1.bit.CdAuLdCt == 1)
			{
				PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x48;
			}
		}
		else
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x40;

	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x48)
	{

		//
		McuData.B_EnU3PhCl = TRUE;
		if (PX_In_Spf.NX_McuOpSt == 0x40A)
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x50;
		}
		else
		{
		}
	}
	//--------------------------------------------------------
	//
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x50)
	{
		if ((PX_In_Spf.NX_McuOpSt == 0x40A)
				&& (PX_In_Spf.XX_McuFlag1.bit.CvOp == 0))
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x51;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x51)
	{
		//
		PX_Out_Spf.XX_DspFlag1.bit.CdAuLdCt = 0;
		if (PX_In_Spf.XX_McuFlag1.bit.CdAuLdCt == 0)
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x52;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x52)
	{
		McuData.B_EnU3PhCl = FALSE;	//
		McuData.C_FRmp = FALSE;	//
		PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x53;
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x53)
	{
		if (McuData.A_FMin == 1)
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x54;
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x54)
	{

		PX_Out_Spf.SX_Run = 0;
		PX_Out_Spf.XX_DspFlag1.bit.CvOp = 0;
		PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x55;
	}
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x55)
	{
		if (PX_In_Spf.NX_McuOpSt == 0x406)
		{
			PX_Out_Spf.NX_DspOpSt.bit.CvSt = 0x20;
		}
	}
	//------------------------------------------------
	//
	else if (PX_Out_Spf.oldDspSt.bit.CvSt == 0x60)
	{
		PX_Out_Spf.XX_DspFlag1.all = 0x0000;
		PX_Out_Spf.SX_Run = 0;
	}

	//---------------------------------------
	//OvpCp
	/**/
	if (PX_Out_Spf.oldDspSt.bit.OvpCp == 0x0)
	{
		if ((PX_In_Spf.NX_McuOpSt == 0x405)
				&& (PX_In_Spf.XX_McuFlag1.bit.OvpFcTsAv == 1)
				&& (PX_Out_Spf.oldDspSt.bit.CvSt == 0x30))
			PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x1;
	}
	else if (PX_Out_Spf.oldDspSt.bit.OvpCp == 0x1)
	{
		//
		PX_Out_Spf.XX_DspFlag1.bit.OvpFcTs = 0;	//
		PX_Out_Spf.XX_DspFlag1.bit.OvpCpAv = 1;
		if (1)
		{
			PX_Out_Spf.XX_DspFlag1.bit.OvpFcTs = 1;
			PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x2;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.OvpCp == 0x2)
	{
		//
		PX_Out_Spf.XX_DspFlag1.bit.OvpCpAv = 0;

		if (PX_In_Spf.NX_McuOpSt == 0x404)
		{
			PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x0;
			PX_Out_Spf.XX_DspFlag1.bit.OvpFcTs = 0;
		}
		if ((PX_In_Spf.NX_McuOpSt == 0x40C) || (PX_In_Spf.NX_McuOpSt == 0x417)
				|| (PX_In_Spf.NX_McuOpSt == 0x41C)
				|| (PX_In_Spf.NX_McuOpSt == 0x41A)
				|| (PX_In_Spf.NX_McuOpSt == 0x426)
				|| (PX_In_Spf.NX_McuOpSt == 0x41E))
		{
			PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x4;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.OvpCp == 0x4)
	{
		//
		//		DspData.C_OvpCpOp = 1;
		PX_Out_Spf.XX_DspFlag1.bit.OvpCpAv = 1;
		if ((PX_In_Spf.NX_McuOpSt == 0x40C) || (PX_In_Spf.NX_McuOpSt == 0x417)
				|| (PX_In_Spf.NX_McuOpSt == 0x41C)
				|| (PX_In_Spf.NX_McuOpSt == 0x41A)
				|| (PX_In_Spf.NX_McuOpSt == 0x426)
				|| (PX_In_Spf.NX_McuOpSt == 0x41E))
		{
			//			if (PX_In_Spf.XU_DcLk < 36.0) {
			//				PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x6;
			//			}
			PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x6;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.OvpCp == 0x6)
	{
		//
		//		DspData.C_OvpCpOp = 0;
		PX_Out_Spf.XX_DspFlag1.bit.OvpCpAv = 0;
		if ((PX_In_Spf.NX_McuOpSt == 0x404))
		{
			PX_Out_Spf.NX_DspOpSt.bit.OvpCp = 0x0;
		}
	}
	//---------------------------------------------

	//-------------
	//BtCp
	if (PX_Out_Spf.oldDspSt.bit.BtCp == 0x0)
	{
		PX_Out_Spf.NX_DspOpSt.bit.BtCp = 0x4;
	}
	else if (PX_Out_Spf.oldDspSt.bit.BtCp == 0x2)
	{
		//		DspData.C_BtCpOp = 1;
		PX_Out_Spf.XX_DspFlag1.bit.BtCpAv = 0x1;
		if (PX_In_Spf.XX_McuFlag1.bit.BtCpAv == 0)
		{
			PX_Out_Spf.NX_DspOpSt.bit.BtCp = 0x4;
		}
	}
	else if (PX_Out_Spf.oldDspSt.bit.BtCp == 0x4)
	{
		//		DspData.C_BtCpOp = 0;
		PX_Out_Spf.XX_DspFlag1.bit.BtCpAv = 0x0;
		if (PX_In_Spf.XX_McuFlag1.bit.BtCpAv == 1)
		{
			PX_Out_Spf.NX_DspOpSt.bit.BtCp = 0x2;
		}
	}

	PX_Out_Spf.oldDspSt.all = PX_Out_Spf.NX_DspOpSt.all;
	//------------------------------------------
}

//===============================================================================================
//=========================================THE END===============================================
//===============================================================================================
