/* ==============================================================================
System Name:  	coilgun

File Name:	  	coilgun.c

Description:	Primary system file for the Real Implementation of Sensorless  
          		Field Orientation Control for Three Phase Permanent-Magnet
          		Synchronous Motor(s) (PMSM) 

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DRV8412-EVM kit. 
=====================================================================================
 History: 04-9-2010	Version 1.1: Support F2803x 
=================================================================================  */

// Include header files used in the main function

#include "PeripheralHeaderIncludes.h"
#include "coilgun-Settings.h"
#include "IQmathLib.h"
#include "coilgun.h"
#include <math.h>

union DRV8301_STATUS_REG_1 DRV8301_stat_reg1;
union DRV8301_STATUS_REG_2 DRV8301_stat_reg2;
union DRV8301_CONTROL_REG_1 DRV8301_cntrl_reg1;
union DRV8301_CONTROL_REG_2 DRV8301_cntrl_reg2;
Uint16 read_drv_status = 0;

// Prototype statements for functions found within this file.
interrupt void MainISR(void);
void DeviceInit();
void MemCopy();
void InitFlash();
// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
void C0(void);	//state C0

// A branch states
void A1(void);	//state A1
void A2(void);	//state A2
void A3(void);	//state A3

// B branch states
void B1(void);	//state B1
void B2(void);	//state B2
void B3(void);	//state B3

// C branch states
void C1(void);	//state C1
void C2(void);	//state C2
void C3(void);	//state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);	// Base States pointer
void (*A_Task_Ptr)(void);		// State pointer A branch
void (*B_Task_Ptr)(void);		// State pointer B branch
void (*C_Task_Ptr)(void);		// State pointer C branch

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

int16	VTimer0[4];			// Virtual Timers slaved off CPU Timer 0 (A events)
int16	VTimer1[4]; 		// Virtual Timers slaved off CPU Timer 1 (B events)
int16	VTimer2[4]; 		// Virtual Timers slaved off CPU Timer 2 (C events)
int16	SerialCommsTimer;

// Global variables used in this system

_iq VdTesting = _IQ(0.0);			// Vd reference (pu) 
_iq VqTesting = _IQ(0.2);			// Vq reference (pu) 

_iq IdRef = _IQ(0.0);				// Id reference (pu) 
_iq IqRef = _IQ(0.1);				// Iq reference (pu) 
_iq SpeedRef = _IQ(0.25);			// Speed reference (pu)	

_iq cal_offset_A = _IQ15(0.4890);		//F28069	 
_iq cal_offset_B = _IQ15(0.4877);		//F28069
//_iq cal_offset_A = _IQ15(0.4990);		//F28035	 
//_iq cal_offset_B = _IQ15(0.5034);		//F28035
//_iq cal_offset_A = _IQ15(0.5);	 
//_iq cal_offset_B = _IQ15(0.5);
_iq cal_filt_gain;	 

float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h 

Uint32 IsrTicker = 0;
Uint16 BackTicker = 0;
Uint16 lsw=0;

int16 PwmDacCh1=0;
int16 PwmDacCh2=0;
int16 PwmDacCh3=0;
int16 PwmDacCh4=0;

int16 DlogCh1 = 0;
int16 DlogCh2 = 0;
int16 DlogCh3 = 0;
int16 DlogCh4 = 0;


#if (BUILDLEVEL==LEVEL1)	 
Uint16 DRV_RESET = 1;
#else
Uint16 DRV_RESET = 0;
#endif

volatile Uint16 EnableFlag = FALSE;
Uint16 LockRotorFlag = FALSE;
Uint16 RunMotor = FALSE;

Uint16 SpeedLoopPrescaler = 10;      // Speed loop prescaler
Uint16 SpeedLoopCount = 1;           // Speed loop counter

// Instance a PWM driver instance
PWMGEN pwm1 = PWMGEN_DEFAULTS;

// Instance a PWM DAC driver instance
PWMDAC pwmdac1 = PWMDAC_DEFAULTS;

// Instance a Space Vector PWM modulator.
SVGENMF svgen_mf1 = SVGENMF_DEFAULTS;

// Instance a V/Hz Scalar
VHZPROF vhz1 = VHZPROF_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;

// Create an instance of DATALOG Module
DLOG_4CH dlog = DLOG_4CH_DEFAULTS;      




void main(void)
{
	
	DeviceInit();	// Device Life support & GPIO		

// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler
// (see TwoChannelBuck.pjt file)
#ifdef FLASH
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files. 
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
#endif //(FLASH)

   // Waiting for enable flag set
   while (EnableFlag==FALSE) 
    { 
      BackTicker++;
    }

// Timing sync for slow background tasks 
// Timer period definitions found in device specific PeripheralHeaderIncludes.h
	CpuTimer0Regs.PRD.all =  mSec1;		// A tasks
	CpuTimer1Regs.PRD.all =  mSec5;		// B tasks
	CpuTimer2Regs.PRD.all =  mSec50;	// C tasks

// Tasks State-machine init
	Alpha_State_Ptr = &A0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;
	C_Task_Ptr = &C1;

// Initialize PWM module
    pwm1.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2;  // Prescaler X1 (T1), ISR period = T x 1 
    pwm1.HalfPerMax=pwm1.PeriodMax/2;
	PWM_INIT_MACRO(pwm1)  

// Initialize PWMDAC module
    pwmdac1.PeriodMax=500;          // @60Mhz, 1500->20kHz, 1000-> 30kHz, 500->60kHz
    pwmdac1.HalfPerMax=pwmdac1.PeriodMax/2;

	PWMDAC_INIT_MACRO(pwmdac1)

// Initialize DATALOG module
    dlog.iptr1 = &DlogCh1;
    dlog.iptr2 = &DlogCh2;
	dlog.iptr3 = &DlogCh3;
    dlog.iptr4 = &DlogCh4;
    dlog.trig_value = 0x1;
    dlog.size = 0x00c8;
    dlog.prescalar = 5;
    dlog.init(&dlog);

// Initialize the VHZ_PROF module
     vhz1.LowFreq = _IQ(0.2);
     vhz1.HighFreq = _IQ(0.5);
     vhz1.FreqMax = _IQ(1);
     vhz1.VoltMax = _IQ(0.95);
     vhz1.VoltMin = _IQ(0.2);

// Initialize the SVGEN_MF module
    svgen_mf1.FreqMax = _IQ(6*BASE_FREQ*T);

    // Initialize ADC module
    ADC_MACRO()

// Initialize the phase current offset calibration filter
	cal_filt_gain = _IQ15(T/(T+TC_CAL));
	
// Initialize SPI for communication to the DRV8301
	DRV8301_SPI_Init(&SpiaRegs);
	
// Reassign ISRs. 

	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.EPWM1_INT = &MainISR;
	EDIS;

// Enable PIE group 3 interrupt 1 for EPWM1_INT
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

// Enable CNT_zero interrupt using EPWM1 Time-base
    EPwm1Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation 
    EPwm1Regs.ETSEL.bit.INTSEL = 1;  // Enable interrupt CNT_zero event
    EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event
	EPwm1Regs.ETCLR.bit.INT = 1;     // Enable more interrupts

// Enable CPU INT3 for EPWM1_INT:
	IER |= M_INT3;
// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

// IDLE loop. Just sit and loop forever:	
	for(;;)  //infinite loop
	{
		// State machine entry & exit point
		//===========================================================
		(*Alpha_State_Ptr)();	// jump to an Alpha state (A0,B0,...)
		//===========================================================

		//read the status registers from the DRV8301
		if(read_drv_status)
		{
			if(GpioDataRegs.GPADAT.bit.GPIO28 == 0)
			{
				DRV8301_stat_reg1.all = DRV8301_SPI_Read(&SpiaRegs,STAT_REG_1_ADDR);
				DRV8301_stat_reg2.all = DRV8301_SPI_Read(&SpiaRegs,STAT_REG_2_ADDR);
				read_drv_status = 0;
			}
		}
	}
} //END MAIN CODE



//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{
	// loop rate synchronizer for A-tasks
	if(CpuTimer0Regs.TCR.bit.TIF == 1)
	{
		CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*A_Task_Ptr)();		// jump to an A Task (A1,A2,A3,...)
		//-----------------------------------------------------------

		VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
		SerialCommsTimer++;
	}

	Alpha_State_Ptr = &B0;		// Comment out to allow only A tasks
}

void B0(void)
{
	// loop rate synchronizer for B-tasks
	if(CpuTimer1Regs.TCR.bit.TIF == 1)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// jump to a B Task (B1,B2,B3,...)
		//-----------------------------------------------------------
		VTimer1[0]++;			// virtual timer 1, instance 0 (spare)
	}

	Alpha_State_Ptr = &C0;		// Allow C state tasks
}

void C0(void)
{
	// loop rate synchronizer for C-tasks
	if(CpuTimer2Regs.TCR.bit.TIF == 1)
	{
		CpuTimer2Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*C_Task_Ptr)();		// jump to a C Task (C1,C2,C3,...)
		//-----------------------------------------------------------
		VTimer2[0]++;			//virtual timer 2, instance 0 (spare)
	}

	Alpha_State_Ptr = &A0;	// Back to State A0
}


//=================================================================================
//	A - TASKS (executed in every 1 msec)
//=================================================================================
//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{
	if (EnableFlag == FALSE)
	{
		//de-assert the DRV830x EN_GATE pin
		#ifdef DSP2803x_DEVICE_H
		GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;
		#endif
		#ifdef F2806x_DEVICE_H
		GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;
		#endif

		RunMotor = FALSE;
		
		EALLOW;
	 	EPwm1Regs.TZFRC.bit.OST=1;
		EPwm2Regs.TZFRC.bit.OST=1;
		EPwm3Regs.TZFRC.bit.OST=1;
	 	EDIS;
	}
	else if((EnableFlag == TRUE) && (RunMotor == FALSE))
	{

		//if we want the power stage active we need to enable the DRV830x
		//and configure it.
		if(DRV_RESET == 0)
		{
			//assert the DRV830x EN_GATE pin
			#ifdef DSP2803x_DEVICE_H
			GpioDataRegs.GPBSET.bit.GPIO39 = 1;
			#endif
			#ifdef F2806x_DEVICE_H
			GpioDataRegs.GPBSET.bit.GPIO50 = 1;
			#endif

			DELAY_US(50000);		//delay to allow DRV830x supplies to ramp up
			
			DRV8301_cntrl_reg1.bit.GATE_CURRENT = 0;		// full current 1.7A
//			DRV8301_cntrl_reg1.bit.GATE_CURRENT = 1;		// med current 0.7A
//			DRV8301_cntrl_reg1.bit.GATE_CURRENT = 2;		// min current 0.25A
			DRV8301_cntrl_reg1.bit.GATE_RESET = 0;			// Normal Mode
			DRV8301_cntrl_reg1.bit.PWM_MODE = 0;			// six independant PWMs
//			DRV8301_cntrl_reg1.bit.OC_MODE = 0;				// current limiting when OC detected
			DRV8301_cntrl_reg1.bit.OC_MODE = 1;				// latched OC shutdown
//			DRV8301_cntrl_reg1.bit.OC_MODE = 2;				// Report on OCTWn pin and SPI reg only, no shut-down
//			DRV8301_cntrl_reg1.bit.OC_MODE = 3;				// OC protection disabled
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 0;			// OC @ Vds=0.060V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 4;			// OC @ Vds=0.097V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 6;			// OC @ Vds=0.123V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 9;			// OC @ Vds=0.175V
			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 15;			// OC @ Vds=0.358V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 16;			// OC @ Vds=0.403V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 17;			// OC @ Vds=0.454V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 18;			// OC @ Vds=0.511V
			DRV8301_cntrl_reg1.bit.Reserved = 0;
			
//			DRV8301_cntrl_reg2.bit.OCTW_SET = 0;			// report OT and OC
			DRV8301_cntrl_reg2.bit.OCTW_SET = 1;			// report OT only
#if DRV_GAIN == 10
			DRV8301_cntrl_reg2.bit.GAIN = 0;				// CS amplifier gain = 10
#elif DRV_GAIN == 20
			DRV8301_cntrl_reg2.bit.GAIN = 1;				// CS amplifier gain = 20
#elif DRV_GAIN == 40
			DRV8301_cntrl_reg2.bit.GAIN = 2;				// CS amplifier gain = 40
#elif DRV_GAIN == 80
			DRV8301_cntrl_reg2.bit.GAIN = 3;				// CS amplifier gain = 80
#endif
			DRV8301_cntrl_reg2.bit.DC_CAL_CH1 = 0;			// not in CS calibrate mode
			DRV8301_cntrl_reg2.bit.DC_CAL_CH2 = 0;			// not in CS calibrate mode
			DRV8301_cntrl_reg2.bit.OC_TOFF = 0;				// normal mode
			DRV8301_cntrl_reg2.bit.Reserved = 0;
			
			//write to DRV8301 control register 1, returns status register 1 
			DRV8301_stat_reg1.all = DRV8301_SPI_Write(&SpiaRegs,CNTRL_REG_1_ADDR,DRV8301_cntrl_reg1.all);
			//write to DRV8301 control register 2, returns status register 1
			DRV8301_stat_reg1.all = DRV8301_SPI_Write(&SpiaRegs,CNTRL_REG_2_ADDR,DRV8301_cntrl_reg2.all);

		}

		rc1.TargetValue=0;
		rc1.SetpointValue=0;

		lsw=0; 

		RunMotor = TRUE;
			
		EALLOW;
		EPwm1Regs.TZCLR.bit.OST=1;
		EPwm2Regs.TZCLR.bit.OST=1;
		EPwm3Regs.TZCLR.bit.OST=1;
		EDIS;

		
	}
	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A2
	A_Task_Ptr = &A2;
	//-------------------
}

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{	

	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A3
	A_Task_Ptr = &A3;
	//-------------------
}

//-----------------------------------------
void A3(void) // SPARE (not used)
//-----------------------------------------
{

	//-----------------
	//the next time CpuTimer0 'counter' reaches Period value go to A1
	A_Task_Ptr = &A1;
	//-----------------
}



//=================================================================================
//	B - TASKS (executed in every 5 msec)
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) // Toggle GPIO-00
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B2
	B_Task_Ptr = &B2;	
	//-----------------
}

//----------------------------------------
void B2(void) //  SPARE
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B3
	B_Task_Ptr = &B3;
	//-----------------
}

//----------------------------------------
void B3(void) //  SPARE
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B1
	B_Task_Ptr = &B1;	
	//-----------------
}


//=================================================================================
//	C - TASKS (executed in every 50 msec)
//=================================================================================

//--------------------------------- USER ------------------------------------------

//----------------------------------------
void C1(void) 	// Toggle GPIO-34 
//----------------------------------------
{

	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;	// Blink LED
	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C2
	C_Task_Ptr = &C2;	
	//-----------------

}

//----------------------------------------
void C2(void) //  SPARE
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C3
	C_Task_Ptr = &C3;	
	//-----------------
}


//-----------------------------------------
void C3(void) //  SPARE
//-----------------------------------------
{

	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C1
	C_Task_Ptr = &C1;	
	//-----------------
}


//_iq StepAngle,EntryOld,dx,dy;
//_iq TP = (_IQ(1.0));
//
//#define PI_THIRD    _IQ(1.04719755119660)    /* This is 60 degree */
//
//void SVGENMF_MACRO(SVGENMF *v)
//{
//    /* Normalise the freq input to appropriate step angle   */
//        /* Here, 1 pu. = 60 degree          */
//        StepAngle = _IQmpy(v->Freq,v->FreqMax);
//    /* Calculate new angle alpha            */
//        EntryOld = v->NewEntry;
//        v->Alpha = v->Alpha + StepAngle;
//    if (v->Alpha >= _IQ(1.0))
//        v->Alpha = v->Alpha-_IQ(1.0);
//    v->NewEntry = v->Alpha;
//        dy = _IQsin(_IQmpy(v->NewEntry,PI_THIRD));              /* dy = sin(NewEntry)    */
//        dx = _IQsin(PI_THIRD-_IQmpy(v->NewEntry,PI_THIRD));     /* dx = sin(60-NewEntry) */
//    /* Determine which sector           */
//        if (v->NewEntry-EntryOld<0)
//        {
//            if (v->SectorPointer==5)
//                v->SectorPointer = 0;
//            else
//                v->SectorPointer = v->SectorPointer + 1;
//        }
//    if (v->SectorPointer==0)  /* Sector 1 calculations - a,b,c -. a,b,c*/
//        {
//        v->Ta = (TP-dx-dy)>>1;
//        v->Tb = v->Ta + dx;
//        v->Tc = TP - v->Ta;
//        }
//        else if (v->SectorPointer==1)  /* Sector 2 calculations - a,b,c -. b,a,c  &  dx <-. dy   */
//        {
//        v->Tb = (TP-dx-dy)>>1;
//        v->Ta = v->Tb + dy;
//        v->Tc = TP - v->Tb;
//        }
//        else if (v->SectorPointer==2)  /* Sector 3 calculations - a,b,c -. b,c,a     */
//        {
//        v->Tb = (TP-dx-dy)>>1;
//        v->Tc = v->Tb + dx;
//        v->Ta = TP - v->Tb;
//        }
//        else if (v->SectorPointer==3)  /* Sector 4 calculations - a,b,c -. c,b,a  &  dx <-. dy   */
//        {
//        v->Tc = (TP-dx-dy)>>1;
//        v->Tb = v->Tc + dy;
//        v->Ta = TP - v->Tc;
//        }
//        else if (v->SectorPointer==4)  /* Sector 5 calculations - a,b,c -. c,a,b     */
//        {
//        v->Tc = (TP-dx-dy)>>1;
//        v->Ta = v->Tc + dx;
//        v->Tb = TP - v->Tc;
//        }
//        else if (v->SectorPointer==5)  /* Sector 6 calculations - a,b,c -. a,c,b  &  dx <-. dy   */
//        {
//        v->Ta = (TP-dx-dy)>>1;
//        v->Tc = v->Ta + dy;
//        v->Tb = TP - v->Ta;
//        }
///* Convert the unsigned GLOBAL_Q format (ranged (0,1)) . signed GLOBAL_Q format (ranged (-1,1)) */
///* Then, multiply with a gain and add an offset.                        */
//        v->Ta = (v->Ta-_IQ(0.5))<<1;
//        v->Ta = _IQmpy(v->Gain,v->Ta) + v->Offset;
//
//        v->Tb = (v->Tb-_IQ(0.5))<<1;
//        v->Tb = _IQmpy(v->Gain,v->Tb) + v->Offset;
//
//        v->Tc = (v->Tc-_IQ(0.5))<<1;
//        v->Tc = _IQmpy(v->Gain,v->Tc) + v->Offset;
//        }

// MainISR 
interrupt void MainISR(void)
{

// Verifying the ISR
    IsrTicker++;

if(RunMotor)
	{
// =============================== LEVEL 1 ======================================
//	  Checks target independent modules, duty cycle waveforms and PWM update
//	  Keep the motors disconnected at this level	
// ============================================================================== 

#if (BUILDLEVEL==LEVEL1)	 

    // ------------------------------------------------------------------------------
    //    Connect inputs of the VOLT_PROF module and call the volt profile macro
    // ------------------------------------------------------------------------------
        vhz1.Freq = SpeedRef;
        VHZ_PROF_MACRO(vhz1);

    // ------------------------------------------------------------------------------
    //    Connect inputs of the SVGEN_MF module and call the space-vector gen. macro
    // ------------------------------------------------------------------------------
        svgen_mf1.Gain = vhz1.VoltOut;
        svgen_mf1.Freq = vhz1.Freq;
        SVGENMF_MACRO(svgen_mf1)

    // ------------------------------------------------------------------------------
    //  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
    // ------------------------------------------------------------------------------
        pwm1.MfuncC1 = svgen_mf1.Ta;
        pwm1.MfuncC2 = svgen_mf1.Tb;
        pwm1.MfuncC3 = svgen_mf1.Tc;
        PWM_MACRO(pwm1)                           // Calculate the new PWM compare values

    // ------------------------------------------------------------------------------
    //    Connect inputs of the PWMDAC module
    // ------------------------------------------------------------------------------
        pwmdac1.MfuncC1 = svgen_mf1.Ta;
        pwmdac1.MfuncC2 = svgen_mf1.Tb;
        PWMDAC_MACRO(7,pwmdac1)                         // PWMDAC 6A, 6B

        pwmdac1.MfuncC1 = svgen_mf1.Tc;
        pwmdac1.MfuncC2 = svgen_mf1.Tb-svgen_mf1.Tc;
        PWMDAC_MACRO(8,pwmdac1)                         // PWMDAC 7A, 7B

    // ------------------------------------------------------------------------------
    //    Connect inputs of the DATALOG module
    // ------------------------------------------------------------------------------
        DlogCh1 = (int16)_IQtoIQ15(svgen_mf1.Ta);
        DlogCh2 = (int16)_IQtoIQ15(svgen_mf1.Tb);
        DlogCh3 = (int16)_IQtoIQ15(svgen_mf1.Tc);
        DlogCh4 = (int16)_IQtoIQ15(svgen_mf1.Tb-svgen_mf1.Tc);

#endif // (BUILDLEVEL==LEVEL1)

// =============================== LEVEL 2 ======================================
//	  Level 2 verifies the analog-to-digital conversion, offset compensation, 
//    clarke/park transformations (CLARKE/PARK), phase voltage calculations 
// ============================================================================== 

#if (BUILDLEVEL==LEVEL2) 

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;		
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1) 

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	#ifdef F2806x_DEVICE_H
//	clarke1.As=-(((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2); // Phase A curr.
//	clarke1.Bs=-(((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2); // Phase B curr.
	clarke1.As=(((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2); // Phase A curr.
	clarke1.Bs=(((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2); // Phase B curr.
	#endif													 	   // ((ADCmeas(q12)/2^12)-0.5)*2			

	#ifdef DSP2803x_DEVICE_H
//    clarke1.As=-(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
//    clarke1.Bs=-(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    clarke1.As=(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    clarke1.Bs=(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    #endif

    CLARKE_MACRO(clarke1) 

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;
    park1.Angle = rg1.Out;
    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
    PARK_MACRO(park1) 
 
// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;
    ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
    IPARK_MACRO(ipark1) 

// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
	#ifdef F2806x_DEVICE_H
	volt1.DcBusVolt = ((AdcResult.ADCRESULT2)*0.00024414); // DC Bus voltage meas.
    #endif														 // (ADCmeas(q12)/2^12)	
    
    #ifdef DSP2803x_DEVICE_H
	volt1.DcBusVolt = _IQ15toIQ((AdcResult.ADCRESULT3<<3));	     // DC Bus voltage meas.
    #endif

    volt1.MfuncV1 = svgen_dq1.Ta;
    volt1.MfuncV2 = svgen_dq1.Tb;
    volt1.MfuncV3 = svgen_dq1.Tc;
    VOLT_MACRO(volt1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen_dq1.Ualpha = ipark1.Alpha;
    svgen_dq1.Ubeta = ipark1.Beta;
    SVGEN_MACRO(svgen_dq1)	

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(svgen_dq1.Ta);
    pwm1.MfuncC2 = _IQtoQ15(svgen_dq1.Tb);  
    pwm1.MfuncC3 = _IQtoQ15(svgen_dq1.Tc);
    PWM_MACRO(pwm1)							   	   // Calculate the new PWM compare values	


	EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;	// PWM 1A - PhaseA 
	EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;	// PWM 2A - PhaseB
	EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;	// PWM 3A - PhaseC
 
// ------------------------------------------------------------------------------
//  Connect inputs of the PWCCMDAC module 
// ------------------------------------------------------------------------------	
    PwmDacCh1 = _IQtoQ15(volt1.Valpha); 
    PwmDacCh2 = _IQtoQ15(clarke1.Alpha);
    PwmDacCh3 = _IQtoQ15(volt1.Vbeta );
    PwmDacCh4 = _IQtoQ15(clarke1.Beta); 

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(volt1.Valpha);
    DlogCh2 = _IQtoQ15(clarke1.Alpha);
    DlogCh3 = _IQtoQ15(volt1.Vbeta );
    DlogCh4 = _IQtoQ15(clarke1.Beta);  


#endif // (BUILDLEVEL==LEVEL2)

// =============================== LEVEL 3 ======================================
//	  Level 3 auto-calculates the current sensor offset calibration 
// ============================================================================== 

#if (BUILDLEVEL==LEVEL3) 

_iq IAfdbk;
_iq IBfdbk;
// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
// ------------------------------------------------------------------------------
	#ifdef F2806x_DEVICE_H
	IAfdbk=((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2; // Phase A curr.
	IBfdbk=((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2; // Phase B curr.
	#endif													 	   // ((ADCmeas(q12)/2^12)-0.5)*2			

	#ifdef DSP2803x_DEVICE_H
    IAfdbk=_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1;
    IBfdbk=_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1;
    #endif
    
// ------------------------------------------------------------------------------
//  LPF to average the calibration offsets
//  Use the offsets calculated here to initialize cal_offset_A and cal_offset_B
//  so that they are used for the remaining build levels
// ------------------------------------------------------------------------------
    cal_offset_A = _IQ15mpy(cal_filt_gain,_IQtoIQ15(IAfdbk)) + cal_offset_A;
    cal_offset_B = _IQ15mpy(cal_filt_gain,_IQtoIQ15(IBfdbk)) + cal_offset_B;

// ------------------------------------------------------------------------------
//  force all PWMs to 0% duty cycle
// ------------------------------------------------------------------------------
	EPwm1Regs.CMPA.half.CMPA=pwm1.PeriodMax;	// PWM 1A - PhaseA 
	EPwm2Regs.CMPA.half.CMPA=pwm1.PeriodMax;	// PWM 2A - PhaseB
	EPwm3Regs.CMPA.half.CMPA=pwm1.PeriodMax;	// PWM 3A - PhaseC  

// ------------------------------------------------------------------------------
//  Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------	
    PwmDacCh1 = _IQtoQ15(IAfdbk);
    PwmDacCh2 = _IQtoQ15(IBfdbk);
    PwmDacCh2 = _IQtoQ15(cal_offset_A); 
    PwmDacCh3 = _IQtoQ15(cal_offset_B); 

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(IAfdbk);
    DlogCh2 = _IQtoQ15(IBfdbk);
    DlogCh3 = _IQtoQ15(cal_offset_A);
    DlogCh4 = _IQtoQ15(cal_offset_B);  
#endif // (BUILDLEVEL==LEVEL3)

// =============================== LEVEL 4 ======================================
//	Level 4 verifies the dq-axis current regulation performed by PID and speed 
//	measurement modules  
// ==============================================================================  
//  lsw=0: lock the rotor of the motor 
//  lsw=1: close the current loop


#if (BUILDLEVEL==LEVEL4)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------ 
    if(lsw==0) rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;		
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)  

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------  
	#ifdef F2806x_DEVICE_H
//	clarke1.As=-(((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2); // Phase A curr.
//	clarke1.Bs=-(((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2); // Phase B curr.
	clarke1.As=(((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2); // Phase A curr.
	clarke1.Bs=(((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2); // Phase B curr.
	#endif													 	   // ((ADCmeas(q12)/2^12)-0.5)*2			

	#ifdef DSP2803x_DEVICE_H
//    clarke1.As=-(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
//    clarke1.Bs=-(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    clarke1.As=(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    clarke1.Bs=(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    #endif
  
    CLARKE_MACRO(clarke1) 

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------ 
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;
    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
	
    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
	
    PARK_MACRO(park1) 
 
// ------------------------------------------------------------------------------
//  Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID IQ controller macro
// ------------------------------------------------------------------------------  
    if(lsw==0) pid1_iq.term.Ref = 0;
    else if(lsw==1) pid1_iq.term.Ref = IqRef;
    pid1_iq.term.Fbk = park1.Qs;
    PID_GR_MACRO(pid1_iq)

// ------------------------------------------------------------------------------
//  Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID ID controller macro
// ------------------------------------------------------------------------------   
    if(lsw==0) pid1_id.term.Ref = _IQ(0.05);
    else pid1_id.term.Ref = IdRef; 
    pid1_id.term.Fbk = park1.Ds;
    PID_GR_MACRO(pid1_id) 

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = pid1_id.term.Out;
    ipark1.Qs = pid1_iq.term.Out ;
    ipark1.Sine   = park1.Sine;
    ipark1.Cosine = park1.Cosine;
    IPARK_MACRO(ipark1) 

// ------------------------------------------------------------------------------
//    Call the QEP calculation module 
// ------------------------------------------------------------------------------
    QEP_MACRO(qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro 
// ------------------------------------------------------------------------------ 
    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
	#ifdef F2806x_DEVICE_H
	volt1.DcBusVolt = ((AdcResult.ADCRESULT2)*0.00024414); // DC Bus voltage meas.
    #endif														 // (ADCmeas(q12)/2^12)	
    
    #ifdef DSP2803x_DEVICE_H
	volt1.DcBusVolt = _IQ15toIQ((AdcResult.ADCRESULT3<<3));	     // DC Bus voltage meas.
    #endif

    volt1.MfuncV1 = svgen_dq1.Ta;
    volt1.MfuncV2 = svgen_dq1.Tb;
    volt1.MfuncV3 = svgen_dq1.Tc;
    VOLT_MACRO(volt1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen_dq1.Ualpha = ipark1.Alpha;
    svgen_dq1.Ubeta = ipark1.Beta;
    SVGEN_MACRO(svgen_dq1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(svgen_dq1.Ta);
    pwm1.MfuncC2 = _IQtoQ15(svgen_dq1.Tb);  
    pwm1.MfuncC3 = _IQtoQ15(svgen_dq1.Tc);
    PWM_MACRO(pwm1)							   	   // Calculate the new PWM compare values	


	EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;	// PWM 1A - PhaseA 
	EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;	// PWM 2A - PhaseB
	EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;	// PWM 3A - PhaseC  

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------	
    PwmDacCh1 = _IQtoQ15(rg1.Out);
    PwmDacCh2 = _IQtoQ15(speed1.ElecTheta);    
    PwmDacCh3 = _IQtoQ15(clarke1.As);    
    PwmDacCh4 = _IQtoQ15(clarke1.Bs);  

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(rg1.Out);
    DlogCh2 = _IQtoQ15(svgen_dq1.Ta);
    DlogCh3 = _IQtoQ15(clarke1.As);
    DlogCh4 = _IQtoQ15(clarke1.Bs);  

#endif // (BUILDLEVEL==LEVEL4)


// =============================== LEVEL 5 ======================================
//	  Level 5 verifies the estimated rotor position and speed estimation 
//	  performed by SMOPOS and SPEED_EST modules, respectively. 
// ==============================================================================  
//  lsw=0: lock the rotor of the motor 
//  lsw=1: close the current loop

#if (BUILDLEVEL==LEVEL5)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------ 
    if(lsw==0)rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;		
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)  

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	#ifdef F2806x_DEVICE_H
//	clarke1.As=-(((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2); // Phase A curr.
//	clarke1.Bs=-(((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2); // Phase B curr.
	clarke1.As=(((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2); // Phase A curr.
	clarke1.Bs=(((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2); // Phase B curr.
	#endif													 	   // ((ADCmeas(q12)/2^12)-0.5)*2			

	#ifdef DSP2803x_DEVICE_H
//    clarke1.As=-(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
//    clarke1.Bs=-(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    clarke1.As=(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    clarke1.Bs=(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    #endif

    CLARKE_MACRO(clarke1) 

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------ 
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;
    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
	
    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
	
    PARK_MACRO(park1) 
 
// ------------------------------------------------------------------------------
//  Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID IQ controller macro
// ------------------------------------------------------------------------------  
    if(lsw==0) pid1_iq.term.Ref = 0;
    else if(lsw==1) pid1_iq.term.Ref = IqRef;
    pid1_iq.term.Fbk = park1.Qs;
    PID_GR_MACRO(pid1_iq)

// ------------------------------------------------------------------------------
//  Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID ID controller macro
// ------------------------------------------------------------------------------   
    if(lsw==0) pid1_id.term.Ref = _IQ(0.05);
    else pid1_id.term.Ref = 0; 
    pid1_id.term.Fbk = park1.Ds;
    PID_GR_MACRO(pid1_id)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------ 
    ipark1.Ds = pid1_id.term.Out;
    ipark1.Qs = pid1_iq.term.Out;
    ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
    IPARK_MACRO(ipark1) 

// ------------------------------------------------------------------------------
//    Call the QEP calculation module 
// ------------------------------------------------------------------------------
    QEP_MACRO(qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro 
// ------------------------------------------------------------------------------
    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------ 
	#ifdef F2806x_DEVICE_H
	volt1.DcBusVolt = ((AdcResult.ADCRESULT2)*0.00024414); // DC Bus voltage meas.
    #endif														 // (ADCmeas(q12)/2^12)	
    
    #ifdef DSP2803x_DEVICE_H
	volt1.DcBusVolt = _IQ15toIQ((AdcResult.ADCRESULT3<<3));	     // DC Bus voltage meas.
    #endif

    volt1.MfuncV1 = svgen_dq1.Ta;
    volt1.MfuncV2 = svgen_dq1.Tb;
    volt1.MfuncV3 = svgen_dq1.Tc;        
    VOLT_MACRO(volt1)

// ------------------------------------------------------------------------------
//    Connect inputs of the SMO_POS module and call the sliding-mode observer macro
// ------------------------------------------------------------------------------
    smo1.Ialpha = clarke1.Alpha;
  	smo1.Ibeta  = clarke1.Beta;
    smo1.Valpha = volt1.Valpha;
    smo1.Vbeta  = volt1.Vbeta;
    SMO_MACRO(smo1)

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_EST module and call the estimated speed macro
// ------------------------------------------------------------------------------
    speed3.EstimatedTheta = smo1.Theta;
    SE_MACRO(speed3) 

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
    svgen_dq1.Ualpha = ipark1.Alpha;
    svgen_dq1.Ubeta = ipark1.Beta;
    SVGEN_MACRO(svgen_dq1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(svgen_dq1.Ta);
    pwm1.MfuncC2 = _IQtoQ15(svgen_dq1.Tb);  
    pwm1.MfuncC3 = _IQtoQ15(svgen_dq1.Tc);
    PWM_MACRO(pwm1)							   	   // Calculate the new PWM compare values	
    
	EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;	// PWM 1A - PhaseA 
	EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;	// PWM 2A - PhaseB
	EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;	// PWM 3A - PhaseC  

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------	
    PwmDacCh1 = _IQtoQ15(smo1.Theta); 
    PwmDacCh2 = _IQtoQ15(rg1.Out); 
    PwmDacCh3 = _IQtoQ15(clarke1.As);
    PwmDacCh4 = _IQtoQ15(clarke1.Bs);

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(smo1.Theta);
    DlogCh2 = _IQtoQ15(rg1.Out);
    DlogCh3 = _IQtoQ15(clarke1.As);
    DlogCh4 = _IQtoQ15(clarke1.Bs);  


#endif // (BUILDLEVEL==LEVEL5)


// =============================== LEVEL 6 ======================================
//	  Level 6 verifies the speed regulator performed by PID_GRANDO_CONTROLLER module. 
//	  The system speed loop is closed by using the measured speed as a feedback.
// ==============================================================================  
//  lsw=0: lock the rotor of the motor 
//  lsw=1: close the current loop
//	lsw=2: close the speed/sensorless current loop


#if (BUILDLEVEL==LEVEL6)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------ 
    if(lsw==0)rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;		
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)  

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	#ifdef F2806x_DEVICE_H
//	clarke1.As=-(((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2); // Phase A curr.
//	clarke1.Bs=-(((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2); // Phase B curr.
	clarke1.As=(((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2); // Phase A curr.
	clarke1.Bs=(((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2); // Phase B curr.
	#endif													 	   // ((ADCmeas(q12)/2^12)-0.5)*2			

	#ifdef DSP2803x_DEVICE_H
//    clarke1.As=-(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
//    clarke1.Bs=-(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    clarke1.As=(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    clarke1.Bs=(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    #endif
		
    CLARKE_MACRO(clarke1)  

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------  
    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;
	
    if(lsw==0) park1.Angle = 0;
    else if(lsw==1) park1.Angle = rg1.Out;
    else park1.Angle = smo1.Theta;
	
    park1.Sine = _IQsinPU(park1.Angle);
    park1.Cosine = _IQcosPU(park1.Angle);
	
    PARK_MACRO(park1) 

// ------------------------------------------------------------------------------
//    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID speed controller macro
// ------------------------------------------------------------------------------  
  if (SpeedLoopCount==SpeedLoopPrescaler)
     {
      pid1_spd.term.Ref = rc1.SetpointValue;
      pid1_spd.term.Fbk = speed1.Speed;
      PID_GR_MACRO(pid1_spd)
      SpeedLoopCount=1;
     }
  else SpeedLoopCount++;   

  if(lsw==0 || lsw==1)
  {
  	pid1_spd.data.ui=0;
  	pid1_spd.data.i1=0;
  }

// ------------------------------------------------------------------------------
//    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID IQ controller macro
// ------------------------------------------------------------------------------  
	if(lsw==0) pid1_iq.term.Ref = 0;
    else if(lsw==1) pid1_iq.term.Ref = IqRef;
    else pid1_iq.term.Ref = IqRef; //pid1_spd.term.Out; (pid1_spd.term.Out->Level 6A, IqRef->Level 6B) 
	pid1_iq.term.Fbk = park1.Qs;
	PID_GR_MACRO(pid1_iq)

// ------------------------------------------------------------------------------
//    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID ID controller macro
// ------------------------------------------------------------------------------  
    if(lsw==0) pid1_id.term.Ref = _IQ(0.05);
    else pid1_id.term.Ref = 0; 
    pid1_id.term.Fbk = park1.Ds;
    PID_GR_MACRO(pid1_id)

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------   
    ipark1.Ds = pid1_id.term.Out;
    ipark1.Qs = pid1_iq.term.Out;
    ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
    IPARK_MACRO(ipark1) 

// ------------------------------------------------------------------------------
//    Call the QEP calculation module 
// ------------------------------------------------------------------------------
    QEP_MACRO(qep1);

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro 
// ------------------------------------------------------------------------------
    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//    Connect inputs of the VOLT_CALC module and call the phase voltage macro
// ------------------------------------------------------------------------------
	#ifdef F2806x_DEVICE_H
	volt1.DcBusVolt = ((AdcResult.ADCRESULT2)*0.00024414); // DC Bus voltage meas.
    #endif														 // (ADCmeas(q12)/2^12)	
    
    #ifdef DSP2803x_DEVICE_H
	volt1.DcBusVolt = _IQ15toIQ((AdcResult.ADCRESULT3<<3));	     // DC Bus voltage meas.
    #endif

    volt1.MfuncV1 = svgen_dq1.Ta;
    volt1.MfuncV2 = svgen_dq1.Tb;
    volt1.MfuncV3 = svgen_dq1.Tc;        
    VOLT_MACRO(volt1)

// ------------------------------------------------------------------------------
//    Connect inputs of the SMO_POS module and call the sliding-mode observer macro
// ------------------------------------------------------------------------------
    smo1.Ialpha = clarke1.Alpha;
  	smo1.Ibeta  = clarke1.Beta;
    smo1.Valpha = volt1.Valpha;
    smo1.Vbeta  = volt1.Vbeta;
    SMO_MACRO(smo1)

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_EST module and call the estimated speed macro
// ------------------------------------------------------------------------------
    speed3.EstimatedTheta = smo1.Theta;
    SE_MACRO(speed3)  

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen_dq1.Ualpha = ipark1.Alpha;
    svgen_dq1.Ubeta = ipark1.Beta;
  	SVGEN_MACRO(svgen_dq1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(svgen_dq1.Ta);
    pwm1.MfuncC2 = _IQtoQ15(svgen_dq1.Tb);  
    pwm1.MfuncC3 = _IQtoQ15(svgen_dq1.Tc);
    PWM_MACRO(pwm1)							   	   // Calculate the new PWM compare values	

	EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;	// PWM 1A - PhaseA 
	EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;	// PWM 2A - PhaseB
	EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;	// PWM 3A - PhaseC  


// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------	
    PwmDacCh1 = _IQtoQ15(smo1.Theta); 
    PwmDacCh2 = _IQtoQ15(rg1.Out); 
    PwmDacCh3 = _IQtoQ15(clarke1.As);
    PwmDacCh4 = _IQtoQ15(clarke1.Bs);

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(smo1.Theta);
    DlogCh2 = _IQtoQ15(rg1.Out);
    DlogCh3 = _IQtoQ15(clarke1.As);
    DlogCh4 = _IQtoQ15(clarke1.Bs); 

#endif // (BUILDLEVEL==LEVEL6) 

// =============================== LEVEL 7 ======================================
//	  Level 7 verifies the speed regulator performed by PID_GRANDO_CONTROLLER module. 
//	  The system speed loop is closed by using the estimated speed as a feedback.
// ==============================================================================  
//  lsw=0: lock the rotor of the motor
//  lsw=1: close the current loop 
//  lsw=2: close the speed loop

#if (BUILDLEVEL==LEVEL7)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------ 
    if(lsw==0)rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;		
    RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
    RG_MACRO(rg1)  

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------

	#ifdef F2806x_DEVICE_H
//	clarke1.As=-(((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2); // Phase A curr.
//	clarke1.Bs=-((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2; // Phase B curr.
	clarke1.As=(((AdcResult.ADCRESULT0)*0.00024414-cal_offset_A)*2); // Phase A curr.
	clarke1.Bs=((AdcResult.ADCRESULT1)*0.00024414-cal_offset_B)*2; // Phase B curr.
	#endif													 	   // ((ADCmeas(q12)/2^12)-0.5)*2			

	#ifdef DSP2803x_DEVICE_H
//    clarke1.As=-(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
//    clarke1.Bs=-(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    clarke1.As=(_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    clarke1.Bs=(_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    #endif

	CLARKE_MACRO(clarke1)  

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------  
	park1.Alpha = clarke1.Alpha;
	park1.Beta = clarke1.Beta;
	
	if(lsw==0) park1.Angle = 0;
	else if(lsw==1) park1.Angle = rg1.Out;
	else park1.Angle = smo1.Theta;
	
	park1.Sine = _IQsinPU(park1.Angle);
	park1.Cosine = _IQcosPU(park1.Angle);
	
	PARK_MACRO(park1) 

// ------------------------------------------------------------------------------
//    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID speed controller macro
// ------------------------------------------------------------------------------  
   if (SpeedLoopCount==SpeedLoopPrescaler)
     {
      pid1_spd.term.Ref = rc1.SetpointValue;
      pid1_spd.term.Fbk = speed3.EstimatedSpeed;
	  PID_GR_MACRO(pid1_spd)
      SpeedLoopCount=1;
     }
	else SpeedLoopCount++;   

	if(lsw==0 || lsw==1)
	{
		pid1_spd.data.ui=0;
		pid1_spd.data.i1=0;
	}

// ------------------------------------------------------------------------------
//    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID IQ controller macro
// ------------------------------------------------------------------------------  
	if(lsw==0) pid1_iq.term.Ref = 0;
    else if(lsw==1) pid1_iq.term.Ref = IqRef;
    else pid1_iq.term.Ref =  pid1_spd.term.Out; 
	pid1_iq.term.Fbk = park1.Qs;
	PID_GR_MACRO(pid1_iq)

// ------------------------------------------------------------------------------
//    Connect inputs of the PID_GRANDO_CONTROLLER module and call the PID ID controller macro
// ------------------------------------------------------------------------------  
	if(lsw==0) pid1_id.term.Ref = _IQ(0.05);
    else pid1_id.term.Ref = 0; 
	pid1_id.term.Fbk = park1.Ds;
	PID_GR_MACRO(pid1_id)

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------   
    ipark1.Ds = pid1_id.term.Out;
    ipark1.Qs = pid1_iq.term.Out;
	ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
	IPARK_MACRO(ipark1) 

// ------------------------------------------------------------------------------
//    Connect inputs of the VOLT_CALC module and call the phase voltage macro
// ------------------------------------------------------------------------------
	#ifdef F2806x_DEVICE_H
	volt1.DcBusVolt = ((AdcResult.ADCRESULT2)*0.00024414); // DC Bus voltage meas.
    #endif														 // (ADCmeas(q12)/2^12)	
    
    #ifdef DSP2803x_DEVICE_H
	volt1.DcBusVolt = _IQ15toIQ((AdcResult.ADCRESULT3<<3));	     // DC Bus voltage meas.
    #endif

    volt1.MfuncV1 = svgen_dq1.Ta;
    volt1.MfuncV2 = svgen_dq1.Tb;
    volt1.MfuncV3 = svgen_dq1.Tc;        
	VOLT_MACRO(volt1)

// ------------------------------------------------------------------------------
//    Connect inputs of the SMO_POS module and call the sliding-mode observer macro
// ------------------------------------------------------------------------------
 	smo1.Ialpha = clarke1.Alpha;
  	smo1.Ibeta  = clarke1.Beta;
    smo1.Valpha = volt1.Valpha;
    smo1.Vbeta  = volt1.Vbeta;
	SMO_MACRO(smo1)	

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_EST module and call the estimated speed macro
// ------------------------------------------------------------------------------
    speed3.EstimatedTheta = smo1.Theta;
	SE_MACRO(speed3)  

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen_dq1.Ualpha = ipark1.Alpha;
 	svgen_dq1.Ubeta = ipark1.Beta;
  	SVGEN_MACRO(svgen_dq1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(svgen_dq1.Ta);
    pwm1.MfuncC2 = _IQtoQ15(svgen_dq1.Tb);  
    pwm1.MfuncC3 = _IQtoQ15(svgen_dq1.Tc);
	PWM_MACRO(pwm1)							   	   // Calculate the new PWM compare values	

	EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;	// PWM 1A - PhaseA 
	EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;	// PWM 2A - PhaseB
	EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;	// PWM 3A - PhaseC  


// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------	
    PwmDacCh1 = _IQtoQ15(clarke1.As);
    PwmDacCh2 = _IQtoQ15(clarke1.Bs); 
    PwmDacCh3 = _IQtoQ15(volt1.VphaseA);
    PwmDacCh4 = _IQtoQ15(smo1.Theta); 

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(clarke1.As);
    DlogCh2 = _IQtoQ15(clarke1.Bs);
    DlogCh3 = _IQtoQ15(volt1.VphaseA);
    DlogCh4 = _IQtoQ15(smo1.Theta);  

#endif // (BUILDLEVEL==LEVEL7) 
	}//end if(RunMotor)

// ------------------------------------------------------------------------------
//    Call the PWMDAC update macro.
// ------------------------------------------------------------------------------
//	PWMDAC_MACRO(pwmdac1)

// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
    dlog.update(&dlog);


#if (DSP2803x_DEVICE_H==1)||(DSP280x_DEVICE_H==1)||(F2806x_DEVICE_H==1)
// Enable more interrupts from this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to recieve more interrupts from PIE group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
#endif

}

//===========================================================================
// No more.
//===========================================================================
