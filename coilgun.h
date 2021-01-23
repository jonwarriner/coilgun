/* ==============================================================================
System Name:  	coilgun

File Name:		coilgun.h

Description:	Primary system header file for the Real Implementation of Sensorless  
          		Field Orientation Control for a Three Phase Permanent-Magnet
          		Synchronous Motor 

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DRV8312-EVM. 
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 02-07-2011	Version 1.0
=================================================================================  */

/*-------------------------------------------------------------------------------
Next, Include project specific include files.
-------------------------------------------------------------------------------*/

#include "rmp_cntl.h"           // Include header for the RMPCNTL object
#include "svgen_mf.h"       	// Include header for the SVGEN_MF object
#include "vhzprof.h"            // Include header for VHZ_PROF object

#if (DSP2803x_DEVICE_H==1)
#include "f2803xileg_vdc_PM.h" 	// Include header for the ILEG2DCBUSMEAS object 
#include "f2803xpwm_PM.h"       // Include header for the PWMGEN object
#include "f2803xpwmdac_PM.h"    // Include header for the PWMGEN object
#include "f2803xqep_PM.h"       // Include header for the QEP object
#endif

#if (F2806x_DEVICE_H==1)
#include "f2806xileg_vdc_PM.h" 	// Include header for the ILEG2DCBUSMEAS object 
#include "f2806xpwm_PM.h"       // Include header for the PWMGEN object
#include "f2806xpwmdac_PM.h"    // Include header for the PWMGEN object
#endif

#include "dlog4ch-coilgun.h" // Include header for the DLOG_4CH object

#ifdef DRV8301
#include "DRV8301_SPI.h"
#endif

//===========================================================================
// No more.
//===========================================================================
