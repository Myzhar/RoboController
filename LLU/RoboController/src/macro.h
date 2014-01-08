/* Boot Segment Program Memory:No Boot Segment Program Memory
** Write Protect:Disabled **/
//_FBS(BSS_NO_FLASH & BWRP_WRPROTECT_OFF);

//   Watchdog Timer Enable:
_FWDT(FWDTEN_OFF) //Watchdog timer enabled/disabled by user software
 
// External Oscillator
_FOSCSEL(FNOSC_PRI);			// Primary (XT, HS, EC) Oscillator

// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystanl
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_XT);

/* Background Debug Enable Bit: Device will Reset in user mode
** Debugger/Emulator Enable Bit: Reset in operational mode
** JTAG Enable Bit: JTAG is disabled
** ICD communication channel select bits:
** communicate on PGC1/EMUC1 and PGD1/EMUD1 **/
//_FICD(BKBUG_ON & COE_ON & JTAGEN_OFF & ICS_PGD1); // valid until C30 3.11
_FICD(JTAGEN_OFF & ICS_PGD1);

/*Enable MCLR reset pin and turn on the power-up timers with 64ms delay.
PIN managed by PWM at reset*/
_FPOR(FPWRT_PWR64 & PWMPIN_ON & HPOL_ON & LPOL_ON);

/* Code Protect: Code Protect off
** Code Protect: Disabled
** Write Protect: Disabled **/
_FGS(GSS_OFF & GCP_OFF & GWRP_OFF);
