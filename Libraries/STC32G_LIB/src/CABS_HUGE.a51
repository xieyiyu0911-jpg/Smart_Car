
NAME     CABS_HUGE

/**********************************************************************************************
 * char  ACC  cabs(char ACC);
 * result   = cabs(value1)
 *     ACC  = cabs(ACC) 
 *  8位-->8位(char绝对值)  

 *  用   途  ：STC32系列MCU
 *  作   者  ：许意义
 *  版   本  ：2.00 
 *  日   期  ：2022-12-12  

 **********************************************************************************************/


// 特殊功能寄存器和位 
           DMAIR  DATA  0EDH
             ACC  DATA  0E0H		// R11  =  (ACC)
               B  DATA  0F0H		// R10  =  ( B )
              SP  DATA  081H		// DR60 =  (/, /, SPH, SPL) 
             SPH  DATA  085H
             DPL  DATA  082H		// DR56 =  (/, DPXL, DPH, DPL) 
             DPH  DATA  083H
            DPXL  DATA  084H
             PSW  DATA  0D0H
            PSW1  DATA  0D1H
               Z  BIT   0D1H.1
              OV  BIT   0D0H.2
               P  BIT   0D0H.0
              F0  BIT   0D0H.5
             RS1  BIT   0D0H.4
             RS0  BIT   0D0H.3
              AC  BIT   0D0H.6
              EA  BIT   0A8H.7
// 结束_特殊功能寄存器和位

?PR?_CABS?CABS? SEGMENT ECODE    
PUBLIC ?C?CABS?
RSEG  ?PR?_CABS?CABS?

?C?CABS?    PROC	 FAR

			JNB      ACC.7, _CABS_001_    // 8位2进制整数char绝对值
			CPL      A
			INC      A   
_CABS_001_:
			ERET

			ENDP
		
			END