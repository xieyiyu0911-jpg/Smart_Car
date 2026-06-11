
                NAME  SLDIV_HUGE

/**********************************************************************************************
 * long   sldiv(long DR4, long DR0);
 * result   = sldiv(value1, value2)
 *     DR4  = sldiv(DR4, DR0) 
 *  32位 / 32位 --> 32位(4字节有符号整数除法)  
 *  32位 % 32位 --> 32位(4字节有符号整数取模)  

 *  用   途  ：STC32系列MCU
 *  作   者  ：许意义
 *  版   本  ：2.00 
 *  日   期  ：2023-2-4 

 **********************************************************************************************/
	
// 特殊功能寄存器和位
           DMAIR  DATA  0EDH
             ACC  DATA  0E0H
               B  DATA  0F0H
              SP  DATA  081H
             SPH  DATA  085H
             DPL  DATA  082H
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
// 结束--特殊功能寄存器和位
 	
        ?PR?SLDIV?SLDIV   SEGMENT  ECODE 
	
PUBLIC        ?C?SLDIV?

        RSEG       ?PR?SLDIV?SLDIV
?C?SLDIV?    PROC	 FAR
SSLDIV:	
				MOV      DMAIR,#0x06  // 32位有符号除法
				ERET	 

				ENDP
				END







		