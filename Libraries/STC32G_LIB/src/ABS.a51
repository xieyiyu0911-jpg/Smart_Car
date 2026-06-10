
			NAME     ABS 

/**********************************************************************************************
 * int   WR6  labs(int WR6);
 * result   = labs(value1)
 *     WR6  = labs(WR6) 
 *  16位-->16位(16位2进制整数绝对值)  

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

?PR?_ABS?ABS SEGMENT CODE 
PUBLIC ?C?ABS
RSEG  ?PR?_ABS?ABS

?C?ABS      PROC

			MOV      WR4,WR6		// 16位2进制整数绝对值
			CMP      R4,#0x00
			JSGE     _ABS_001_
			XRL      WR6,WR6
			SUB      WR6,WR4
_ABS_001_:
			RET

			ENDP
		
			END