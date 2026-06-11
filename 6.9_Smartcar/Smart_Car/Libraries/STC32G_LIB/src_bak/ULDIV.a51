
                NAME  ULDIV

/**********************************************************************************************
 * long   uldiv(long DR4, long DR0);
 * result   = uldiv(value1, value2)
 *     DR4  = uldiv(DR4, DR0) 
 *  32位 / 32位 --> 32位(4字节无符号整数除法)  
 *  32位 % 32位 --> 32位(4字节无符号整数取模)  

 * long   ulidiv(long DR4, int WR2);
 * result   = ulidiv(value1, value2)
 *     DR4  = ulidiv(DR4, DR0) 
 *  32位 / 16位 --> 32位(4字节无符号整数除法)  
 *  32位 % 16位 --> 32位(4字节无符号整数取模)  

 *  用   途  ：STC32系列MCU
 *  作   者  ：许意义
 *  版   本  ：2.00 
 *  日   期  ：2023-2-4 

 **********************************************************************************************/

$IF ROMHUGE
Prefix	LIT '?'
Model   LIT 'FAR'
PRSeg	LIT 'ECODE'
Return  LIT 'ERET'	
$ELSE
Prefix  LIT ''
Model   LIT 'NEAR'
PRSeg	LIT 'CODE'
Return  LIT 'RET'
$ENDIF
	
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
 	
        ?PR?ULDIV?ULDIV   SEGMENT  PRSeg 
	
PUBLIC        ?C?ULDIV{Prefix}
PUBLIC        ?C?ULIDIV{Prefix}


        RSEG       ?PR?ULDIV?ULDIV
?C?ULIDIV{Prefix}    PROC	 Model
				XRL      WR0,WR0		
?C?ULDIV{Prefix}:
				MOV      DMAIR,#0x04  // 32位无符号除法
				Return	 


				ENDP
				END







		