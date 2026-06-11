
                NAME  SUB64_HUGE  

/**********************************************************************************************
 * void  add64(value1, value2, *pointer);
 *  *pointer = value1 - value2
 *  64位 - 64位 --> 64位(8字节整数减法) 

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
 	
        ?PR?SUB64?SUB64  SEGMENT  ECODE 
        ?DT?SUB64?SUB64  SEGMENT  DATA  OVERLAYABLE (SUB64?)  

        RSEG       ?DT?SUB64?SUB64
?DT?SUB64?START_1   EQU  $
?SUB64??BYTE    EQU  $
                   ORG  ?DT?SUB64?START_1 + 0
?SUB64_value1?040:    DSD  1		// 被加数/被减数/被乘数/被除数(高位双字),作临时变量用
?SUB64_value2?041:    DSD  1		// 被加数/被减数/被乘数/被除数(低位双字),作临时变量用
?SUB64_value3?042:    DSD  1		// 加数/减数/乘数/除数(高位双字)
?SUB64_value4?043:    DSD  1		// 加数/减数/乘数/除数(低位双字)
?SUB64_pointer?044:   DSD  1		// 计算结果回传指针 

PUBLIC        SUB64?
PUBLIC        ?SUB64??BYTE 

// void  SUB64(long DR4,long DR0,long STC_value3?042,long STC_value4?043,long* STC_pointer?044)  
RSEG       ?PR?SUB64?SUB64

SUB64?    PROC	 FAR
// DR4:DR0 - (STC_value3?042):(STC_value4?043) --> *(STC_pointer?044) 
			MOV     DR12, ?SUB64_value4?043
			SUB   	DR0, DR12
			JNC    	SUB64_1
			DEC		DR4,#1
SUB64_1:	MOV     DR12, ?SUB64_value3?042 
			SUB    	DR4, DR12
			MOV     DR8, ?SUB64_pointer?044
			MOV     @DR8, WR4
			MOV     @DR8+2, WR6
			MOV     @DR8+4, WR0
			MOV     @DR8+6, WR2
		 	ERET

			ENDP
			END
