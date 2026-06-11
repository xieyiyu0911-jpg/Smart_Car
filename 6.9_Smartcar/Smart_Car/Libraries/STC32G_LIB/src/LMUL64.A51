
                NAME  MUL64  

/**********************************************************************************************
 * void  add64(value1, value2, *pointer);
 *  *pointer = value1 * value2
 *  64位 * 64位 --> 64位(8字节整数乘法)  

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
 	
        ?PR?MUL64?MUL64  SEGMENT  CODE 
        ?DT?MUL64?MUL64  SEGMENT  DATA  OVERLAYABLE (MUL64 )  

        RSEG       ?DT?MUL64?MUL64
?DT?MUL64?START_1   EQU  $
?MUL64?BYTE    EQU  $
                   ORG  ?DT?MUL64?START_1 + 0
?MUL64_value1?040:    DSD  1		// 被加数/被减数/被乘数/被除数(高位双字),作临时变量用
?MUL64_value2?041:    DSD  1		// 被加数/被减数/被乘数/被除数(低位双字),作临时变量用
?MUL64_value3?042:    DSD  1		// 加数/减数/乘数/除数(高位双字)
?MUL64_value4?043:    DSD  1		// 加数/减数/乘数/除数(低位双字)
?MUL64_pointer?044:   DSD  1		// 计算结果回传指针 

PUBLIC        MUL64 
PUBLIC        ?MUL64?BYTE 

// void  MUL64(long DR4,long DR0,long STC_value3?042,long STC_value4?043,long* STC_pointer?044)  
        RSEG       ?PR?MUL64?MUL64

MUL64     PROC	 NEAR
// DR4:DR0 * (STC_value3?042):(STC_value4?043) --> *(STC_pointer?044) 
// DR4,DR0 * DR16,DR20 --> DR12,DR4  
			MOV     DR16, ?MUL64_value3?042
			MOV     DR20, ?MUL64_value4?043
			XRL		WR12, WR12
			MOV     ?MUL64_value1?040+2, WR12

			MOV     WR10, WR2
			MUL    	WR10, WR22    //  WR2 * WR22 --> WR12
			MOV 	?MUL64_value2?041+2, WR10 
			MOV 	WR14, WR8
			
			MOV     WR10, WR0
			MUL    	WR10, WR22    //  WR0 * WR22 + WR12--> WR12
			ADD     DR12, DR8
			JNC    	MUL64_1 
			MOV     WR10, ?MUL64_value1?040+2
			INC		WR10, #1
			MOV     ?MUL64_value1?040+2, WR10 
MUL64_1:	MOV     WR10, WR2
			MUL    	WR10, WR20    //  WR2 * WR20 + WR12--> WR12
			ADD     DR12, DR8
			JNC    	MUL64_2 
			MOV     WR10, ?MUL64_value1?040+2
			INC		WR10, #1
			MOV     ?MUL64_value1?040+2, WR10 
MUL64_2:	MOV     ?MUL64_value2?041, WR14 
			MOV     WR14, WR12 
			MOV     WR12, ?MUL64_value1?040+2
			
			MOV     WR10, WR2
			MUL    	WR10, WR18  //  WR2 * WR18 + DR12--> DR12
			ADD     DR12, DR8
			MOV     WR10, WR0
			MUL    	WR10, WR20  //  WR0 * WR20 + DR12--> DR12
			ADD     DR12, DR8
			//-------------------//
			MOV     WR10, WR6
			MUL    	WR10, WR22  //  WR6 * WR22 + DR12--> DR12
			ADD     DR12, DR8

			MOV     WR10, WR2
			MUL    	WR10, WR16  //  WR2 * WR16 + WR12--> WR12
			ADD     WR12, WR10
			MUL    	WR0, WR18   //  WR0 * WR18 + WR12--> WR12
			ADD     WR12, WR2
			//-------------------//
			MOV     WR10, WR6
			MUL    	WR10, WR20  //  WR6 * WR20 + WR12--> WR12
			ADD     WR12, WR10
			MUL    	WR4, WR22   //  WR4 * WR22 + WR12--> WR12
			ADD     WR12, WR6
			
			MOV     DR8, ?MUL64_pointer?044
			MOV     @DR8, WR12 
			MOV     @DR8+2, WR14
			MOV     DR4, ?MUL64_value2?041
			MOV     @DR8+4, WR4
			MOV     @DR8+6, WR6
		 	RET		

			ENDP
			END
