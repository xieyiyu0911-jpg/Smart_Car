
                NAME  SDIV64  

/**********************************************************************************************
 * void  sdiv64(value1, value2, *pointer);
 *  *pointer = value1 / value2 或
 *  *pointer = value1 % value2
 *  64位 / 64位 --> 64位(8字节有符号整数除法)  
 *  64位 % 64位 --> 64位(8字节有符号整数取模)  

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
 	
        ?PR?SDIV64?SDIV64  SEGMENT  PRSeg 
        ?DT?SDIV64?SDIV64  SEGMENT  DATA  OVERLAYABLE (SDIV64{Prefix})  

        RSEG       ?DT?SDIV64?SDIV64
?DT?SDIV64?START_1   EQU  $
?SDIV64{Prefix}?BYTE    EQU  $
                   ORG  ?DT?SDIV64?START_1 + 0
?SDIV64_value1?040:    DSD  1		// 被加数/被减数/被乘数/被除数(高位双字),作临时变量用
?SDIV64_value2?041:    DSD  1		// 被加数/被减数/被乘数/被除数(低位双字),作临时变量用
?SDIV64_value3?042:    DSD  1		// 加数/减数/乘数/除数(高位双字)
?SDIV64_value4?043:    DSD  1		// 加数/减数/乘数/除数(低位双字)
?SDIV64_value5?044:    DS   1		// 求模/除法 标志 
?SDIV64_pointer?045:   DSD  1		// 计算结果回传指针 

PUBLIC        SDIV64{Prefix}
PUBLIC        ?SDIV64{Prefix}?BYTE 
EXTRN		  CODE : Model (UDIV64_1{Prefix})

// void  SDIV64(long DR4,long DR0,long STC_value3?042,long STC_value4?043,long* STC_pointer?044)  
RSEG       ?PR?SDIV64?SDIV64
	
SDIV64{Prefix}    PROC	 Model
// DR4:DR0 * (STC_value3?042):(STC_value4?043) --> *(STC_pointer?044) 
// DR4,DR0 / DR16,DR20 --> 商: DR24,DR20, 余数: DR4,DR0 
			MOV      DR16, ?SDIV64_value3?042 
			MOV      DR20, ?SDIV64_value4?043 

//SDIV64:    
//	        CLR      F0
//	        MOV      R11,R0
//	      JNB      0xE0.7,SDIV64_1
			MOV      WR8,WR16
			CMP		 R8, #80H	// 除数高位为0(正数),跳转
			JC		 SDIV64_1

//	        CPL      F0
			XRL		 A, #80H
	        SUB      DR12,DR12
	        SUB      DR12,DR20
	        MOV      DR20,DR12
			MOV		 F0, C
	        SUB      DR12,DR12
	        SUB      DR12,DR16
	        MOV      DR16,DR12
			JNB      F0, SDIV64_1
			DEC 	 DR16, #1
SDIV64_1: 
//	       MOV      R11,R4
//	      JNB      0xE0.7,SDIV64_2
			CMP		 R4,#80H	// 被除数高位为0(正数),跳转
			JC		 SDIV64_2

//	        CPL      F0
			XRL		 A, #0C0H
//			JNB		 ACC.7, SDIV64_3
//			SETB	 ACC.6
SDIV64_3:   SUB      DR12,DR12
	        SUB      DR12,DR0
	        MOV      DR0,DR12
			MOV		 F0, C
	        SUB      DR12,DR12
	        SUB      DR12,DR4
	        MOV      DR4,DR12
			JNB      F0, SDIV64_2
			DEC 	 DR4, #1
SDIV64_2:      
			MOV      DR12, ?SDIV64_pointer?045
			PUSH	 DR12 
			JMP      UDIV64_1{Prefix}

			ENDP
			END

/*
//SDIV64:    
//	        CLR      F0
//	        MOV      R11,R0
//	      JNB      0xE0.7,SDIV64_1
		CMP	R0, #80H
		JC	SDIV64_1

//	        CPL      F0
		XRL	R11, #80H
	        SUB      DR8,DR8
	        SUB      DR8,DR0
	        MOV      DR0,DR8
SDIV64_1: 
//	       MOV      R11,R4
//	      JNB      0xE0.7,SDIV64_2
		CMP	R4,#80H
		JC	SDIV64_2

//	        CPL      F0
		XRL	R11, #C0H
	        SUB      DR8,DR8
	        SUB      DR8,DR4
	        MOV      DR4,DR8
		PUSH	R11 
	      LCALL    C?ULDIV(C:0x0231)  
	        SUB      DR8,DR8
	        SUB      DR8,DR0
	        MOV      DR0,DR8
	        SJMP     SDIV64_3

SDIV64_2:      
		PUSH	R11 
		LCALL    C?ULDIV(C:0x0231)
SDIV64_3:
	      JNB      F0,SDIV64_4
	        SUB      DR8,DR8
	        SUB      DR8,DR4
	        MOV      DR4,DR8
SDIV64_4:          RET    
  */