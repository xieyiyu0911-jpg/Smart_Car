
                NAME  FPMUL 

/**********************************************************************************************
 * float   fpmul(float DR4, float DR0);
 * result   = fpmul(value1, value2)
 *     DR4  = fpmul(DR4, DR0) 
 *  32位 * 32位 --> 32位(4字节单精度浮点数乘法)  

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
 	
        ?PR?FPMUL?FPMUL   SEGMENT  CODE 

PUBLIC        ?C?FPMUL 
EXTRN		  CODE : NEAR (?C?FPGETOPN2)
EXTRN		  CODE : NEAR (?C?FPRESULT)
EXTRN		  CODE : NEAR (?C?FPRESULT2)
EXTRN		  CODE : NEAR (?C?FPNANRESULT)
EXTRN		  CODE : NEAR (?C?FPUNDERFLOW)
EXTRN		  CODE : NEAR (?C?FPOVERFLOW)


 

        RSEG       ?PR?FPMUL?FPMUL 
?C?FPMUL     PROC	 NEAR
				ANL      WR4,WR4
				JE       _FPMUL_002
				ANL      WR0,WR0
				JNE      _FPMUL_004
				ADD      WR4,WR4
				INC      R4,#0x01
				JE       _FPMUL_003
				SUB      DR4,DR4
_FPMUL_001:     RET	 

_FPMUL_002:     ADD      WR0,WR0
				INC      R0,#0x01
				JNE      _FPMUL_001
_FPMUL_003:     JMP      ?C?FPNANRESULT  
_FPMUL_004:     CALL     ?C?FPGETOPN2 
				ANL      R8,R0
				INC      R8,#0x01
				JE       _FPMUL_005
				ADD      A,#0x81
				JNC      _FPMUL_006
				ADD      R11,R0
				JNC      _FPMUL_007
_FPMUL_005:     JMP      ?C?FPOVERFLOW 
_FPMUL_006:     ADD      R11,R0
				JC       _FPMUL_007
				JMP      ?C?FPUNDERFLOW 
_FPMUL_007:     MOV      R8,R11
				ANL      WR2,WR2
				JNE      _FPMUL_013
				MOV      R11,R1
				MOV      R10,R5
				CMP      R1,#0x80
				JE       _FPMUL_009
				ANL      WR6,WR6
				JNE      _FPMUL_011
				CJNE     R5,#0x80,_FPMUL_010
_FPMUL_008:     MOV      DR4,DR0
_FPMUL_009:     JMP     ?C?FPRESULT2 

_FPMUL_010:     XRL      WR0,WR0
				SJMP     _FPMUL_016
_FPMUL_011:     MOV      DR0,DR4
				MOV      R5,R11
_FPMUL_012:     MOV      R11,R1
				MOV      R10,R5
				MUL      WR2,WR4
				SJMP     _FPMUL_016
_FPMUL_013:     ANL      WR6,WR6
				JNE      _FPMUL_014
				CJNE     R5,#0x80,_FPMUL_012
				SJMP     _FPMUL_008
_FPMUL_014:     PUSH     R8
				MOV      WR8,WR2
				MUL      WR8,WR6
				MOV      R11,R1
				MOV      R10,R5
				MUL      WR2,WR4
				ADD      WR2,WR8
				JNC      _FPMUL_015
				INC      WR0,#0x01
_FPMUL_015:     MOV      R5,R11
				MUL      WR4,WR6
				ADD      DR0,DR4
				POP      R8
_FPMUL_016:     MUL      AB
				ADD      WR0,WR10
				CMP      R0,#0x80
				JC       _FPMUL_017
				INC      R8,#0x01
				JNE      _FPMUL_018
				SJMP     _FPMUL_005
_FPMUL_017:     ADD      DR0,DR0
_FPMUL_018:     MOV      R4,R3
				MOV      R5,R0
				MOV      R6,R1
				MOV      R7,R2
				JMP      ?C?FPRESULT 

				ENDP
				END







		