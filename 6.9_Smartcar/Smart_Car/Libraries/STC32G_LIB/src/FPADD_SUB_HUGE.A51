
                NAME  FPADD_SUB_HUGE

/**********************************************************************************************
 * float   fpadd(float DR4, float DR0);
 * result   = fpadd(value1, value2)
 *     DR4  = fpadd(DR4, DR0) 
 *  32位 + 32位 --> 32位(4字节单精度浮点数加法)  

 * float DR4  fpsub(float DR4, float DR0);
 * result   = fpsub(value1, value2)
 *     DR4  = fpsub(DR4, DR0) 
 *  32位 - 32位 --> 32位(4字节单精度浮点数减法)  

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
 	
        ?PR?FPADD_SUB?FPADD_SUB  SEGMENT  ECODE 

PUBLIC        ?C?FPADD?
PUBLIC        ?C?FPSUB?
EXTRN		  CODE : FAR (?C?FPGETOPN2?)
EXTRN		  CODE : FAR (?C?FPRESULT?)
EXTRN		  CODE : FAR (?C?FPRESULT2?)
EXTRN		  CODE : FAR (?C?FPNANRESULT?)
EXTRN		  CODE : FAR (?C?FPUNDERFLOW?)
EXTRN		  CODE : FAR (?C?FPOVERFLOW?)
 

        RSEG       ?PR?FPADD_SUB?FPADD_SUB
FPADD_SUB?    PROC	 FAR

		
_FPADD_001:     JMP     ?C?FPNANRESULT?
?C?FPSUB?:
				XRL      R0,#0x80
?C?FPADD?:
				MOV      R11,R1
				RLC      A
				MOV      R11,R0
				RLC      A
				JE       _FPADD_002
				INC      A
				JE       _FPADD_001
				MOV      R11,R5
				RLC      A
				MOV      R11,R4
				RLC      A
				JNE      _FPADD_003
				MOV      DR4,DR0
_FPADD_002:     ERET	 
_FPADD_003:     INC      A
				JE       _FPADD_001
				MOV      WR10,WR0
				MOV      B.7,C
				MOV      WR8,WR10
				MOV      WR10,WR2
				CMP      DR8,DR4
				JC       _FPADD_004
				MOV      R8,R0
				MOV      DR0,DR4
				MOV      DR4,DR8
_FPADD_004:     CALL     ?C?FPGETOPN2?
				MOV      R10,PSW
				ANL      R11,R0
				INC      A
				JNE      _FPADD_005
				JB       F0,_FPADD_001
_FPADD_005:     MOV      R11,R0
				INC      A
				JNE      _FPADD_007
				JNC      _FPADD_006
				CPL      F0
_FPADD_006:     JMP      ?C?FPOVERFLOW?
_FPADD_007:     MOV      F0,C
				CMP      R8,#0xFF
				JE       _FPADD_006
				MOV      R11,R8
				SUB      R11,R0
				XRL      R0,R0
				JZ       _FPADD_013
				CJNE     A,#0x18,_FPADD_009
				MOV      R4,R1
_FPADD_008:     JMP      ?C?FPRESULT?
_FPADD_009:     JNC      _FPADD_008
				SJMP     _FPADD_011
_FPADD_010:     ADD      DR0,DR0
				INC      A
_FPADD_011:     MOV      R9,R11
				ANL      R9,#0x07
				JNE      _FPADD_010
				JNB      ACC.4,_FPADD_012
				MOV      R4,R3
				MOV      WR2,WR0
				XRL      WR0,WR0
_FPADD_012:     JNB      ACC.3,_FPADD_013
				MOV      R4,R3
				MOV      R3,R2
				MOV      R2,R1
				MOVZ     WR0,R0
_FPADD_013:     JNB      B.5,_FPADD_017
				SUB      DR4,DR0
				JNE      _FPADD_016
				ERET	 
_FPADD_014:     JMP      ?C?FPUNDERFLOW?
_FPADD_015:     DEC      R8,#0x01
				JE       _FPADD_014
				ADD      DR4,DR4
				JNC      _FPADD_016
				INC      R7,#0x01
_FPADD_016:     MOV      R11,R5
				JNB      ACC.7,_FPADD_015
				SJMP     _FPADD_020
_FPADD_017:     ADD      WR6,WR2
				MOV      R11,R5
				ADDC     A,R1
				MOV      R5,R11
				JNC      _FPADD_020
				INC      R8,#0x01
				JNE      _FPADD_018
				JMP      ?C?FPOVERFLOW?
_FPADD_018:     SRL      WR6
				RRC      A
				JNC      _FPADD_019
				ORL      R6,#0x80
_FPADD_019:     MOVS     WR4,R11
_FPADD_020:     JMP      ?C?FPRESULT?
			//	ERET	 
			
				ENDP
				END

