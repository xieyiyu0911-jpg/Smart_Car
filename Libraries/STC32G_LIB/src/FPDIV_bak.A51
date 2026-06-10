
                NAME  FPDIV 

/**********************************************************************************************
 * float   fpdiv(float DR4, float DR0);
 * result   = fpdiv(value1, value2)
 *     DR4  = fpdiv(DR4, DR0) 
 *  32位 / 32位 --> 32位(4字节单精度浮点数除法)  

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
 	
        ?PR?FPDIV?FPDIV   SEGMENT  PRSeg 

PUBLIC        ?C?FPDIV{Prefix}
EXTRN		  CODE : Model (?C?FPGETOPN2{Prefix})
EXTRN		  CODE : Model (?C?FPRESULT{Prefix})
EXTRN		  CODE : Model (?C?FPRESULT2{Prefix})
EXTRN		  CODE : Model (?C?FPNANRESULT{Prefix})
EXTRN		  CODE : Model (?C?FPUNDERFLOW{Prefix})
EXTRN		  CODE : Model (?C?FPOVERFLOW{Prefix})
 

        RSEG       ?PR?FPDIV?FPDIV
?C?FPDIV{Prefix}    PROC	 Model

//?C?FPDIV:
				CMP      WR4,#0xFFFF
				JE       _FPDIV_001
				CMP      WR0,#0xFFFF
				JNE      _FPDIV_002
_FPDIV_001:     JMP     ?C?FPNANRESULT{Prefix}
_FPDIV_002:     CALL    ?C?FPGETOPN2{Prefix}
				ANL      R11,R0
				INC      A
				JE       _FPDIV_001
				MOV      R11,R8
				ORL      A,R0
				JE       _FPDIV_001
				MOV      R11,R8
				JNZ      _FPDIV_003
				SUB      DR4,DR4
				Return	 
_FPDIV_003:     CJNE     R0,#0x00,_FPDIV_005
_FPDIV_004:     JMP     ?C?FPOVERFLOW{Prefix}
_FPDIV_005:     ADD      R0,#0x81
				JNC      _FPDIV_007
				SUB      R8,R0
				JG       _FPDIV_008
_FPDIV_006:     JMP     ?C?FPUNDERFLOW{Prefix}
_FPDIV_007:     SUB      R8,R0
				JNC      _FPDIV_004
_FPDIV_008:     MOVZ     WR10,R1
				CLR      ACC.7
				ORL      WR10,WR2
				JNE      _FPDIV_009
				JMP     ?C?FPRESULT2{Prefix}
				
_FPDIV_009:     MOV      WR10,#0x0080
				MOV      R9,#0x1A
				XRL      R0,R0
				SJMP     _FPDIV_011
_FPDIV_010:     DEC      R9,#0x01
				JE       _FPDIV_013
				ADD      DR4,DR4
_FPDIV_011:     CMP      DR4,DR0
				JC       _FPDIV_012
				SUB      DR4,DR0
				ORL      R10,R11
_FPDIV_012:     RR       A
				JNB      ACC.7,_FPDIV_010
				PUSH     R10
				XRL      R10,R10
				SJMP     _FPDIV_010
				
_FPDIV_013:     POP      WR6
				POP      R5
				MOV      R4,R10
				CMP      R5,#0x80
				JNC      _FPDIV_014
				DEC      R8,#0x01
				JE       _FPDIV_006
				ADD      DR4,DR4
				JNC      _FPDIV_014
				INC      R7,#0x01
_FPDIV_014:     JMP     ?C?FPRESULT{Prefix}
			
				ENDP
				END







		