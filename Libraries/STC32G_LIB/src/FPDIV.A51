
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
 	
        ?PR?FPDIV?FPDIV   SEGMENT  CODE 
	
PUBLIC        ?C?FPDIV 
EXTRN		  CODE : NEAR (?C?FPGETOPN2)
EXTRN		  CODE : NEAR (?C?FPRESULT)
EXTRN		  CODE : NEAR (?C?FPRESULT2)
EXTRN		  CODE : NEAR (?C?FPNANRESULT)
EXTRN		  CODE : NEAR (?C?FPUNDERFLOW)
EXTRN		  CODE : NEAR (?C?FPOVERFLOW)
 

        RSEG       ?PR?FPDIV?FPDIV
?C?FPDIV     PROC	 NEAR
				CMP      WR4,#0xFFFF
				JE       _FPDIV_001B
				CMP      WR0,#0xFFFF
				JNE      _FPDIV_002B
_FPDIV_001B:    JMP      ?C?FPNANRESULT 
_FPDIV_002B:    CALL     ?C?FPGETOPN2 
				ANL      R11,R0
				INC      A
				JE       _FPDIV_001B
				MOV      R11,R8
				ORL      A,R0
				JE       _FPDIV_001B
				MOV      R11,R8
				JNZ      _FPDIV_003B
				SUB      DR4,DR4
				RET	 
_FPDIV_003B:    CJNE     R0,#0x00,_FPDIV_005B
_FPDIV_004B:    JMP      ?C?FPOVERFLOW 
_FPDIV_005B:    ADD      R0,#0x81
				JNC      _FPDIV_007B
				SUB      R8,R0
				JG       _FPDIV_008B
_FPDIV_006B:    JMP      ?C?FPUNDERFLOW 
_FPDIV_007B:    SUB      R8,R0
				JNC      _FPDIV_004B
_FPDIV_008B:    MOVZ     WR10,R1
				CLR      ACC.7
				ORL      WR10,WR2
				JNE      _FPDIV_009B
				JMP      ?C?FPRESULT2 
_FPDIV_009B: 	XRL      R0,R0
				PUSH     R8  
				MOV      DR8, DR0  	// 除数 --> DR16

				MOV      R4, R5  
				MOV      R5, R6  
				MOV      R6, R7 
				XRL      R7, R7 
				MOV      DMAIR,#0x4     // R4R5R6R_ / R_R1R2R3 
				PUSH     WR6            // 商(最高2位)
			
				MOV      R4, R1  
				MOV      R5, R2  
				MOV      R6, R3  
				XRL      R7, R7 
				MOV      DR0, DR8  
				MOV      DMAIR,#0x4     // R4R5R6R_ / R_R1R2R3 
				PUSH     R7             // 商(次高位)
	
				MOV      R4, R1  
				MOV      R5, R2  
				MOV      R6, R3  
				XRL      R7, R7 
				MOV      DR0, DR8  
				MOV      DMAIR,#0x4     // R4R5R6R_ / R_R1R2R3 
				POP		 R6
				POP		 WR4
				
				CMP		 R4, #0
				JE		 _FPDIV_011B
				SRL 	 WR6
				RRC      A
				SRL      WR4
				JNC      _FPDIV_010B
				ORL      R6, #0x80
_FPDIV_010B:	MOV      R4, R11
				POP		 R8
				JMP      ?C?FPRESULT 
_FPDIV_011B:	ADD		 DR0, DR0
				SUB		 DR8, DR0
				MOV		 R4, R8
				POP		 R8
				DEC		 R8, #1
				JMP      ?C?FPRESULT 
			
				ENDP
				END







		