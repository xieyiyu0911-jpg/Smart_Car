
                NAME  FPGETOPN2_HUGE

/**********************************************************************************************
 * 内部函数
 * 用于 4字节单精度浮点数加减乘除法  

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
 	
        ?PR?FPGETOPN2?FPGETOPN2   SEGMENT  ECODE

PUBLIC        ?C?FPGETOPN2?
PUBLIC        ?C?FPRESULT?
PUBLIC        ?C?FPRESULT2?
PUBLIC        ?C?FPNANRESULT?
PUBLIC        ?C?FPUNDERFLOW?
PUBLIC        ?C?FPOVERFLOW?

        RSEG       ?PR?FPGETOPN2?FPGETOPN2
			
?C?FPGETOPN2?    PROC	 FAR
				ADD      WR0,WR0			// 浮点数小数,指数分离, 符号F0, 小数R1R2R3, 指数R0, 小数R5R6R7, R4清0, 指数R8,ACC 
				MOV      F0,C
				SRA      R1
				ORL      R1,#0x80
				MOV      WR8,WR4
				ADD      WR8,WR8
				JNC      _FPGETOPN2_001
				CPL      F0
_FPGETOPN2_001: ORL      R5,#0x80
				XRL      R4,R4
				MOV      R11,R8
				ERET 
				ENDP

?C?FPRESULT?     PROC	 FAR
				ADD      R4,R4					// 浮点数规格化 (小数R5R6R7R4, 指数R8)
				JNC      ?C?FPRESULT2?
				INC      DR4,#0x01				// 四舍五入
				MOV      R4,#0x00
				ANL      R5,#0x7F
				CMP      DR4,#0x0000
				JNE      ?C?FPRESULT2?
				INC      R8,#0x01
				JE       ?C?FPOVERFLOW?
?C?FPRESULT2?:
				MOV      C,F0					// 符号, 指数规范后, 返回
				MOV      R11,R8
				RRC      A
				MOV      R4,R11
				MOV      R11,R5
				MOV      ACC.7,C
				MOV      R5,R11
				ERET 
?C?FPNANRESULT?:
				MOV      DR4,#0xFFFFFFFF		// 非浮点数
				ERET 
?C?FPUNDERFLOW?:
				SUB      DR4,DR4				// 零
				ERET 
?C?FPOVERFLOW?:
				MOV      WR4,#0xFF80			// 溢出
				JB       F0,_FPOVERFLOW_001
				XRL      R4,R5
_FPOVERFLOW_001:XRL      WR6,WR6
				ERET 
				ENDP

				END
			