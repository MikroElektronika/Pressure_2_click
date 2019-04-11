_systemInit:
;Click_Pressure2_STM.c,34 :: 		void systemInit()
SUB	SP, SP, #4
STR	LR, [SP, #0]
;Click_Pressure2_STM.c,36 :: 		mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_CS_PIN, _GPIO_OUTPUT );
MOVS	R2, #0
MOVS	R1, #2
MOVS	R0, #0
BL	_mikrobus_gpioInit+0
;Click_Pressure2_STM.c,37 :: 		mikrobus_spiInit( _MIKROBUS1, &_PRESSURE2_SPI_CFG[0] );
MOVW	R0, #lo_addr(__PRESSURE2_SPI_CFG+0)
MOVT	R0, #hi_addr(__PRESSURE2_SPI_CFG+0)
MOV	R1, R0
MOVS	R0, #0
BL	_mikrobus_spiInit+0
;Click_Pressure2_STM.c,38 :: 		mikrobus_logInit( _LOG_USBUART_A, 9600 );
MOVW	R1, #9600
MOVS	R0, #32
BL	_mikrobus_logInit+0
;Click_Pressure2_STM.c,39 :: 		Delay_ms( 100 );
MOVW	R7, #20351
MOVT	R7, #18
NOP
NOP
L_systemInit0:
SUBS	R7, R7, #1
BNE	L_systemInit0
NOP
NOP
NOP
;Click_Pressure2_STM.c,40 :: 		}
L_end_systemInit:
LDR	LR, [SP, #0]
ADD	SP, SP, #4
BX	LR
; end of _systemInit
_applicationInit:
;Click_Pressure2_STM.c,42 :: 		void applicationInit()
SUB	SP, SP, #4
STR	LR, [SP, #0]
;Click_Pressure2_STM.c,44 :: 		pressure2_spiDriverInit( (T_PRESSURE2_P)&_MIKROBUS1_GPIO, (T_PRESSURE2_P)&_MIKROBUS1_SPI );
MOVW	R1, #lo_addr(__MIKROBUS1_SPI+0)
MOVT	R1, #hi_addr(__MIKROBUS1_SPI+0)
MOVW	R0, #lo_addr(__MIKROBUS1_GPIO+0)
MOVT	R0, #hi_addr(__MIKROBUS1_GPIO+0)
BL	_pressure2_spiDriverInit+0
;Click_Pressure2_STM.c,45 :: 		pressure2_init();
BL	_pressure2_init+0
;Click_Pressure2_STM.c,46 :: 		Delay_ms( 300 );
MOVW	R7, #61055
MOVT	R7, #54
NOP
NOP
L_applicationInit2:
SUBS	R7, R7, #1
BNE	L_applicationInit2
NOP
NOP
NOP
;Click_Pressure2_STM.c,47 :: 		}
L_end_applicationInit:
LDR	LR, [SP, #0]
ADD	SP, SP, #4
BX	LR
; end of _applicationInit
_applicationTask:
;Click_Pressure2_STM.c,49 :: 		void applicationTask()
SUB	SP, SP, #4
STR	LR, [SP, #0]
;Click_Pressure2_STM.c,51 :: 		pressure2_readSensor( &pressure_P, &pressure_T );
MOVW	R1, #lo_addr(_pressure_T+0)
MOVT	R1, #hi_addr(_pressure_T+0)
MOVW	R0, #lo_addr(_pressure_P+0)
MOVT	R0, #hi_addr(_pressure_P+0)
BL	_pressure2_readSensor+0
;Click_Pressure2_STM.c,53 :: 		mikrobus_logWrite( "Pressure: ", _LOG_TEXT );
MOVW	R0, #lo_addr(?lstr1_Click_Pressure2_STM+0)
MOVT	R0, #hi_addr(?lstr1_Click_Pressure2_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Pressure2_STM.c,54 :: 		FloatToStr( pressure_P, demoText );
MOVW	R0, #lo_addr(_pressure_P+0)
MOVT	R0, #hi_addr(_pressure_P+0)
LDR	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_FloatToStr+0
;Click_Pressure2_STM.c,55 :: 		demoText[6] = 0;
MOVS	R1, #0
MOVW	R0, #lo_addr(_demoText+6)
MOVT	R0, #hi_addr(_demoText+6)
STRB	R1, [R0, #0]
;Click_Pressure2_STM.c,56 :: 		mikrobus_logWrite( demoText, _LOG_LINE );
MOVS	R1, #2
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Pressure2_STM.c,58 :: 		mikrobus_logWrite( "Temperature: ", _LOG_TEXT );
MOVW	R0, #lo_addr(?lstr2_Click_Pressure2_STM+0)
MOVT	R0, #hi_addr(?lstr2_Click_Pressure2_STM+0)
MOVS	R1, #1
BL	_mikrobus_logWrite+0
;Click_Pressure2_STM.c,59 :: 		FloatToStr( pressure_T, demoText );
MOVW	R0, #lo_addr(_pressure_T+0)
MOVT	R0, #hi_addr(_pressure_T+0)
LDR	R0, [R0, #0]
MOVW	R1, #lo_addr(_demoText+0)
MOVT	R1, #hi_addr(_demoText+0)
BL	_FloatToStr+0
;Click_Pressure2_STM.c,60 :: 		demoText[4] = 0;
MOVS	R1, #0
MOVW	R0, #lo_addr(_demoText+4)
MOVT	R0, #hi_addr(_demoText+4)
STRB	R1, [R0, #0]
;Click_Pressure2_STM.c,61 :: 		mikrobus_logWrite( demoText, _LOG_LINE );
MOVS	R1, #2
MOVW	R0, #lo_addr(_demoText+0)
MOVT	R0, #hi_addr(_demoText+0)
BL	_mikrobus_logWrite+0
;Click_Pressure2_STM.c,62 :: 		mikrobus_logWrite( "-----------", _LOG_LINE );
MOVW	R0, #lo_addr(?lstr3_Click_Pressure2_STM+0)
MOVT	R0, #hi_addr(?lstr3_Click_Pressure2_STM+0)
MOVS	R1, #2
BL	_mikrobus_logWrite+0
;Click_Pressure2_STM.c,63 :: 		Delay_ms( 1000 );
MOVW	R7, #6911
MOVT	R7, #183
NOP
NOP
L_applicationTask4:
SUBS	R7, R7, #1
BNE	L_applicationTask4
NOP
NOP
NOP
;Click_Pressure2_STM.c,64 :: 		}
L_end_applicationTask:
LDR	LR, [SP, #0]
ADD	SP, SP, #4
BX	LR
; end of _applicationTask
_main:
;Click_Pressure2_STM.c,66 :: 		void main()
;Click_Pressure2_STM.c,68 :: 		systemInit();
BL	_systemInit+0
;Click_Pressure2_STM.c,69 :: 		applicationInit();
BL	_applicationInit+0
;Click_Pressure2_STM.c,71 :: 		while (1)
L_main6:
;Click_Pressure2_STM.c,73 :: 		applicationTask();
BL	_applicationTask+0
;Click_Pressure2_STM.c,74 :: 		}
IT	AL
BAL	L_main6
;Click_Pressure2_STM.c,75 :: 		}
L_end_main:
L__main_end_loop:
B	L__main_end_loop
; end of _main
