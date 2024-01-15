;===============================================================================
	.title	Aurora 18x18 rev.2d
;
;		by Akimitsu Sadoi - www.theLEDart.com
;-------------------------------------------------------------------------------
;	Version history
;	1.0	ported from P24FV16KA302 version (03/11/2012)
;	1.1	Analog mode (audio interface) improvement
;	1.2 universal source code for Aurora 9x18 mk2 and 18x18
;===============================================================================

	.include "P24FV16KA304.INC"
	.list	b=4
	.include "SONY_TV.INC"	; Sony TV remote codes

;===============================================================================
;	constants
;
	.global __reset
	.global __T2Interrupt				; Timer2 ISR entry
	.global __T5Interrupt				; Timer5 ISR entry - auto-shutoff
	.global __CNInterrupt				; IR (CN) ISR entry
;-------------------------------------------------------------------------------
;
	.equ	osc_adjust,	0				; value to calibrate internal oscillator (-32 ~ +31)

	.equ	speed_step,0x8000/8			; speed adjust step value
	.equ	speed_norm,0x7FFE			; speed adjust center/normal value

	.equ	max_bor, 10					; maximum number of BOR allowed in series

	.equ	num_modes,(mode_tbl_end-mode_tbl)/2

	;--- auto-shutoff timer setting ------------------------
	.equ	shutoff_time, 0				; shutoff time in minutes (max:1,145 (about 19 Hours))
.if (shutoff_time != 0)
	.equ	timer_ticks, (shutoff_time*3750000)
	.equ	TIMER4_VAL, (timer_ticks & 0xFFFF)
	.equ	TIMER5_VAL, (timer_ticks / 0x10000)
	.equ	T4CON_VAL, ((1<<TON)+(1<<TCKPS1)+(1<<TCKPS0)+(1<<T32))
.endif
	;--- PWM timing parameters -----------------------------
	; LED refresh time = (pr_value+1)*(3*127+1)*Tcy = (pr_value+1)*382*Tcy
	;   optimized for video @225 Hz

	.equ	max_duty, 0xFF				; duty cycle value for 100% duty (8 bit)
	.equ	port_delay,	46				; delay time (in Tcy) between timer INT and port set (compensates for LED/Column drivers' fall time)
	.equ	port_delay_comp, 0			; pulse rinse time compensation value
	.equ	port_blank, 72				; blank period before start of the pulse (compensates for RGB drivers' fall time)
	.equ	pr_value, 186

	; Output Compare - double compare single-shot mode, system clock
	.equ	OCCON_VAL,(1<<OCM2+1<<OCTSEL0+1<<OCTSEL1+1<<OCTSEL2)
	; Output Compare - inverted output, sync to timer 2
	.equ	OCCON2_VAL,(1<<12+1<<SYNCSEL2+1<<SYNCSEL3)

	;--- switch parameters ---------------------------------
	.equ	debounce_time, 16			; (up to 16) x 2.048 mS
	.equ	debounce_bits, (1<<debounce_time-1)
	.equ	long_push_time, 240			; x 2.048 mS

	;--- IR receiver parameters ----------------------------
	.equ	start_bit,		2000*16		; minimum duration of start bit (in micro second)
	.equ	bit_threshold,	900*16		; threshold value used to determin the bit value(long/short pulse)(in micro second)
	.equ	accept_dev,	device_sony_tv
	.equ	IR_timeout_pr,	500/2		; timeout period for key repeat (in milliseconds)

	;--- port & pin mapping for I/O ----------------------------------

	; switch 1 is connected to RA1/CN3
	.equ	SW1_PORT, PORTA
	.equ	SW1_TRIS, TRISA
	.equ	SW1_ANS, ANSELA				; not ANSx as the manual states
	.equ	SW1_BIT, 1
	.equ	SW1_CN, 3
	; IR receiver is connected to RA9/CN34
	.equ	IR_PORT, PORTA
	.equ	IR_TRIS, TRISA
	.equ	IR_ANS, ANSELA				; not ANSx as the manual states
	.equ	IR_BIT, 9
	.equ	IR_CN, 34
	; analog input - RA0/AN0/CN2
	.equ	AN_IN_PORT, PORTA
	.equ	AN_IN_TRIS, TRISA
	.equ	AN_ANS, ANSELA				; not ANSx as the manual states
	.equ	AN_IN_BIT, 0
	.equ	AN_IN_CN, 2
	.equ	AN_IN_APIN, 0				; analog input # - sometimes different from pin #

.ifdef __DEBUG
	.equ	_debug_tris,TRISB
	.equ	_debug_port,LATB
	.equ	_debug_out,1
.endif

	;--- COL & ROW drive parameters ------------------------
	.equ	COL_POL, 1					; 0:active-low 1:active-high
	.equ	ROW_POL, 0					; 0:active-low 1:active-high
	
	.equ	num_COLs, 18				; number of COLs
	.equ	num_ROWs, 1					; number of RGB ROWs
	.equ	num_LEDs, num_COLs*num_ROWs	; number of LEDs

	;--- LEDs ----------------------------------------------
	.equ	LED_1_PORT,_RC_
	.equ	LED_1_PIN,1

	.equ	LED_2_PORT,_RC_
	.equ	LED_2_PIN,0

	.equ	LED_3_PORT,_RB_
	.equ	LED_3_PIN,3

	.equ	LED_4_PORT,_RB_
	.equ	LED_4_PIN,2

	.equ	LED_5_PORT,_RB_
	.equ	LED_5_PIN,15

	.equ	LED_6_PORT,_RB_
	.equ	LED_6_PIN,14

	.equ	LED_7_PORT,_RB_
	.equ	LED_7_PIN,11

	.equ	LED_8_PORT,_RB_
	.equ	LED_8_PIN,10

	.equ	LED_9_PORT,_RC_
	.equ	LED_9_PIN,9

	.equ	LED_10_PORT,_RC_
	.equ	LED_10_PIN,7

	.equ	LED_11_PORT,_RC_
	.equ	LED_11_PIN,6

	.equ	LED_12_PORT,_RB_
	.equ	LED_12_PIN,9

	.equ	LED_13_PORT,_RB_
	.equ	LED_13_PIN,8

	.equ	LED_14_PORT,_RC_
	.equ	LED_14_PIN,5

	.equ	LED_15_PORT,_RC_
	.equ	LED_15_PIN,4

	.equ	LED_16_PORT,_RC_
	.equ	LED_16_PIN,3

	.equ	LED_17_PORT,_RB_
	.equ	LED_17_PIN,4

	.equ	LED_18_PORT,_RC_
	.equ	LED_18_PIN,2

	.equ	PORTA_bits, 0				; highest port # used + 1
	.equ	PORTB_bits, 16
	.equ	PORTC_bits, 10

	;--- PWM R/G/B ports ---
	.equ	PWM_R_LAT,LATB
	.equ	PWM_R_PIN,7		; OC1/RB7
	.equ	PWM_R_OC,OC1RS
	.equ	PWM_R_OCC,OC1CON1

	.equ	PWM_G_LAT,LATC
	.equ	PWM_G_PIN,8		; OC2/RC8
	.equ	PWM_G_OC,OC2RS
	.equ	PWM_G_OCC,OC2CON1

	.equ	PWM_B_LAT,LATA
	.equ	PWM_B_PIN,10	; OC3/RA10
	.equ	PWM_B_OC,OC3RS
	.equ	PWM_B_OCC,OC3CON1

	.equ	_RA_,0
	.equ	_RB_,(PORTA_bits*2)
	.equ	_RC_,((PORTA_bits+PORTB_bits)*2)

	.text
	;--- LED data -> duty_buff offset lookup table ---------
LED_pins:
	.byte	LED_1_PIN*2+LED_1_PORT
	.byte	LED_2_PIN*2+LED_2_PORT
	.byte	LED_3_PIN*2+LED_3_PORT
	.byte	LED_4_PIN*2+LED_4_PORT
	.byte	LED_5_PIN*2+LED_5_PORT
	.byte	LED_6_PIN*2+LED_6_PORT
	.byte	LED_7_PIN*2+LED_7_PORT
	.byte	LED_8_PIN*2+LED_8_PORT
	.byte	LED_9_PIN*2+LED_9_PORT
	.byte	LED_10_PIN*2+LED_10_PORT
	.byte	LED_11_PIN*2+LED_11_PORT
	.byte	LED_12_PIN*2+LED_12_PORT
	.byte	LED_13_PIN*2+LED_13_PORT
	.byte	LED_14_PIN*2+LED_14_PORT
	.byte	LED_15_PIN*2+LED_15_PORT
	.byte	LED_16_PIN*2+LED_16_PORT
	.byte	LED_17_PIN*2+LED_17_PORT
	.byte	LED_18_PIN*2+LED_18_PORT

;===============================================================================

	.include "aurora_main.s"	; Aurora main codes

;===============================================================================

	.end
