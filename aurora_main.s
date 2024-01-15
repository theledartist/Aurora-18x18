;===============================================================================
	.title	Aurora main engine
;
;		by Akimitsu Sadoi - www.theLEDart.com
;-------------------------------------------------------------------------------
;	Version history
;	1.0	(04/12/2012)
;	1.1 audio interface detection at the startup added
;	1.2	PWM pulse inhibited when no LEDs are lit
;	1.3 IR remote receive service streamlined, and added data verification
;	1.4 speed adjustment kept in EEPROM
;===============================================================================

;===============================================================================
;	constants

;--- EEPROM location (relative) for parameter save ---
	.equ	mode_save, 0			; mode number
	.equ	speed_save, 2			; speed adjustment value

;===============================================================================
;	variables

	.data

mode_num:	.word 0					; mode (movment) number
bor_count:	.word 0					; BOR counter

var_start:							; top of variables to initialize

;--- global flags ----------------------
IR_flags:
g_flags:	.space 2
	do_not_read = 0					; LED data being modified - do not display
	HSV_mode = 1					; HSV color mode
	RND_mode = 2					; random mode
	analog = 3						; analog mode
	an_peak = 4						; analog peak detected
	;--- IR flags ----------------------
	IR_RCV = 5						; IR deta being received
	IR_RCD = 7						; IR received flag
	;--- LED dimming -------------------
	dimmed1 = 8
	dimmed2 = 9
	;--- audio interface ---------------
	ai_detected = 10
	;--- other ---
	no_LED = 11						; no LEDs are on - inhibit PWM pulse out


;--- IR receiver -----------------------
IR_keycode:	.word 0					; IR key code received
IR_device:	.word 0					; IR device code received
_IR_data:	.word 0					; IR command being received (internal)
IR_timeout:	.word 0					; IR command timeout counter
_IR_last_data:	.word 0xFF			; last data received (internal)
IR_last_key:	.word 0xFF			; last key received (needs to be initialized with 0xFF)

;--- analog data -----------------------
.equ	ana_devbit1,3
.equ	ana_samples1,1<<ana_devbit1		; number of samples to average
.equ	ana_devbit2,5
.equ	ana_samples2,1<<ana_devbit1
analog_buf1:	.space 2*ana_samples1	; analog input value buffer
analog_ptr1:	.word 0					; buffer pointer
analog_buf2:	.space 2*ana_samples2	; analog input value buffer
analog_ptr2:	.word 0					; buffer pointer
analog_val1:	.space 2			; analog input value with slow decay - slowest
analog_val2:	.space 2			; analog input value with slow decay
analog_val3:	.space 2			; analog input value with slow decay - fastest
analog_avg1:	.space 2
analog_avg2:	.space 2
an_peak_time:	.space 2			; peak detection 
an_peak_cnt:	.space 2
an_offset:		.space 2

;--- button data -----------------------
btn_data:							; button sample data
	button_flags = 0				; button flags
		btn_down = 0				; button is held down
		btn_long = 1				; long push
		btn_push = 2				; button has been pushed (long or short)
	samples = 2						; button status samples
	long_push = 4					; button long push counter

btnA_data:	.space 6

;--- animation parameters --------------
update_rate:	.word 0				; overall animation speed
update_cnt:		.word 0				; keeps track of animation update timing

anim_params:
	anim_delay = 0
	anim_duty_diff = 2
	anim_step_up = 4
	anim_step_down = 6
	anim_update_rate = 8
	anim_update = 10
	anim_max_duty = 12
	anim_release_waitH = 14		; duty_hold at high duty will be released after this wait
	anim_release_waitL = 16		; duty_hold at low duty will be released after this wait
	anim_flags = 18
		af_direction = 0		; 0=forward, 1=reverse
		af_duty_hold_rs = 1		; DUTY_HOLD_RS
	anim_release_cnt = 20		; DUTY_HOLD release counter for each LED

anim_params_R:
delay_R:		.word 0
duty_diff_R:	.word 0
step_up_R:		.word 0
step_down_R:	.word 0
update_rate_R:	.word 0
update_R:		.word 0
max_duty_R:		.word 0
release_waitH_R:	.word 0
release_waitL_R:	.word 0
anim_flags_R:	.word 0
release_cnt_R:	.space 2*num_LEDs

anim_params_G:
delay_G:		.word 0
duty_diff_G:	.word 0
step_up_G:		.word 0
step_down_G:	.word 0
update_rate_G:	.word 0
update_G:		.word 0
max_duty_G:		.word 0
release_waitH_G:	.word 0
release_waitL_G:	.word 0
anim_flags_G:	.word 0
release_cnt_G:	.space 2*num_LEDs

anim_params_B:
delay_B:		.word 0
duty_diff_B:	.word 0
step_up_B:		.word 0
step_down_B:	.word 0
update_rate_B:	.word 0
update_B:		.word 0
max_duty_B:		.word 0
release_waitH_B:	.word 0
release_waitL_B:	.word 0
anim_flags_B:	.word 0
release_cnt_B:	.space 2*num_LEDs

preprocess:		.space 2			; subroutine to call before animation process


;--- LED data - duty levels, and other parameters/flags ---
LED_data:
	DUTY_DIR = 14					; PWN duty sweep up/down direction (1=down)
	DUTY_HOLD = 15
LED_data_R:
LED_data_R1:	.space 2*num_LEDs

LED_data_G:
LED_data_G1:	.space 2*num_LEDs

LED_data_B:
LED_data_B1:	.space 2*num_LEDs

;--- HSV data buffer -------------------
LED_data_H:		.space 2*num_LEDs
LED_data_S:		.space 2*num_LEDs
LED_data_V:		.space 2*num_LEDs

;--- for LED refresh/interrupt ---------
RGB:	.word 0						; RGB channel counter
output_duty:	.word 0				; PWM duty level being output
pulse_duration: .word 0				; pulse duration for the PWM level

duty_buff:							; LED data buffers
duty_buff_R:	.space 2*(PORTA_bits+PORTB_bits+PORTC_bits)
duty_buff_G:	.space 2*(PORTA_bits+PORTB_bits+PORTC_bits)
duty_buff_B:	.space 2*(PORTA_bits+PORTB_bits+PORTC_bits)

port_buff:		.space 6			; PORTA/PORTB/PORTC buffers

;--- RGB -> HSV conversion temporary variables ---
var_I:			.word 0
var_H:			.word 0
var_1:			.word 0
var_2:			.word 0
var_3:			.word 0

end_of_vars:							; end of variables area to initialize

;===============================================================================
;	main

	.text
__reset:
				bclr	CLKDIV,#RCDIV0				; set FRC postscaler to 0 (8 MHz)

				mov		#osc_adjust,w0				; adjust internal oscillator
				mov		w0,OSCTUN

				mov		#__SP_init,w15				; Initialize stack pointer and stack limit register,
				mov		#__SPLIM_init,w0
				mov		w0,SPLIM					;
				nop									; A NOP here is required for proper SPLIM functionality

				;--- configure interrupt, etc. -----------------------
				bset	INTCON1,#NSTDIS				; disable nested interrupts

				bset	CORCON,#PSV					; enable PSV

				;--- pause for apox. 250 ms --------------------------
				clr		w0
0:				repeat	#60
				clrwdt
				dec		w0,w0
				bra		NZ,0b
.if 0
				;--- check if brown-out reset ------------------------
				btss	RCON,#BOR
				bra		0f
bor_start:
				bclr	RCON,#BOR
				inc		bor_count					; count BOR
				mov		#max_bor,w0
				cp		bor_count
				bra		GEU,power_down				; power down if too many BOR occurred
0:
.endif
				;--- check if cold or hot start ----------------------
				btss	RCON,#POR
				bra		hot_start
cold_start:
				bclr	RCON,#POR
				;--- clear RAM -----------------------------
				mov		#var_start,w0				; make sure that the value is even number
				disi	#(end_of_vars-var_start)*2
				repeat	#(end_of_vars-var_start)/2-1
				clr		[w0++]						; or trap will result
hot_start:

				;--- configure I/O ports -----------------------------
.if (COL_POL)
				clr		LATA						; clear all latches (LEDs are active-high)
				clr		LATB
.if (PORTC_bits)
				clr		LATC
.endif
.else
				setm	LATA						; set all latches high (LEDs are active-low)
				setm	LATB
.if (PORTC_bits)
				setm	LATC
.endif
.endif
.if (ROW_POL)
				bclr	PWM_R_LAT,#PWM_R_PIN		; set PWM pins low (inactive)
				bclr	PWM_G_LAT,#PWM_G_PIN
				bclr	PWM_B_LAT,#PWM_B_PIN
.else
				bset	PWM_R_LAT,#PWM_R_PIN		; set PWM pins high (inactive)
				bset	PWM_G_LAT,#PWM_G_PIN
				bset	PWM_B_LAT,#PWM_B_PIN
.endif
				clr		TRISA						; set all ports output
				clr		TRISB
.if (PORTC_bits)
				clr		TRISC
.endif
				;--- detect audio interface ----------------
				bclr	g_flags,#ai_detected		; clear the flag bit
				bset	AN_IN_TRIS,#AN_IN_BIT		; set AN-IN input
				bclr	AN_ANS,#AN_IN_BIT			; set AN-IN digital mode
				bset	CNPD1+(AN_IN_CN/16)*2,#(AN_IN_CN%16) ; enable the pull-down
				repeat	#127							; wait for a moment
				clrwdt
				btsc	AN_IN_PORT,#AN_IN_BIT		; if port is high
				bset	g_flags,#ai_detected		;   set the flag bit
				bclr	CNPD1+(AN_IN_CN/16)*2,#(AN_IN_CN%16) ; disable the pull-down

				bset	CNPU1+(AN_IN_CN/16)*2,#(AN_IN_CN%16) ; enable the pull-up
				repeat	#127							; wait for a moment
				clrwdt
				btsc	AN_IN_PORT,#AN_IN_BIT		; if port is high
				btg		g_flags,#ai_detected		;   toggle the flag bit
				bclr	CNPU1+(AN_IN_CN/16)*2,#(AN_IN_CN%16) ; disable the pull-up

				btg		g_flags,#ai_detected		; toggle the flag bit again

				;--- configure SW input --------------------
				bset	SW1_TRIS,#SW1_BIT			; set the switch port input
				bclr	SW1_ANS,#SW1_BIT			; set digital mode
				bset	CNPU1+(SW1_CN/16)*2,#(SW1_CN%16) ; enable the pull-ups

				;--- configure IR receiver input -----------
				bset	IR_TRIS,#IR_BIT				; set the IR port input
				bclr	IR_ANS,#IR_BIT				; set digital mode
				bset	CNPU1+(IR_CN/16)*2,#(IR_CN%16) ; enable the pull-ups
				repeat	#16							; wait for a moment for the port to settle
				clrwdt
				bset	CNEN1+(IR_CN/16)*2,#(IR_CN%16) ; enable CN interrupt on IR receiver pin
				bset	IEC1,#CNIE					; enable CN interrupt

				;--- configure analog input (ADC) ----------
				btss	g_flags,#ai_detected
				bra		0f
				bset	AN_IN_TRIS,#AN_IN_BIT		; set AN-IN input
				bset	AN_ANS,#AN_IN_BIT			; set AN-IN analog mode
				bset	CNPD1+(AN_IN_CN/16)*2,#(AN_IN_CN%16) ; enable the pull-down
				mov		#AN_IN_APIN,w0				; set AN_IN_APIN as CH0 input
				mov		w0,AD1CHS

				mov		#(1<<SSRC2+1<<SSRC1+1<<SSRC0),w0 ; auto-convert mode
				mov		w0,AD1CON1

				mov		#((31<<8)+22),w0			; sample time = 31 TAD, TAD = 23 Tcy
				mov		w0,AD1CON3

				bset	AD1CON1,#10					; activate 12 bit mode

				bset	AD1CON1,#ADON				; turn ADC on
				bset	AD1CON1,#SAMP				; start sampling analog input
0:
.ifdef __DEBUG
				bclr	_debug_tris,#_debug_out		; *DEBUG* set debug out
.endif
				;--- configure Output Compare ------------------------
				mov		#port_delay-port_delay_comp,w0
				mov		w0,OC1R						; set pulse start time
				mov		w0,OC2R					
				mov		w0,OC3R					
				mov		#OCCON2_VAL,w0				; set other parameters
				mov		w0,OC1CON2
				mov		w0,OC2CON2
				mov		w0,OC3CON2

				;--- configure timer2 for PWM service ----------------
				mov		#pr_value,w0				; set the time period
				mov		w0,PR2
				bset	T2CON,#TON					; start timer2

0:				clrwdt
				btss	SW1_PORT,#SW1_BIT			; wait until the button up
				bra		0b

;-------------------------------------------------------------------------------
;
startup:
.if (shutoff_time != 0)
				;--- set up TIMER4/5 for auto shut off ---
				mov		#TIMER4_VAL,w0
				mov		w0,PR4
				mov		#TIMER5_VAL,w0
				mov		w0,PR5
				clr		TMR4
				clr		TMR5
				mov		#T4CON_VAL,w0
				mov		w0,T4CON
				
				bclr	IFS1,#T5IF					; clear timer5 int flag
				bset	IEC1,#T5IE					; enable timer5 int
.endif
				;--- use TIMER1 as frame timebase --------------------
				mov		#0x7FFF,w0					; set PR1 = 0x7FFF
				mov		w0,PR1						; timer period = 2.048 mS
				bset	T1CON,#TON					; start timer 1

				;--- reset anim params ---------------------
				mov		#anim_params,w0				; make sure that the value is even number
				disi	#(end_of_vars-(anim_params))*2
				repeat	#(end_of_vars-(anim_params))/2-1
				clr		[w0++]						; or trap will result

				bclr	g_flags,#do_not_read		; clear do_not_read flag
				bclr	g_flags,#HSV_mode			; clear HSV mode flag
				bclr	g_flags,#analog				; clear analog mode flag
				bclr	g_flags,#no_LED				; clear no_LED flag

				;--- set default anim_parameters -----------
				; set max_duty
				mov		#max_duty,w0
				mov		w0,anim_params_R+anim_max_duty
				mov		w0,anim_params_G+anim_max_duty
				mov		w0,anim_params_B+anim_max_duty

				;--- read EEPROM to restore previous mode ---
				mov		#mode_save,w1				; read location -> w1
				call	eeprom_read
				mov		w0,mode_num

				;--- read EEPROM to restore previous speed adjust ---
				mov		#speed_save,w1				; read location -> w1
				call	eeprom_read
				com		w0,w1						; if (w0 != 0xFFFF)
				bra		Z,0f						; {
				mov		w0,update_rate				;   set update_rate
				bra		1f							; } else {
0:				mov		#speed_norm,w0				;   set update_rate to the midway point
				mov		w0,update_rate				; }
1:
				;--- keep mode_num within range ---
				mov		(#mode_tbl_end-#mode_tbl)/2-1,w0	; num_modes-1
				cp		mode_num
				bra		LEU,1f
				cp0		mode_num
				bra		GE,0f						; if (mode_num < 0)
				mov		w0,mode_num					;   mode_num = num_modes-1
				bra		1f							; if (mode_num >= num_modes-1)
0:				clr		mode_num					;   mode_num = 0
1:				mov		mode_num,w0
				bra		w0
mode_tbl:
				bra		mode_01
				bra		mode_02
				bra		mode_03
				bra		mode_04
				bra		mode_05
				bra		mode_06
				bra		mode_07
				bra		test_01
				bra		audio_01
				bra		audio_02
mode_tbl_end:


mode_01:		;---------------------------------
				; half ripple - pastel (Aurora 9x18 classic)

				; set initial delay
				mov		#0,w0
				mov		w0,anim_params_R+anim_delay
				mov		#num_LEDs*8,w0
				mov		w0,anim_params_G+anim_delay
				mov		#num_LEDs*16,w0
				mov		w0,anim_params_B+anim_delay

				; set update_rate
				mov		#0xBFFF-197,w0
				mov		w0,anim_params_R+anim_update_rate
;				mov		#0xBFFF,w0
				mov		w0,anim_params_G+anim_update_rate
;				mov		#0xBFFF+199,w0
				mov		w0,anim_params_B+anim_update_rate

				; set step_up
				mov		#1,w0
				mov		w0,anim_params_R+anim_step_up
				mov		w0,anim_params_G+anim_step_up
				mov		w0,anim_params_B+anim_step_up
				; set step_down
				mov		w0,anim_params_R+anim_step_down
				mov		w0,anim_params_G+anim_step_down
				mov		w0,anim_params_B+anim_step_down

				; set duty_diff
				mov		#255/num_LEDs,w0
				mov		w0,anim_params_R+anim_duty_diff
				mov		w0,anim_params_G+anim_duty_diff
				mov		w0,anim_params_B+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data,w0
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				bclr	LED_data_R,#DUTY_HOLD
				bclr	LED_data_G,#DUTY_HOLD
				bclr	LED_data_B,#DUTY_HOLD

				bra		main

mode_02:		;-------------------------------------------
				; half ripple - trianguler

				; set initial delay
				mov		#0,w0
				mov		w0,anim_params_R+anim_delay
				mov		#num_LEDs*18,w0
				mov		w0,anim_params_G+anim_delay
				mov		#num_LEDs*36,w0
				mov		w0,anim_params_B+anim_delay

				; set update_rate
				mov		#0xFFFF/2,w0
				mov		w0,anim_params_R+anim_update_rate
				mov		w0,anim_params_G+anim_update_rate
				mov		w0,anim_params_B+anim_update_rate

				; set step_up
				mov		#1,w0
				mov		w0,anim_params_R+anim_step_up
				mov		w0,anim_params_G+anim_step_up
				mov		w0,anim_params_B+anim_step_up
				; set step_down
				mov		#2,w0
				mov		w0,anim_params_R+anim_step_down
				mov		w0,anim_params_G+anim_step_down
				mov		w0,anim_params_B+anim_step_down

				; set duty_diff
				mov		#255/num_LEDs*2/3,w0
				mov		w0,anim_params_R+anim_duty_diff
				mov		w0,anim_params_G+anim_duty_diff
				mov		w0,anim_params_B+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data,w0
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				bclr	LED_data_R,#DUTY_HOLD
				bclr	LED_data_G,#DUTY_HOLD
				bclr	LED_data_B,#DUTY_HOLD

				bra		main

				;-------------------------------------------
				; full ripple - rainbow
mode_04:
				bset	anim_flags_R,#af_direction	; reverse direction - counter clockwise
				bset	anim_flags_G,#af_direction	; reverse direction - counter clockwise
				bset	anim_flags_B,#af_direction	; reverse direction - counter clockwise
mode_03:
				; set update_rate
				mov		#0xFFFF*50/100,w0
				mov		w0,anim_params_R+anim_update_rate
				mov		#0xFFFF*50/100,w0
				mov		w0,anim_params_G+anim_update_rate
				mov		#0xFFFF*50/100,w0
				mov		w0,anim_params_B+anim_update_rate

				; set step_up
				mov		#1,w0
				mov		w0,anim_params_R+anim_step_up
				mov		w0,anim_params_G+anim_step_up
				mov		w0,anim_params_B+anim_step_up
				; set step_down
				mov		w0,anim_params_R+anim_step_down
				mov		w0,anim_params_G+anim_step_down
				mov		w0,anim_params_B+anim_step_down

				; set duty_diff
				mov		#255*2/num_LEDs,w0
				mov		w0,anim_params_R+anim_duty_diff
				mov		w0,anim_params_G+anim_duty_diff
				mov		w0,anim_params_B+anim_duty_diff

				; set all DUTY_HOLD bits
				mov		#LED_data,w0
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				; clear DUTY_HOLD bits
				bclr	LED_data_R,#DUTY_HOLD
				bclr	LED_data_G+(2*num_LEDs/3),#DUTY_HOLD
				bclr	LED_data_B+(2*num_LEDs*2/3),#DUTY_HOLD

				bra		main

				;-------------------------------------------
				; HSV mode chase
				; R->H, G->S, B->V
mode_06:
				bset	anim_flags_B,#af_direction	; reverse direction - counter clockwise
mode_05:
				bset	g_flags,#HSV_mode			; HSV mode flag on

				; set max_duty*6 for H channel
				mov		#max_duty*6,w0
				mov		w0,anim_params_R+anim_max_duty

				; set initial delay
				mov		#0,w0
				mov		w0,delay_R
				mov		w0,delay_G
				mov		w0,delay_B

				; set update_rate
				mov		#0x1000,w0
				mov		w0,update_rate_R
				mov		#0xFFFF,w0
				mov		w0,update_rate_G
				mov		#0xFFFF,w0
				mov		w0,update_rate_B

				; set step_up
				mov		#1,w0
				mov		w0,step_up_R
				mov		#255,w0
				mov		w0,step_up_G
				mov		#256/8,w0
				mov		w0,step_up_B
				; set step_down
				mov		#max_duty*6,w0
				mov		w0,step_down_R
				mov		#0,w0
				mov		w0,step_down_G
				mov		#1,w0
				mov		w0,step_down_B

				; set duty_diff
				mov		#1,w0
				mov		w0,duty_diff_R
				mov		#0,w0
				mov		w0,duty_diff_G
				mov		#255/num_LEDs,w0
				mov		w0,duty_diff_B

				; set all DUTY_HOLD bits
				mov		#LED_data_H,w0
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				bclr	LED_data_H,#DUTY_HOLD
				bclr	LED_data_S,#DUTY_HOLD
				bclr	LED_data_V,#DUTY_HOLD

				; set DUTY_HOLD_RS bits
				bset	anim_flags_B,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,release_waitH_B
				mov		#0,w0
				mov		w0,release_waitL_B

				bra		main

mode_07:		;-------------------------------------------
				; HSV mode - color change only
				; R->H, G->S, B->V

				bset	g_flags,#HSV_mode			; HSV mode flag on

				; set max_duty*6 for H channel
				mov		#(max_duty+1)*6,w0
				mov		w0,anim_params_R+anim_max_duty

				; set initial delay
				mov		#0,w0
				mov		w0,delay_R
				mov		w0,delay_G
				mov		w0,delay_B

				; set update_rate
				mov		#0x0FFF,w0
				mov		w0,update_rate_R
				mov		#0xFFFF,w0
				mov		w0,update_rate_G
				mov		#0xFFFF,w0
				mov		w0,update_rate_B

				; set step_up
				mov		#1,w0
				mov		w0,step_up_R
				mov		#255,w0
				mov		w0,step_up_G
				mov		#1,w0
				mov		w0,step_up_B
				; set step_down
				mov		#(max_duty+1)*6,w0
				mov		w0,step_down_R
				mov		#0,w0
				mov		w0,step_down_G
				mov		w0,step_down_B

				; set duty_diff
				mov		#4,w0
				mov		w0,duty_diff_R

				; set all DUTY_HOLD bits
				mov		#LED_data_H,w0
				repeat	#num_LEDs-1
				bset	[w0++],#DUTY_HOLD

				bclr	LED_data_H,#DUTY_HOLD

				bra		main

test_01:		;-------------------------------------------
				; test RGB - chase

				; set initial delay
				mov		#0,w0
				mov		w0,delay_R
				mov		#256*2,w0
				mov		w0,delay_G
				mov		#256*4,w0
				mov		w0,delay_B

				; set update_rate -> max
				mov		#0xFFFF,w0
				mov		w0,update_rate_R
				mov		w0,update_rate_G
				mov		w0,update_rate_B

				; set step_up
				mov		#1,w0
				mov		w0,step_up_R
				mov		w0,step_up_G
				mov		w0,step_up_B
				; set step_down
				mov		w0,step_down_R
				mov		w0,step_down_G
				mov		w0,step_down_B

				; set duty_diff
				mov		#0xFF/num_LEDs,w0
				mov		w0,duty_diff_R
				mov		w0,duty_diff_G
				mov		w0,duty_diff_B

				; set all DUTY_HOLD bits
				mov		#LED_data,w0
				repeat	#num_LEDs*3-1
				bset	[w0++],#DUTY_HOLD

				bclr	LED_data_R1,#DUTY_HOLD
				bclr	LED_data_G1,#DUTY_HOLD
				bclr	LED_data_B1,#DUTY_HOLD

				; set all DUTY_HOLD_RS bits
				bset	anim_flags_R,#af_duty_hold_rs
				bset	anim_flags_G,#af_duty_hold_rs
				bset	anim_flags_B,#af_duty_hold_rs

				; set release_wait
				mov		#1,w0
				mov		w0,release_waitH_R
				mov		w0,release_waitH_G
				mov		w0,release_waitH_B
				mov		#0xFF*4,w0
				mov		w0,release_waitL_R
				mov		w0,release_waitL_G
				mov		w0,release_waitL_B

				bra		main

audio_01:		;-------------------------------------------
				; audio mode 1

				btsc	g_flags,#ai_detected		; skip if audio interface not detacted
				bra		0f
				clr		mode_num
				bra		mode_change
0:
				bset	g_flags,#analog				; set analog mode flag
				mov		#handle(analog_LED3),w0		; set analog_LED3 as preprocessor
				mov		w0,preprocess

				bra		main

audio_02:		;-------------------------------------------
				; audio mode 2

				btsc	g_flags,#ai_detected		; skip if audio interface not detacted
				bra		0f
				clr		mode_num
				bra		mode_change
0:
				bset	g_flags,#analog				; set analog mode flag
				bset	g_flags,#HSV_mode			; HSV mode flag on
				mov		#handle(analog_LED2),w0		; set analog_LED2 as preprocessor
				mov		w0,preprocess

				bra		main

				;-------------------------------------------
				; power down
__T5Interrupt:
power_down:
				clr		bor_count					; clear BOR count
				clrwdt
				bclr	RCON,#SWDTEN				; disable WDT
				bclr	T1CON,#TON					; stop timer 1
				bclr	T2CON,#TON					; stop timer 2
				bclr	IEC0,#T2IE					; disable timer2 int
				bclr	IFS0,#T2IF					; clear timer2 int flag
				bclr	IEC1,#T5IE					; disable timer5 int
				bclr	IFS1,#T5IF					; clear timer5 int flag

				setm	TRISA						; set all ports input/high-impedance
				setm	TRISB
.if (PORTC_bits)
				setm	TRISC
.endif
0:				btss	SW1_PORT,#SW1_BIT			; wait until the button up
				bra		0b

				clr		w0							; pause for a bit to avoid SW chatter
0:				repeat	#127
				clrwdt
				dec		w0,w0
				bra		NZ,0b

1:				btss	IR_PORT,#IR_BIT				; wait until the IR sensor inactive
				bra		1b
				bclr	IR_flags,#IR_RCD			; clear IR received flag

				bclr	IFS1,#CNIF					; clear CN int flag
				bset	CNEN1+(SW1_CN/16)*2,#(SW1_CN%16) ; enable CN interrupt on SW1
				bset	IEC1,#CNIE					;
				pwrsav	#SLEEP_MODE					; go into sleep mode

				;--- wake from sleep -----------------------
				btss	SW1_PORT,#SW1_BIT			; is the button down?
				bra		wake_up						;   then wake up
													; otherwise wait for a moment to receive IR command
				;--- use TIMER1 as timeout timer ---
				mov		#0x7FFF,w0					; set PR1 = 0x7FFF
				mov		w0,PR1						; timer period = 2.048 ms
				bclr	IFS0,#T1IF					; clear timer 1 int flag
				bset	T1CON,#TON					; start timer 1

				mov		#IR_timeout_pr,w0			; IR_timeout = IR_timeout_pr
				mov		w0,IR_timeout
0:
				btsc	IR_flags,#IR_RCD			; check if IR command received
				bra		verify_dev

				btss	IFS0,#T1IF					; wait for 2 ms
				bra		0b
				bclr	IFS0,#T1IF					; clear timer 1 int flag
				dec		IR_timeout					; decrement IR key timeout counter
				bra		NZ,0b						; loop while (IR_timeout != 0)
				bra		1b							; back to sleep if no IR command
verify_dev:
				mov		#accept_dev,w0				; verify the device ID
				cp		IR_device
				bra		NZ,1b						; back to sleep if not accepted device
wake_up:											; command received - wake up
				bset	RCON,#POR					; pretend cold start
				reset

;-----------------------------------------------------------------------------------------

main:
				;-------------------------------------------------------------------------
				clrwdt
				bset	RCON,#SWDTEN				; enable WDT

				bset	IEC0,#T2IE					; enable timer2 int for PWM

main_loop:		;--- main loop -----------------------------------------------------------
				btss	IFS0,#T1IF					; wait for TIMER1 overflow
				bra		main_loop
				bclr	IFS0,#T1IF					; clear T1IF

				;--- IR remote services ------------------------------
				dec		IR_timeout					; take care of IR key timeout
				btss	SR,#C
				setm	IR_last_key					; clear the last key upon timeout

				btss	IR_flags,#IR_RCD			; if IR received flag set
				bra		skip_IR						; {
				bclr	IR_flags,#IR_RCD			;   clear the flag

				mov		#accept_dev,w0				;   verify the device ID
				cp		IR_device
				bra		NZ,skip_IR

				mov		IR_keycode,w0				;   is this a different key from the last one?
				cp		IR_last_key
				bra		Z,skip_IR
													;   different key
				mov		w0,IR_last_key				;   keep the keycode to compare later
				mov		#IR_timeout_pr,w1			;   set the timeout counter
				mov		w1,IR_timeout

				;--- scan for the keys with functions ---
				mov		#psvpage(IR_key_table),w1	; set up PSV
				mov		w1,PSVPAG
				mov		#psvoffset(IR_key_table),w1
				mov		#psvoffset(IR_key_table_end),w2
0:				cp		w0,[w1++]					; find the key code in the table {
				bra		Z,IR_key_match				;   match found
				cp		w1,w2
				bra		NZ,0b						; } loop until the table end
				bra		skip_IR						; no match found
IR_key_match:
				mov		#psvoffset(IR_key_table),w2	; using w1-w2 as index
				sub		w1,w2,w1
				lsr		w1,w1
				dec		w1,w1
				bra		w1							; jump to the fuction

				bra		func_next_mode
				bra		func_next_mode
				bra		func_prev_mode
				bra		func_prev_mode
				bra		func_pause
				bra		func_inc_speed
				bra		func_inc_speed
				bra		func_dec_speed
				bra		func_dec_speed
				bra		power_down
				bra		power_down
				bra		func_jump					; 1
				bra		func_jump					; 2
				bra		func_jump					; 3
				bra		func_jump					; 4
				bra		func_jump					; 5
				bra		func_jump					; 6
				bra		func_jump					; 7
				bra		func_jump					; 8
				bra		func_jump					; 9
				bra		func_jump					; 0 (10)
				bra		func_dimm
				bra		func_reset
IR_key_table:	;-----------------------------------------------------
				.word	IR_key_up					; next mode
				.word	IR_key_Ch_up				; next mode
				.word	IR_key_down					; prev mode
				.word	IR_key_Ch_down				; prev mode
				.word	IR_key_center				; pause
				.word	IR_key_right				; increase speed
				.word	IR_key_Vol_up				; increase speed
				.word	IR_key_left					; decrease speed
				.word	IR_key_Vol_down				; decrease speed
				.word	IR_key_Power				; power off
				.word	IR_key_Sleep				; power off
				.word	IR_key_1					; jump to mode 1
				.word	IR_key_2					; jump to mode 2
				.word	IR_key_3					; jump to mode 3
				.word	IR_key_4
				.word	IR_key_5
				.word	IR_key_6
				.word	IR_key_7
				.word	IR_key_8
				.word	IR_key_9
				.word	IR_key_0
				.word	IR_key_Muting				; dim LEDs
				.word	IR_key_Reset				; reset functions
IR_key_table_end:	;-------------------------------------------------


func_next_mode:	;---------------------------------
				inc		mode_num					; go to next mode
				bra		mode_change

func_prev_mode:	;---------------------------------
				dec		mode_num					; go to prev mode
				bra		mode_change

func_pause:		;---------------------------------
				cp0		update_rate					; re-start if update_rate == 0
				bra		Z,1f
				clr		update_rate					; otherwise pause the animation
				bra		skip_IR
1:													; start the animation
													; set update_rate to the saved value
				mov		#speed_save,w1				; read location -> w1
				call	eeprom_read
				mov		w0,update_rate

				bra		skip_IR

func_inc_speed:	;---------------------------------
.if 0
				mov		#0xFFFF,w0
				cp		update_rate					; skip if the speed is already max
				bra		Z,skip_IR
				sl		update_rate					; update_rate <<= 1
				bset	update_rate,#0				; update_rate += 1
.else
				mov		#(0xFFFF-speed_step),w0
				cp		update_rate					; skip if the speed is already max
				bra		GTU,skip_IR
				mov		#speed_step,w0
				add		update_rate
.endif
				mov		update_rate,w0				; write the new value into EEPROM
				mov		#speed_save,w1				; location
				call	eeprom_write

				bra		skip_IR

func_dec_speed:	;---------------------------------
.if 0
				cp0		update_rate					; skip if the speed is already min
				bra		Z,skip_IR
				lsr		update_rate					; update_rate >>= 1
.else
				mov		#speed_step,w0
				cp		update_rate					; skip if the speed is already min
				bra		LEU,skip_IR
				sub		update_rate
.endif
				mov		update_rate,w0				; write the new value into EEPROM
				mov		#speed_save,w1				; location
				call	eeprom_write

				bra		skip_IR

func_jump:		;---------------------------------
				mov		w0,mode_num
				bra		mode_change

func_dimm:		;---------------------------------
				btg		g_flags,#dimmed1			; rotate between 01,11,00 (dimmed2 dimmed1)
				btss	g_flags,#dimmed1
				btg		g_flags,#dimmed2
				btsc	g_flags,#dimmed2
				btg		g_flags,#dimmed1
				bra		skip_IR

func_reset:		;---------------------------------
				mov		#speed_norm,w0				; set update_rate to the midway point
				mov		w0,update_rate				; write the new value into EEPROM
				mov		#speed_save,w1				; location -> w1
				call	eeprom_write

				bclr	g_flags,#dimmed1			; reset dimming level
				bclr	g_flags,#dimmed2

				clr		mode_num					; mode_num = 0
				bra		mode_change

				;---------------------------------
skip_IR:											; }


				;--- button services ---------------------------------
				call	button_check				; check the button states

				btsc	btnA_data,#btn_long			; if button long pushed
				bra		power_down					; power down

				btss	btnA_data,#btn_push			; if button pushed
				bra		0f
				bclr	btnA_data,#btn_push			; clear the button flag
				inc		mode_num					; go to next mode
mode_change:
				bclr	IEC0,#T2IE					; disable timer2 int for PWM
1:				clrwdt
				btss	IFS0,#T2IF					; wait until TMR2 overflow
				bra		1b

				mov		mode_num,w0					; write the new mode into EEPROM
				mov		#mode_save,w1				; location -> w1
				call	eeprom_write

				clrwdt
				bclr	RCON,#SWDTEN				; disable WDT

				bra		startup
0:

animate:		;--- animate LEDs ------------------------------------
				btss	g_flags,#analog				; if analog mode
				bra		0f
				call	analog_input				;   process A/D samples
				bra		1f							;   skip update_cnt (animate at the max rate)
0:
				mov		update_rate,w0
				add		update_cnt
				bra		NC,main_loop				; skip update if update_cnt <= 0xFFFF
1:
				cp0		preprocess					; if preprocess is set
				bra		Z,0f
				mov		preprocess,w0
				call	w0							;   do the preprocess
0:
				btsc	g_flags,#HSV_mode			; if HSV mode
				bra		animate_HSV

animate_RGB:
				bset	g_flags,#do_not_read		; disable LED data read

				mov		#anim_params_R,w1			; w1 -> anim_params_R
				mov		#LED_data_R,w2				; w2 -> LED_data_R
				call	animate_LED

				mov		#anim_params_G,w1			; w1 -> anim_params_G
				mov		#LED_data_G,w2				; w2 -> LED_data_G
				call	animate_LED

				mov		#anim_params_B,w1			; w1 -> anim_params_B
				mov		#LED_data_B,w2				; w2 -> LED_data_B
				call	animate_LED

				bclr	g_flags,#do_not_read		; enable LED data read

				bra		main_loop

animate_HSV:
				mov		#anim_params_R,w1			; w1 -> anim_params_R
				mov		#LED_data_H,w2				; w2 -> LED_data_H
				call	animate_LED

				mov		#anim_params_G,w1			; w1 -> anim_params_G
				mov		#LED_data_S,w2				; w2 -> LED_data_S
				call	animate_LED

				mov		#anim_params_B,w1			; w1 -> anim_params_B
				mov		#LED_data_V,w2				; w2 -> LED_data_V
				call	animate_LED

				call	convert_HSV

				bra		main_loop

;--- end of main loop --------------------------------------------------------------------

;-----------------------------------------------------------------------------------------

;-------------------------------------------------------------------------------
;	analog mode LED animation
;
analog_LED1:
.equ	adc_floor_R,264	; 266 low level threshold
.equ	adc_step_R,35*18/num_LEDs	; 35 steps per LED (smaller -> more sensitive)
.equ	adc_floor_G,287	; 287
.equ	adc_step_G,45*18/num_LEDs	; 45
.equ	adc_floor_B,404	; 404
.equ	adc_step_B,65*18/num_LEDs	; 65

				;--- R ---
				mov		#LED_data_R,w2				; w2 -> LED_data_R
				mov		#LED_data_R+(num_LEDs*2),w3	; w3 is loop stopper

				mov		#adc_floor_R-adc_step_R,w0	; value - (adc_floor - adc_step)
				sub		analog_val1,wreg			; use slowest decay
1:													; while (w2 < w3) {
				mov		#max_duty,w1
				sub		#adc_step_R,w0				; 	value - adc_step
				bra		NN,2f						; 	if (w0 < 0)
				add		w1,w0,w1					; 	  w1 = max_duty + w0
				bra		3f							; 	else
2:				sub		w1,w0,w1					; 	  w1 = max_duty - w0
3:				bra		NN,0f						; 	if (w1 < 0)
				clr		w1							; 	  w1 = 0
0:				mov		w1,[w2++]					; 	result -> LED_data
				cpseq	w2,w3
				bra		1b							; } end while

				;--- G ---
				mov		#LED_data_G+(num_LEDs*2),w3	; w3 is loop stopper

				mov		#adc_floor_G-adc_step_G,w0	; value - (adc_floor - adc_step)
				sub		analog_val2,wreg			; use mid decay
1:													; while (w2 < w3) {
				mov		#max_duty,w1
				sub		#adc_step_G,w0				; 	value - adc_step
				bra		NN,2f						; 	if (w0 < 0)
				add		w1,w0,w1					; 	  w1 = max_duty + w0
				bra		3f							; 	else
2:				sub		w1,w0,w1					; 	  w1 = max_duty - w0
3:				bra		NN,0f						; 	if (w1 < 0)
				clr		w1							; 	  w1 = 0
0:				mov		w1,[w2++]					; 	result -> LED_data
				cpseq	w2,w3
				bra		1b							; } end while

				;--- B ---
				mov		#LED_data_B+(num_LEDs*2),w3	; w3 is loop stopper

				mov		#adc_floor_B-adc_step_B,w0	; value - (adc_floor - adc_step)
				sub		analog_val3,wreg			; use fast decay
1:													; while (w2 < w3) {
				mov		#max_duty,w1
				sub		#adc_step_B,w0				; 	value - adc_step
				bra		NN,2f						; 	if (w0 < 0)
				add		w1,w0,w1					; 	  w1 = max_duty + w0
				bra		3f							; 	else
2:				mov		w1,w1						; 	  w1 = max_duty
;2:				sub		w1,w0,w1					; 	  w1 = max_duty - w0
3:				bra		NN,0f						; 	if (w1 < 0)
				clr		w1							; 	  w1 = 0
0:				mov		w1,[w2++]					; 	result -> LED_data
				cpseq	w2,w3
				bra		1b							; } end while

				return

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;	

analog_LED2:
.equ	adc_floor_V,264	; low level threshold
.equ	adc_step_V,35*18/num_LEDs	; steps per LED (smaller -> more sensitive)
.equ	adc_floor_H,300	; 
.equ	adc_step_H,10	; 
.equ	adc_floor_S,10	; 
.equ	adc_step_S,1	; 

				;--- V ---
				mov		#LED_data_V,w2				; w2 -> LED_data_V
				mov		#LED_data_V+(num_LEDs*2),w3	; w3 is loop stopper

				mov		#adc_floor_V-adc_step_V,w0	; value - (adc_floor - adc_step)
				sub		analog_val1,wreg			; use slowest decay
1:													; while (w2 < w3) {
				mov		#max_duty,w1
				sub		#adc_step_V,w0				; 	value - adc_step
				bra		NN,2f						; 	if (w0 < 0)
				add		w1,w0,w1					; 	  w1 = max_duty + w0
				bra		3f							; 	else
2:				mov		w1,w1						; 	  w1 = max_duty
3:				bra		NN,0f						; 	if (w1 < 0)
				clr		w1							; 	  w1 = 0
0:				mov		w1,[w2++]					; 	result -> LED_data
				cpseq	w2,w3
				bra		1b							; } end while

				;--- H ---
				mov		#LED_data_H,w2				; w2 -> LED_data_H
				mov		#LED_data_H+(num_LEDs*2),w3	; w3 is loop stopper

				mov		#adc_floor_H-adc_step_H,w0	; value - (adc_floor - adc_step)
				sub		analog_val1,wreg			; use slowest decay
;				sub		analog_avg2,wreg			; use slow average
				mov		#max_duty,w1
1:													; while (w2 < w3) {
				sub		#adc_step_H,w0				; 	value - adc_step
				bra		NN,2f						; 	if (w0 < 0)
				add		w1,w0,w4					; 	  w4 = max_duty + w0
				bra		3f							; 	else
2:				sub		w1,w0,w4					; 	  w4 = max_duty - w0
3:				bra		NN,0f						; 	if (w4 < 0) {
				neg		w4,w4						; 	  w4 = -w4
0:													;   }
				inc		an_offset					;   increase offset value
				mov		an_offset,w5				;   add offset to hue
				lsr		w5,#8,w5
				add		w4,w5,w4
				and		#0xFF,w4					;   use lower 8 bits only
				mul.uu	w4,#6,w4					;   w4 *= 6
				mov		w4,[w2++]					; 	result -> LED_data
				cpseq	w2,w3
				bra		1b							; } end while

				;--- S ---
				mov		#LED_data_S,w2				; w2 -> LED_data_S
				mov		#0xFF,w0					; fill with 0xFF
				disi	#num_LEDs
				repeat	#num_LEDs-1
				mov		w0,[w2++]

;				call	convert_HSV

				return

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
; analog_LED1 with peak detection
;
analog_LED3:
.equ	peak_thres_h,580 ; 680
.equ	peak_thres_l,500 ; 580
.equ	peak_reset,64		; threshold to reset the peak effect


				bset	g_flags,#do_not_read		; disable LED data read
				call	analog_LED1

				;--- color change on the beat ---
				btss	g_flags,#an_peak			; if peak was detected before
				bra		1f
				mov		analog_val1,w0				; 	wait until the value goes below the threshold
				mov		#peak_thres_l,w1
				cp		w0,w1						; 	analog_val - peak_thres_l
				bra		GTU,0f						;	below the threshold
				bclr	g_flags,#an_peak			; 	  clear the flag

1:				mov		analog_val1,w0
				mov		#peak_thres_h,w1
				cp		w0,w1						; analog_val - peak_thres_h
				bra		LTU,0f						; peak detected
				inc		an_peak_cnt					; count the peak/beats
				mov		#1500,w0					; set the reset timer
				mov		w0,an_peak_time
				bset	g_flags,#an_peak			; set the flag
0:
				mov		analog_val1,w0
				mov		#peak_reset,w1
				cp		w0,w1						; analog_val - peak_reset
				bra		GTU,0f						; if the level goes below the reset threshold
				dec		an_peak_time				; decrement the timer
				bra		NZ,0f						; if timer = 0
				clr		an_peak_cnt					;   clear peak count
0:
				btsc	an_peak_cnt,#0
				call	swap_R_B
				btsc	an_peak_cnt,#1
				call	swap_R_G
				btsc	an_peak_cnt,#2
				call	swap_G_B

				bclr	g_flags,#do_not_read		; enable LED data read
				return

;- - - - - - - - - - - - - - - - - - - -
; swap R/G/B channel data
swap_R_G:
				mov		#LED_data_R,w1
				mov		#LED_data_G,w2
				bra		1f
swap_R_B:
				mov		#LED_data_R,w1
				mov		#LED_data_B,w2
				bra		1f
swap_G_B:
				mov		#LED_data_G,w1
				mov		#LED_data_B,w2
1:
				mov		w2,w3
				add		#num_LEDs*2,w3				; w3 is loop stopper
0:												; while (w2 < w3) {
				mov		[w2],w0
				mov		[w1],[w2++]
				mov		w0,[w1++]
				cpseq	w2,w3
				bra		0b						; } end while
				return


;-------------------------------------------------------------------------------
;	process analog input
;		slow decay on analog_val1,analog_val2, and analog_val3
;
.equ	adc_floor,32	; ADC noise/offset level
analog_input:
				btss	IFS0,#AD1IF					; return if conversion is not done
				return

				bclr	IFS0,#AD1IF					; clear the DONE flag
				bset	AD1CON1,#SAMP				; start sampling analog input

				mov		ADC1BUF0,w0					; w0 = ADC value

				; store the ADC value in the buffer
				mov		#analog_buf1,w1
				mov		analog_ptr1,w2
				mov		w0,[w1+w2]
				inc2	analog_ptr1
				mov		#(ana_samples1-1)*2,w3		; if the pointer points outside buffer
				cp		w2,w3
				btsc	SR,#C
				clr		analog_ptr1					; 	roll back

				; average all samples
				clr		w2
				disi	#ana_samples1
				repeat	#ana_samples1-1
				add		w2,[w1++],w2				; w2 = sum(analog_buf1)
				lsr		w2,#ana_devbit1,w2			; divide by n
				sub		#adc_floor,w2				; adjust the noise offset
				btsc	SR,#N
				clr		w2
				mov		w2,analog_avg1

.if 0			;--- long average ---
				;--- store the average value in the buffer ---
				mov		#analog_buf2,w1
				mov		analog_ptr2,w2
				mov		w0,[w1+w2]
				cp0		analog_ptr1					; advance the pointer only every #ana_samples samples
				bra		NZ,0f
				inc2	analog_ptr2
				mov		#(ana_samples2-1)*2,w3		; if the pointer points outside buffer
				cp		w2,w3
				btsc	SR,#C
				clr		analog_ptr2					; 	roll back
0:
				; average all samples
				clr		w2
				disi	#ana_samples2
				repeat	#ana_samples2-1
				add		w2,[w1++],w2				; w2 = sum(analog_buf2)
				lsr		w2,#ana_devbit2,w2			; divide by n
				sub		#adc_floor,w2				; adjust the noise offset
				btsc	SR,#N
				clr		w2
				mov		w2,analog_avg2
.endif
				;--- process other values ----------------------------
				mov		analog_avg1,w0				; use short average value

				;--- apply knee ---
.equ	ana_knee_point,400
				mov		#ana_knee_point,w1			; if (w0 > knee_point)
				cp		w0,w1
				bra		LT,0f						; {
				add		w0,w1,w0					;   w0 = (w0 + knee_point)/2
				lsr		w0,w0
0:													; }
				mov		#(1<<10)-1,w1				; if (w0 > 0x3FF)
				cp		w0,w1
				bra		LEU,0f						; {
				mov		w1,w0						;   w0 = 0x3FF (limit the value within 10 bit)
0:													; }
				;--- analog value with slow decay ----------
				cp		analog_val1					; if (analog_val < analog_val1)
				bra		LE,0f
				dec		analog_val1					;   analog_val1--
				bra		1f							; else
0:				mov		w0,analog_val1				;   analog_val1 = analog_val
1:
				bclr	g_flags,#no_LED				; set no_LED flag if value is zero
				cp0		analog_val1
				btsc	SR,#Z
				bset	g_flags,#no_LED

				;--- analog value with mid decay -----------
				cp		analog_val2					; if (analog_val < analog_val2)
				bra		LE,0f
				dec2	analog_val2					;   analog_val2--
				cp		analog_val2					;   if (analog_val > analog_val2)
				bra		GT,1f						;      analog_val2 = analog_val
													; else
0:				mov		w0,analog_val2				;   analog_val2 = analog_val
1:
				;--- analog value with fast decay ----------
				cp		analog_val3					; if (analog_val < analog_val3) {
				bra		LE,0f
				dec2	analog_val3					;   analog_val3 -= 2
				cp		analog_val3					;   if (analog_val > analog_val3)
				bra		LE,0f						;      analog_val3 = analog_val
				dec2	analog_val3					;   analog_val3 -= 2
				cp		analog_val3					;   if (analog_val > analog_val3)
				bra		GT,1f						;      analog_val3 = analog_val
													; } else
0:				mov		w0,analog_val3				;   analog_val3 = analog_val
1:
				return


;-------------------------------------------------------------------------------
;	button check

button_check:
				;--- sample all buttons ---
				mov		SW1_PORT,w0
				btst.c	w0,#SW1_BIT
				rlc		btnA_data+samples			; add new sample at LSB

				mov		#btnA_data,w1
				call	check_one_button			; process one switch at a time

				return

				;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
check_one_button:
				;--- process the button data ---
				btsc	[w1],#btn_down		; is the button up or down?
				bra		button_down

button_up:		;--- test if the button is down ---
				mov		#debounce_bits,w0
				mov		[w1+samples],w2
				and		w0,w2,w2					; if all samples are low
				bra		NZ,0f
				bset	[w1],#btn_down				; button is now down
				mov		#long_push_time,w0			; set the long push counter
				mov		w0,[w1+long_push]
0:				return

button_down:	;--- test if the button is up ---
				mov		#debounce_bits,w0
				mov		[w1+samples],w2
				and		w0,w2,w2					; if all samples are low
				xor		w2,w0,w2					; if all samples are high
				bra		NZ,button_held
				bclr	[w1],#btn_down				; button is now up
				bset	[w1],#btn_push				; button has been pushed
				return

button_held:
				add		w1,#long_push,w2
				dec		[w2],[w2]					; decrement long push counter
				bra		Z,0f
				return
0:				bset	[w1],#btn_long				; button is now long pushed
				mov		#long_push_time,w0			; set the long push counter
				mov		w0,[w1+long_push]
				return

;-------------------------------------------------------------------------------
;	animate LEDs
;
;	register usage: 
;		w5 = anim_release_cnt_X
;		w6 = anim_flags_X
;		w14 = max_duty
;		w7 = max_duty - duty_diff_X
;		w8 = duty_diff_X
;		w9,w10,w11,w12,w13

animate_LED:
				add		w1,#anim_delay,w0
				cp0		[w0]						; check delay
				bra		Z,0f
				dec		[w0],[w0]
				return
0:
				mov		[w1+#anim_update_rate],w0
				add		w1,#anim_update,w3
				add		w0,[w3],[w3]				; update_X += update_rate_X
				bra		C,0f						; skip update if update_X <= 0xFFFF
				return
0:
				;--- prepare for LED loop --------
				mov		w2,w9						; w9 -> LED_data_X
				mov		#(2*(num_LEDs-1)),w0
				add		w2,w0,w11					; w11 -> last LED_data_X

				mov		[w1+#anim_step_up],w12
				mov		[w1+#anim_step_down],w13

				mov		[w1+#anim_duty_diff],w8		; w8 = duty_diff_X
				mov		[w1+#anim_max_duty],w14		; w14 = max_duty
				sub		w14,w8,w7					; w7 = max_duty - duty_diff_X

				mov		[w1+#anim_flags],w6
				btsc	w6,#af_direction			; if (af_direction = 0) {
				bra		1f
				add		w2,#2,w10					;   LED_data_#2 -> w10
				bra		2f							; } else {
1:				mov		w11,w10						;   last LED_data_X -> w10
2:													; }

				add		w1,#anim_release_cnt,w5		; w5 -> release_cnt_X

LED_loop:		;--- LED x loop ----------------------------
				mov		#(0xFFFF-1<<DUTY_DIR-1<<DUTY_HOLD),w0
				and		w0,[w9],w0					; mask flag bits and copy LED_data_Xn -> w0		

				btss	[w9],#DUTY_HOLD				; if DUTY_HOLD set
				bra		1f							; {
				dec		[w5],[w5]					;   if (--release_cnt_X != 0)
				bra		NZ,2f						;     continue to the next LED
				bclr	[w9],#DUTY_HOLD				;   else clear DUTY_HOLD
1:													; }
				btsc	[w9],#DUTY_DIR				; duty up or down?
				bra		LED_rampDown
LED_rampUp:
				cpsne	w8,w0						; if LED_data_Xn = duty_diff_X
				bclr	[w10],#DUTY_HOLD			; release DUTY_HOLD of next LED

				add		w0,w12,w0					; LED_data_Xn += step_up
				cp		w0,w14						; 
				bra		LTU,2f						; if (LED_data_Xn >= max_duty) {
				mov		w14,w0						;   LED_data_Xn = max_duty
				btg		[w9],#DUTY_DIR				;   and toggle the dir flag
				btss	w6,#af_duty_hold_rs			;   if DUTY_HOLD_RS set {
				bra		2f							; 
				bset	[w9],#DUTY_HOLD				;     hold the fade
				mov		[w1+#anim_release_waitH],w3
				mov		w3,[w5]						;     reset anim_release_cnt_Xn
				bra		2f							;   }
													; }
LED_rampDown:
				cpsne	w7,w0						; if LED_data_Xn = (max_duty - duty_diff_X)
				bclr	[w10],#DUTY_HOLD			; release DUTY_HOLD of next LED

				sub		w0,w13,w0					; LED_data_Xn -= step_down
				bra		C,2f						; if (LED_data_Xn <= 0) {
				clr		w0							;   LED_data_Xn = 0
				btg		[w9],#DUTY_DIR				;   and toggle the dir flag
				btss	w6,#af_duty_hold_rs			;   if DUTY_HOLD_RS set {
				bra		2f
				bset	[w9],#DUTY_HOLD				;     hold the fade
				mov		[w1+#anim_release_waitL],w3
				mov		w3,[w5]						;     reset anim_release_cnt_Xn
													;   }
													; }
2:
				mov		#(1<<DUTY_DIR-1<<DUTY_HOLD),w3
				and		w3,[w9],[w9]				; keep only the flags
				ior		w0,[w9],[w9]				; copy w0 -> LED_data_Xn
				cpsne	w9,w11						; last one done?
				return								;   then return

				inc2	w5,w5						; else move on to next LED
				inc2	w9,w9
				inc2	w10,w10
				cpsgt	w10,w11						; if next LED > last LED
				bra		LED_loop					; {
				mov		w2,w10						;   LED_data_#1 -> w10
				bra		LED_loop					; }

;-------------------------------------------------------------------------------
;	convert HSV data to RGB
;
convert_HSV:
				mov		#LED_data_H,w3				; w3 -> LED_data_H
				mov		#LED_data_R,w7				; w7 -> LED_data_R
				mov		#LED_data_R+(num_LEDs-1)*2,w9	; w9 -> end of LED_data_R

				bset	g_flags,#do_not_read		; disable LED data read

0:													; while (w7 != w9) {
				mov		#(0xFFFF-1<<DUTY_DIR-1<<DUTY_HOLD),w0
				mov		[w3],w4						;   H value -> w4
				and		w0,w4,w4					;   mask the anim flags

				mov		[w3+num_LEDs*2],w5			;   S value -> w5
				and		w0,w5,w5					;   mask the anim flags

				mov		[w3+num_LEDs*4],w6			;   V value -> w6
				and		w0,w6,w6					;   mask the anim flags

				call	HSV2RGB						;   do HSV -> RGB conversion
				cp		w7,w9
				bra		Z,1f
				inc2	w3,w3
				inc2	w7,w7
				bra		0b							; }
1:
				bclr	g_flags,#do_not_read		; enable LED data read

				return

;-------------------------------------------------------------------------------
;	HSV -> RGB conversion
;	takes:		H (0-255*6): w4
;				S (0-255): w5
;				V (0-255): w6
;	returns:	R (0-255): [w7] (LED_data_Rx)
;				G (0-255): [w7+num_LEDs*2] (LED_data_Gx)
;				B (0-255): [w7+num_LEDs*4] (LED_data_Bx)

HSV2RGB:
				cp0		w5							; if (S == 0) {
				bra		NZ,0f
				mov		w6,[w7]						;   R = G = B = V
				mov		w6,[w7+num_LEDs*2]
				mov		w6,[w7+num_LEDs*4]
				return
0:													; } else {
				mov		w4,var_H					;   var_H = H
				lsr		w4,#8,w0					;   var_I = var_H / 256
				mov 	w0,var_I					;   var_I (0-5)
				sl		w0,#8,w0					;   var_H = var_H - var_I*256
				sub		var_H

													;   var_1 = V(256 - S)/256
				mov		#256,w0
				sub		w0,w5,w0					;   w0 = 256 - S
				mul.uu	w0,w6,w0					;   w1:w0 = V * w0
				lsr		w0,#8,w0					;   w0 = w0 / 256
				mov		w0,var_1

													;   var_2 = V(256 - S(var_H)/256)/256
				mov		var_H,w0
				mul.uu	w0,w5,w0					;   w0 = S * var_H
				lsr		w0,#8,w0					;   w0 = w0 / 256
				neg		w0,w0						;   w0 = 256 - w0
				add		#256,w0
				mul.uu	w0,w6,w0					;   w1:w0 = V * w0
				lsr		w0,#8,w0					;   w0 = w0 / 256
				mov		w0,var_2

													;   var_3 = V(256 - S(256 - var_H)/256)/256
				mov		var_H,w0
				neg		w0,w0						;   w0 = 256 - var_H
				add		#256,w0
				mul.uu	w0,w5,w0					;   w0 = S * w0
				lsr		w0,#8,w0					;   w0 = w0 / 256
				neg		w0,w0						;   w0 = 256 - w0
				add		#256,w0
				mul.uu	w0,w6,w0					;   w1:w0 = V * w0
				lsr		w0,#8,w0					;   w0 = w0 / 256
				mov		w0,var_3

				mov		var_I,w0					;   switch (var_I)
				bra		w0
				bra		var_I0
				bra		var_I1
				bra		var_I2
				bra		var_I3
				bra		var_I4
				bra		var_I5
var_I0:
				mov		w6,[w7]						; R = V
				mov		var_3,w0					; G = var_3
				mov		w0,[w7+num_LEDs*2]
				mov		var_1,w0					; B = var_1
				mov		w0,[w7+num_LEDs*4]
				return
var_I1:
				mov		var_2,w0					; R = var_2
				mov		w0,[w7]
				mov		w6,[w7+num_LEDs*2]			; G = V
				mov		var_1,w0					; B = var_1
				mov		w0,[w7+num_LEDs*4]
				return
var_I2:
				mov		var_1,w0					; R = var_1
				mov		w0,[w7]
				mov		w6,[w7+num_LEDs*2]			; G = V
				mov		var_3,w0					; B = var_3
				mov		w0,[w7+num_LEDs*4]
				return
var_I3:
				mov		var_1,w0					; R = var_1
				mov		w0,[w7]
				mov		var_2,w0					; G = var_2
				mov		w0,[w7+num_LEDs*2]
				mov		w6,[w7+num_LEDs*4]			; B = V
				return
var_I4:
				mov		var_3,w0					; R = var_3
				mov		w0,[w7]
				mov		var_1,w0					; G = var_1
				mov		w0,[w7+num_LEDs*2]
				mov		w6,[w7+num_LEDs*4]			; B = V
				return
var_I5:
				mov		w6,[w7]						; R = V
				mov		var_1,w0					; G = var_1
				mov		w0,[w7+num_LEDs*2]
				mov		var_2,w0					; B = var_2
				mov		w0,[w7+num_LEDs*4]
				return
													; } (endif)

;===============================================================================
;	PWM update service
;
__T2Interrupt:
				push.s								; push w0 - w3 and status
				push.d	w4							; push w4,w5

				;--- RGB channel switch ------------------------------
				mov		RGB,w0
				bra		w0
				bra		Blanking					; first time call only
				bra		Ch_G
				bra		Ch_B
				clr		RGB		; catch

Ch_R:			;--- R channel ---------------------------------------
				;--- set next PWM duty ---
				mov		pulse_duration,w0
				btss	g_flags,#no_LED				; if no_LED flag clear
				mov		w0,PWM_R_OC					; set the next OC1RS value

				;--- start single pulse generation on R ---
				mov		#OCCON_VAL,w0
				btss	g_flags,#no_LED				; if no_LED flag clear
				mov		w0,PWM_R_OCC				;   set OC register

				;--- set PORTs -----------------------------
				mov		port_buff,w0				; load next PORTA data into W
				mov		port_buff+2,w1				; load next PORTB data into W
				
.if (PORTA_bits>0)
				mov		w0,LATA						; update PORTA LATCH
				mov		w1,LATB						; update PORTB LATCH
.else
				mov		w0,LATB						; update PORTB LATCH
				mov		w1,LATC						; update PORTC LATCH
.endif

				;--- prepare PORT data for the next cycle --
				call	update_ports_G

				bra		RGB_done

Ch_G:			;--- G channel ---------------------------------------
				;--- set next PWM duty ---
				mov		pulse_duration,w0
				btss	g_flags,#no_LED				; if no_LED flag clear
				mov		w0,PWM_G_OC					; set the next OC3RS value

				;--- start single pulse generation on G ---
				mov		#OCCON_VAL,w0
				btss	g_flags,#no_LED				; if no_LED flag clear
				mov		w0,PWM_G_OCC				;   set OC register

				;--- set PORTs -----------------------------
				mov		port_buff,w0				; load next PORTA data into W
				mov		port_buff+2,w1				; load next PORTB data into W
.if (PORTA_bits>0)
				mov		w0,LATA						; update PORTA LATCH
				mov		w1,LATB						; update PORTB LATCH
.else
				mov		w0,LATB						; update PORTB LATCH
				mov		w1,LATC						; update PORTC LATCH
.endif

				;--- prepare PORT data for the next cycle --
				call	update_ports_B

				bra		RGB_done

Ch_B:			;--- B channel ---------------------------------------
				;--- set next PWM duty ---
				mov		pulse_duration,w0
				btss	g_flags,#no_LED				; if no_LED flag clear
				mov		w0,PWM_B_OC					; set the next OC2RS value

				;--- start single pulse generation on B ---
				mov		#OCCON_VAL,w0
				btss	g_flags,#no_LED				; if no_LED flag clear
				mov		w0,PWM_B_OCC				;   set OC register

				;--- set PORTs -----------------------------
				mov		port_buff,w0				; load next PORTA data into W
				mov		port_buff+2,w1				; load next PORTB data into W
.if (PORTA_bits>0)
				mov		w0,LATA						; update PORTA LATCH
				mov		w1,LATB						; update PORTB LATCH
.else
				mov		w0,LATB						; update PORTB LATCH
				mov		w1,LATC						; update PORTC LATCH
.endif

				;--- move on to the next PWM level -------------------
				inc2	output_duty
				mov		#max_duty,w0
				cp		output_duty					; if output_duty > max_duty
				bra		LEU,B_done
MSB_done:											; max_duty done {
				btsc	g_flags,#do_not_read		; if LED data being updated
				bra		Blanking					;   skip copying of LED data

				;--- sort & copy LED duty values to buffer -----------
				mov		#tblpage(LED_pins),w0		; prepare LED pin lookup
				mov		w0,TBLPAG
				clr		w0							; clear w0 to prepare for byte read

				;--- R data ---
				mov		#tbloffset(LED_pins),w5		; table address -> w5
				mov		#LED_data_R,w1				; w1 -> LED_data
				mov		#LED_data_R+(2*num_LEDs),w4	; w4 -> last LED_data (loop stopper)
				mov		#duty_buff_R,w2				; w2 -> duty_buff
0:
				tblrdl.b [w5++],w0					; read the LED pin destination
				mov		[w1++],[w2+w0]				; copy LED_data(w1) -> duty_buff_x
				cp		w1,w4						; finish if (w1 = w4)
				bra		NZ,0b

				;--- G data ---
				mov		#tbloffset(LED_pins),w5		; table address -> w5

				mov		#LED_data_G,w1
				mov		#LED_data_G+(2*num_LEDs),w4	; address of the last LED_data -> w4
				mov		#duty_buff_G,w2
0:
				tblrdl.b [w5++],w0					; read the LED pin destination
				mov		[w1++],[w2+w0]				; LED_data(w1) -> duty_buff_x
				cp		w1,w4						; finish if (w1 = w4)
				bra		NZ,0b

				;--- B data ---
				mov		#tbloffset(LED_pins),w5		; table address -> w5

				mov		#LED_data_B,w1
				mov		#LED_data_B+(2*num_LEDs),w4	; address of the last LED_data -> w4
				mov		#duty_buff_B,w2
0:
				tblrdl.b [w5++],w0					; read the LED pin destination
				mov		[w1++],[w2+w0]				; LED_data(w1) -> duty_buff_x
				cp		w1,w4						; finish if (w1 = w4)
				bra		NZ,0b

Blanking:		;--- Blanking period ---------------------------------
				clr		output_duty
				inc		output_duty					; output_duty = 1
				mov		#2,w0
				mov		w0,RGB						; RGB = 2 so R-ch will be next
.ifdef __DEBUG
				btg		_debug_port,#_debug_out		; *DEBUG* output pulse
.endif

				;-----------------------------------------------------
													; }

B_done:			;--- expand duty value via LUT ---
				mov		PSVPAG,w1					; backup PSVPAV
				mov		#psvpage(duty_LUT),w0		; set up PSV
				mov		w0,PSVPAG
				mov		#psvoffset(duty_LUT),w3

				mov		output_duty,w0
				; dimmed output - shorten the LED on time
				btsc	g_flags,#dimmed1
				lsr		w0,w0						; dim one level
				btsc	g_flags,#dimmed2
				lsr		w0,w0						; dim another level
0:
;				sl		w0,w0						; address offset = value * 2
				bclr	w0,#0						; make sure w0 is even number
				mov		[w3+w0],w0
				mov		w0,pulse_duration
				mov		w1,PSVPAG					; restore PSVPAV

				;--- prepare PORT data for the next cycle --
				call	update_ports_R

RGB_done:		;---  ------------------------------------------------
				inc		RGB							; next RGB channel

				clrwdt

				pop.d	w4							; pop w4,w5
				pop.s
				bclr	IFS0,#T2IF					; clear TMR2 int flag
				retfie

;---------------------------------------------------------------------
;	update ports based on the LED duty values
;
update_ports_R:
				mov		#duty_buff_R,w1				; point to duty buffer
				mov		#port_buff,w2				; point to port buffer
				bra		1f
update_ports_G:
				mov		#duty_buff_G,w1				; point to duty buffer
				mov		#port_buff,w2				; point to port buffer
				bra		1f
update_ports_B:
				mov		#duty_buff_B,w1				; point to duty buffer
				mov		#port_buff,w2				; point to port buffer
1:
				mov		output_duty,w3				; current output_duty -> w3
;				bclr	g_flags,#no_LED				; clear no LED flag

.if (PORTA_bits > 0)
				mov		#(1<<(PORTA_bits-1)),w4		; set the stopper bit
				mov		w4,[w2]						; this exits the loop
0:
				mov		[w1++],w0					; LED_data -> w0
				cp.b	w0,w3						; compare LED_data:output_duty -> C
				rrc		[w2],[w2]					; C -> MSB of port data
				bra		NC,0b

;				cp0		[w2]						; if ([w2] == 0)
;				bra		NZ,1f
;				bset	g_flags,#no_LED				;   set no LED flag
1:
.if (PORTA_bits < 16)
				repeat	#16-PORTA_bits-1
				lsr		[w2],[w2]
.endif
.if	(COL_POL)	; active-high drive
				inc2	w2,w2						; advance the pointer
.else			; active-low drive
				com		[w2],[w2++]					; invert the data
.endif
.endif

													; repeat on PORTB
				mov		#(1<<(PORTB_bits-1)),w4		; set the stopper bit
				mov		w4,[w2]						; this exits the loop
0:
				mov		[w1++],w0					; LED_data -> w0
				cp.b	w0,w3						; compare LED_data:output_duty -> C
				rrc		[w2],[w2]					; C -> MSB of port data
				bra		NC,0b

;				btss	g_flags,#no_LED				; if no_LED flag set
;				bra		1f							; {
;				cp0		[w2]						;   if ([w2] != 0)
;				bra		Z,1f
;				bclr	g_flags,#no_LED				;     clear no LED flag
1:													; }
.if (PORTB_bits < 16)
				repeat	#16-PORTB_bits-1
				lsr		[w2],[w2]
.endif
.if	(COL_POL)	; active-high drive
				inc2	w2,w2						; advance the pointer
.else			; active-low drive
				com		[w2],[w2++]					; invert the data
.endif

.if (PORTC_bits > 0)
													; repeat on PORTC
				mov		#(1<<(PORTC_bits-1)),w4		; set the stopper bit
				mov		w4,[w2]						; this exits the loop
0:
				mov		[w1++],w0					; LED_data -> w0
				cp.b	w0,w3						; compare LED_data:output_duty -> C
				rrc		[w2],[w2]					; C -> MSB of port data
				bra		NC,0b

;				btss	g_flags,#no_LED				; if no_LED flag set
;				bra		1f							; {
;				cp0		[w2]						;   if ([w2] != 0)
;				bra		Z,1f
;				bclr	g_flags,#no_LED				;     clear no LED flag
1:													; }
.if (PORTC_bits < 16)
				repeat	#16-PORTC_bits-1
				lsr		[w2],[w2]
.endif
.if	(!COL_POL)	; active-low drive
				com		[w2],[w2]					; invert the data
.endif
.endif
				return

;---------------------------------------------------------------------
;	duty expansion look up table
;
	.equ	duty_knee,6
	.equ	max_pulse_length,(pr_value-port_blank-1)

duty_LUT:	.word	port_delay+1
			.rept	duty_knee			; slow slope at the low brightness
				.word	port_delay+1
			.endr
			.equ	x, 0
			.rept	max_duty/2-duty_knee
				.equ	x, x+2
				.equ	y, x*(max_pulse_length-1)/((max_duty/2-duty_knee)*2)+1
				.word	(port_delay+y) % (pr_value+1)		; OCxRS (end of pulse) value
			.endr

.if 0	;--- show/hide pulse_duration values for evaluation ---
duty_LUT_eval:	.word	pr_value
			.rept	duty_knee			; slow slope at the low brightness
				.word	1
			.endr
			.equ	x, 0
			.rept	max_duty/2-duty_knee
				.equ	x, x+2
				.equ	y, x*(max_pulse_length-1)/((max_duty/2-duty_knee)*2)+1
;				.equ	y, x*(max_pulse_length-1-(duty_knee/2))/(max_duty-duty_knee)+(duty_knee/2)
				.word	y
			.endr
duty_LUT_end:
			.word	max_pulse_length	; EOT mark
.endif

;-------------------------------------------------------------------------------
;	EEPROM read/write
;
	.equ	eeprom_addr,0x7ffe00	; EEPROM address to keep the data
eeprom_write:	;--- write w0 into eeprom_addr+w1 ---
				mov		#tblpage(eeprom_addr),w2	; top 8 bits of EEPROM address
				mov		w2,TBLPAG
				mov		#tbloffset(eeprom_addr),w2	; address -> w2
				add		w2,w1,w1

				tblwtl	w0,[w1]						

				clr		NVMCON
				bset	NVMCON,#WREN				; set write enable bit
				bset	NVMCON,#NVMOP2

				disi	#7							; disable INITs
													; unlock sequence
				mov		#0x55,w1
				mov		w1,NVMKEY
				mov		#0xAA,w1
				mov		w1,NVMKEY
				bset	NVMCON,#WR					; initiate write oparation
				nop
				nop

0:				clrwdt
				btsc	NVMCON,#WR					; wait till WR clears
				bra		0b

				return


eeprom_read:	;--- read eeprom_addr+w1 into w0 --------------
				mov		#tblpage(eeprom_addr),w2	; top 8 bits of EEPROM address
				mov		w2,TBLPAG
				mov		#tbloffset(eeprom_addr),w2	; address -> w2
				add		w2,w1,w1

				tblrdl	[w1],w0

				return

;-------------------------------------------------------------------------------
;	IR receiver service
__CNInterrupt:
				push.s

				btsc	IR_PORT,#IR_BIT				; is the IR port high?
				bra		IR_read_timer				;   IR port is high - read the timer
IR_start_timer:										;   IR port is low - start the timer
				mov		#start_bit,w0				; set timer3 period
				mov		w0,PR3
				clr		TMR3
				bclr	IFS0,#T3IF					; clear timer3 overflow flag
				bset	T3CON,#TON					; start timer3
				bra		IR_end
IR_read_timer:
				bclr	T3CON,#TON					; stop timer3
				btsc	IFS0,#T3IF					; did timer3 overflow?
				bra		IR_start_bit				;   then it's a start bit

				btss	IR_flags,#IR_RCV			; else check if start bit has already been received
				bra		IR_end						;   not a start bit, skip until start bit comes in
													; yes, move on to decoding data bit

IR_long_short:										; long or short pulse (1 or 0)?
				mov		#bit_threshold,w0			; compare timer with bit threshold
				cp		TMR3						; data bit is in the C flag
				rrc		_IR_data					; shift into the data buffer
IR_data_end:
				bra		NC,IR_end					; if the C flag set, all bits in
IR_data_done:										; process the received data
				mov		_IR_data,w0
				lsr		w0,#4,w0					; shift data 4 bits (data is only 12 bits)
				cp		_IR_last_data				; compare the data with the last data received
				bra		Z,IR_data_verified
				mov		w0,_IR_last_data			; update last data buffer
				bra		IR_data_discard				; discard the data received
IR_data_verified:
				mov		w0,IR_keycode				; separate the data into keycode and device code
				lsr		w0,#7,w0					; device code is the upper 5 bits above 7 bits of keycode
				mov		w0,IR_device
				mov		#0b01111111,w0
				and		IR_keycode					; keycode is the last 7 bits
				bset	IR_flags,#IR_RCD			; set IR received flag
IR_data_discard:
				bclr	IR_flags,#IR_RCV			; clear IR receiving flag
				bra		IR_end

IR_start_bit:										; start bit came in
				mov		#1<<11,w0					; initialize the data buffer (12th bit set)
				mov		w0,_IR_data
				bset	IR_flags,#IR_RCV			; set IR receiving flag

IR_end:
				bclr	IFS1,#CNIF					; clear CN int flag
				pop.s
				retfie

;-------------------------------------------------------------------------------
;===============================================================================

	.end
