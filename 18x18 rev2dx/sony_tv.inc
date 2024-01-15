;=====================================================================
;	Sony TV remote codes
;

;--- command codes(device code=0x01 for TV) ---
;				key				|	code
; 			------------------------------
	.equ	IR_key_Power		,	0x15
	.equ	IR_key_Muting		,	0x14
	.equ	IR_key_Sleep		,	0x36
	.equ	IR_key_TV_VCR		,	0x25
	.equ	IR_key_Display		,	0x3A
	.equ	IR_key_1			,	0x00
	.equ	IR_key_2			,	0x01
	.equ	IR_key_3			,	0x02
	.equ	IR_key_4			,	0x03
	.equ	IR_key_5			,	0x04
	.equ	IR_key_6			,	0x05
	.equ	IR_key_7			,	0x06
	.equ	IR_key_8			,	0x07
	.equ	IR_key_9			,	0x08
	.equ	IR_key_0			,	0x09
	.equ	IR_key_Jump			,	0x3B
	.equ	IR_key_Enter		,	0x0B
	.equ	IR_key_Vol_up		,	0x12
	.equ	IR_key_Vol_down		,	0x13
	.equ	IR_key_Ch_up		,	0x10
	.equ	IR_key_Ch_down		,	0x11
	.equ	IR_key_Reset		,	0x16
	.equ	IR_key_Menu			,	0x60
	.equ	IR_key_up			,	0x74
	.equ	IR_key_down			,	0x75
	.equ	IR_key_left			,	0x34
	.equ	IR_key_right		,	0x33
	.equ	IR_key_center		,	0x65

; device code
	.equ	device_sony_tv		,	0x01
