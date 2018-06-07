;
; main.asm
;
; Created: 05/03/2018
; Author : Evan Hollins
; SID: 43987982
;
.include "m328Pdef.inc"
.include "constants.inc"
.org 0x0000
; complete interrupt vector table.
	jmp RESET ; Reset
	jmp INT0_IR ; IRQ0
	jmp INT1_IR ; IRQ1
	jmp PCINT0_IR ; PCINT0
	jmp PCINT1_IR ; PCINT1
	jmp PCINT2_IR ; PCINT2
	jmp WDT_IR ; Watchdog Timeout
	jmp TIM2_COMPA ; Timer2 CompareA
	jmp TIM2_COMPB ; Timer2 CompareB
	jmp TIM2_OVF ; Timer2 Overflow
	jmp TIM1_CAPT ; Timer1 Capture
	jmp TIM1_COMPA ; Timer1 CompareA
	jmp TIM1_COMPB ; Timer1 CompareB
	jmp TIM1_OVF ; Timer1 Overflow
	jmp TIM0_COMPA ; Timer0 CompareA
	jmp TIM0_COMPB ; Timer0 CompareB
	jmp TIM0_OVF ; Timer0 Overflow
	jmp SPI_STC ; SPI Transfer Complete
	jmp USART_RXC ; USART RX Complete
	jmp USART_UDRE ; USART UDR Empty
	jmp USART_TXC ; USART TX Complete
	jmp ADC_CC ; ADC Conversion Complete
	jmp EE_RDY ; EEPROM Ready
	jmp ANA_COMP ; Analog Comparator
	jmp TWI_IR ; 2-wire Serial
	jmp SPM_RDY ; SPM Ready
;	and we branch to the final location from here.
;	Interrupts we cannot handle in this example we just loop at noint
;INT0_IR: rjmp  noint ; defined later
INT1_IR: rjmp  noint
PCINT0_IR: rjmp  noint
PCINT1_IR: rjmp  noint
PCINT2_IR: rjmp  noint
WDT_IR: rjmp  noint
TIM2_COMPA: rjmp  noint
TIM2_COMPB: rjmp  noint
;TIM2_OVF: rjmp  noint
TIM1_CAPT: rjmp  noint
TIM1_COMPA: rjmp  noint
;TIM1_COMPB: rjmp  noint ; defined later
TIM1_OVF: rjmp  noint
TIM0_COMPA: rjmp  noint
TIM0_COMPB: rjmp  noint
TIM0_OVF: rjmp  noint
SPI_STC: rjmp  noint
USART_RXC: rjmp  noint
USART_UDRE: rjmp  noint
USART_TXC: rjmp  noint
ADC_CC: rjmp  noint
EE_RDY: rjmp  noint
ANA_COMP: rjmp  noint
TWI_IR:	rjmp  noint
SPM_RDY: rjmp  noint
;
error:
noint:	
	rjmp	noint
;
;	On reset we branch to here
;
RESET:					; Main program start
	ldi		r16,high(RAMEND)	; Set Stack Pointer to top of RAM
	out		SPH,r16
	ldi		r16,low(RAMEND)
	out		SPL,r16
;
;
; initialise I/O ports and peripherals
;
; PB0 LED	Output	1
; PB1 ??	Output	1
; PB2 !SS	Output	1
; PB3 MOSI0	Output	1
; PB4 MISO0	Input	0
; PB5 SCK	Output	1
; PB6 XTAL	X	0
; PB7 XTAL	X	0
; PC4 SDA 	Output  1
; PC5 SCL 	Output  1

	ldi		r16,0b00101111		; set pin directions
	out		DDRB,r16
	ldi 	r16,0b00000100
	out 	PORTB,r16
	cbi 	DDRC,4			; next four are for I2C
	cbi 	DDRC,5
	sbi 	PORTC,4
	sbi 	PORTC,5
	ldi 	r16,0b01001010
	out 	DDRD,r16
	ldi 	r16,0b00001110
	out 	PORTD,r16

	in		r16,MCUCR	; set interrupt vectors to address 0x0002
	ori		r16,(1<<IVCE)
	out		MCUCR,r16
	andi	r16,0xfc
	out		MCUCR,r16

	cbi		EIMSK,INT0	; disable the interrupt while we configure INT0
	ldi		r16,0b00000010	; set up high to low transition interrupt for INT0
	sts		EICRA,r16
	sbi		EIMSK,INT0	; and enable the interrupt for INT0

;
; Setup Serial port - 9600,8,N,1
;
	clr	r16
	sts	UCSR0A,r16
	ldi	r16,0x18	; enable receiver and transmitter
	sts	UCSR0B,r16
	ldi	r16,0x06	; async, no parity, 1 stop, 8 bits
	sts	UCSR0C,r16
	clr	r16
	sts	UBRR0H,r16
	ldi	r16,0x67	; baud rate divisor 103 (16M/9600 - 1)
	sts	UBRR0L,r16

	; r22 holds states for zones. 
	clr 	r22
 
	; r26 holds overall system state
	clr 	r26

	; r27 holds reset button state for rising edge / debouncing
	clr 	r27

	; r28 holds timer info 
	clr 	r28

;
; Setup I2C display.

	ldi		r16,193		; setup TWI frequency scaling
	sts		TWBR,r16
	ldi		r16,0x00
	sts		TWSR,r16

	ldi		r24,0x27	; Setup LCD display at this address (Maybe 0x3f instead)
	call	LCD_Setup

	call 	LCD_write_states

;
; Setup SPI operations
; See pp217-218 of the data sheet
;
	ldi	r16,(1<<SPE)|(1<<MSTR); set master SPI, (SPI mode 0 operation is 00)
	out	SPCR,r16			; SCK is set fosc/4 => 4MHz
	clr	r16				; clear interrupt flags and oscillator mode.
	out	SPSR,r16

	clr 	r20			; Firstly, read from register 0x00, which is initialized to 0xff
	clr 	r21			; By doing this, we can sit here until the SPI chip responds
	call 	SPI_Read_Command

	ldi 	r20,0x0A		; register IOCON (configuration)
	ldi 	r21,0b00000100  ; just enable hardware addressing
	call 	SPI_Send_Command
	ldi		r20,0x00	; register IODIRA (port A data direction)
	ldi		r21,0x00	; all outputs
	call	SPI_Send_Command
	ldi		r20,0x01	; register IODIRB (port B data direction)
	ldi		r21,0xff	; all inputs
	call	SPI_Send_Command
	ldi		r20,0x0d	; register GPPUB (port B GPIO Pullups)
	ldi		r21,0xff	; turn on all pullups
	call	SPI_Send_Command
	ldi 	r20,0x05 	; register GPINTENB (port B GPIO interupts)
	ldi 	r21,0xff 	; turn on all interupts
	call 	SPI_Send_Command
	ldi 	r20,0x09 	; register INTCONB (port B interupt control)
	ldi 	r21,0x00 	; compare all pins to previous value
	call 	SPI_Send_Command

	; Read from the INTCAPB register to clear the current interrupts
	; and check to make sure that the  chip is ready
	ldi 	r20,0x11
	clr 	r21
	call 	SPI_Read_Command

	; Lastly, clear all of the outputs.
	ldi 	r20,0x12
	ldi 	r21,0x00
	call 	SPI_Send_Command

	sei				; Enable interrupts


LCD_Setup_Err:
main:
	sleep
	rjmp 	main

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; INTERRUPT ROUTINE
; 
; Run when a interrupt is recieved from the SPI chip
; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
INT0_IR:

	ldi 	r20,0x11 	; INTCAPB register
	clr 	r21
	call 	SPI_Read_Command

	mov 	r23,r16
	sbrs 	r23,0
	call 	checkZone0
	sbrs 	r23,1
	call 	checkZone1
	sbrs 	r23,2
	call 	checkZone2
	sbrs 	r23,3
	call 	checkZone3
	sbrs 	r23,6
	call 	setStateEmergency

; Check for rising edge of reset button
	sbrc 	r23,4
	jmp 	resetButtonCheckRising
	ori 	r27,0x01
	jmp 	checkIsolateButton
resetButtonCheckRising:
	sbrs 	r27,0
	jmp 	checkIsolateButton
	andi 	r27,0b11111110
	call 	resetButtonReleased

; Check for rising edge of reset button
checkIsolateButton:
	sbrc 	r23,5
	jmp 	isolateButtonCheckRising
	ori 	r27,0x02
	jmp 	interruptReturn
isolateButtonCheckRising:
	sbrs 	r27,1
	jmp 	interruptReturn
	andi 	r27,0b11111101
	call 	isolateButtonReleased
interruptReturn:
	reti			; and we're done with the interrupt

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; BUSINESS RULES
; 
; Facilitates when a state transition happens
; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

updateState:
; Start by counting the number of sensors that are triggered and and not isolated
	push 	r16
	clr 	r16
	sbrs 	r22,0
	jmp 	updateState_0
	sbrc 	r22,1
	jmp 	updateState_0
	inc 	r16
updateState_0:
	sbrs 	r22,2
	jmp 	updateState_1
	sbrc 	r22,3
	jmp 	updateState_1
	inc 	r16
updateState_1:
	sbrs 	r22,4
	jmp 	updateState_2
	sbrc 	r22,5
	jmp 	updateState_2
	inc 	r16
updateState_2:
	sbrs 	r22,6
	jmp 	updateState_3
	sbrc 	r22,7
	jmp 	updateState_3
	inc 	r16

; If we are in evacuate, stay as long and more than zero sensors, otherwise, go to all clear
updateState_3:
	call 	checkStateEvacuate
	brne 	updateState_4
	cpi 	r16, 0x01
	brge 	updateState_4
	call 	setStateAllClear
	rjmp 	updateState_ret

; If we are in alert, stay if one sensor, move to all clear if zero sensors,
; move to evacutae if more than one
updateState_4:
	call 	checkStateAlert
	brne 	updateState_5
	cpi 	r28, 0x00   	; If timer has reached zero, move to evacuate
	breq 	updateState_4_0
	cpi 	r16, 0x01
	breq 	updateState_5
	brlt 	updateState_4_1
updateState_4_0:
	call 	setStateEvacuate
	rjmp 	updateState_5
updateState_4_1:
	call 	setStateAllClear

; If we are in idle, move to the correct state if any sensors are triggered
updateState_5:
	call 	checkStateIdle
	brne 	updateState_ret
	cpi 	r16, 0x01
	brlt 	updateState_ret
	breq 	updateState_5_1
	call 	setStateEvacuate
	rjmp 	updateState_ret
updateState_5_1:
	call 	setStateAlert 

updateState_ret:
	pop 	r16
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; STATE TRANSITION LOGIC
; 
; What happens when moving into a state and checks for if in state
; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

setStateIdle:
	ldi 	r26,0x00
	andi 	r22,0b10101010		; Clear triggers on sensors
	call 	LCD_write_states
	call 	setZoneLights
	ret

checkStateIdle:
	cpi 	r26,0x00
	ret

setStateAlert:
	push 	r16
	ldi 	r26,0x01
	ldi 	r28,0x03
	call 	t1_countdown
	call 	LCD_write_states
	call 	setZoneLights
	ldi 	r16,0b01000010	; Turn on the timers for speaker out
	out 	TCCR0A,r16
	ldi 	r16,0b00000101	
	out 	TCCR0B,r16
	ldi 	r16,35 			; Set speaker compare register to play an "A"
	out 	OCR0A,r16
	pop 	r16
	ret

checkStateAlert:
	cpi 	r26,0x01
	ret

setStateEvacuate:
	ldi 	r26,0x02
	call 	t1_countdown_stop
	call 	LCD_write_states
	call 	setZoneLights
	push 	r16
	ldi 	r16,12 			; Set speaker compare register to play low note
	out 	OCR0A,r16
	ldi 	r16,0b00000000 	; Set up timer 2 to change the tone on overflow
	sts 	TCCR2A,r16
	ldi 	r16,0b00000111  
	sts  	TCCR2B,r16
	ldi 	r16,0b00000001
	sts 	TIMSK2,r16
	clr 	r16
	sts 	TCNT2,r16
	pop 	r16
	ret

checkStateEvacuate:
	cpi 	r26,0x02
	ret

setStateAllClear:
	ldi 	r26,0x03 			; Set state register
	call 	LCD_write_states
	call 	t1_countdown_stop
	call 	setZoneLights
	push 	r16
	clr 	r16
	out 	TCCR0A,r16
	out 	TCCR0B,r16
	sts 	TCCR2A,r16
	sts  	TCCR2B,r16
	sts 	TIMSK2,r16
	sts 	TCNT2,r16
	pop 	r16
	ret

checkStateAllClear:
	cpi 	r26,0x03
	ret

setStateEmergency:
	ldi 	r26,0x04
	call 	t1_countdown_stop
	call 	LCD_write_states
	call 	setZoneLights
	push 	r16
	ldi 	r16,12 			; Set speaker compare register to play low note
	out 	OCR0A,r16
	ldi 	r16,0b00000000 	; Set up timer 2 to change the tone on overflow
	sts 	TCCR2A,r16
	ldi 	r16,0b00000111  
	sts  	TCCR2B,r16
	ldi 	r16,0b00000001
	sts 	TIMSK2,r16
	clr 	r16
	sts 	TCNT2,r16
	ldi 	r16,0b01000010	; Turn on the timers for speaker out
	out 	TCCR0A,r16
	ldi 	r16,0b00000101	
	out 	TCCR0B,r16
	ldi 	r16,35 			; Set speaker compare register to play an "A"
	out 	OCR0A,r16
	pop 	r16
	ret

checkStateEmergency:
	cpi 	r26,0x04
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; FIRE ALARM SPECIFIC HELPER ROUTINES
; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

checkZone0:
	sbrc 	r22,0				; Return if the zone is already triggered
	ret
	sbr 	r22,0b00000001
	call 	LCD_write_states
	call 	setZoneLights
	call 	updateState
	ret

checkZone1:
	sbrc 	r22,2				; Return if the zone is already triggered
	ret
	sbr 	r22,0b00000100
	call 	LCD_write_states
	call 	setZoneLights
	call 	updateState
	ret

checkZone2:
	sbrc 	r22,4				; Return if the zone is already triggered
	ret
	sbr 	r22,0b00010000
	call 	LCD_write_states
	call 	setZoneLights
	call 	updateState
	ret

checkZone3:
	sbrc 	r22,6				; Return if the zone is already triggered
	ret
	sbr 	r22,0b01000000
	call 	LCD_write_states
	call 	setZoneLights
	call 	updateState
	ret

setZoneLights:
	ldi 	r20,MCP_PORT_A
	mov 	r21,r22
	call 	SPI_Send_Command
	ret


resetButtonReleased:
	call 	checkStateAlert
	breq 	resetButtonReleased_0
	call 	checkStateEvacuate
	breq 	resetButtonReleased_0
	call 	checkStateEmergency
	breq 	resetButtonReleased_0
	call 	checkStateAllClear
	breq 	resetButtonReleased_1
	call 	checkStateIdle
	breq 	resetButtonReleased_2
	jmp 	resetButtonReleased_ret ; If not in any of these states, just return

resetButtonReleased_0:
	call 	setStateAllClear
	jmp 	resetButtonReleased_ret

resetButtonReleased_1:
	call 	setStateIdle
	jmp 	resetButtonReleased_ret

resetButtonReleased_2:
	clr 	r22
	call 	LCD_write_states
	jmp 	resetButtonReleased_ret

resetButtonReleased_ret:
	call 	setZoneLights
	ret

isolateButtonReleased:
	sbrc 	r22,0
	sbr 	r22,0b00000010
	sbrc 	r22,2
	sbr 	r22,0b00001000
	sbrc 	r22,4
	sbr 	r22,0b00100000
	sbrc 	r22,6
	sbr 	r22,0b10000000
	call 	LCD_write_states
	call 	updateState
	ret

;
; interrupt timer 1 match compare B
; Used to manage countdown in alert state
;
TIM1_COMPB:
	cpi 	r28,0x00 		; Countdown complete
	brne 	TIM1_COMPB_0
	call 	updateState
	reti
TIM1_COMPB_0:
	dec 	r28
	call 	LCD_write_states
	call 	t1_countdown 
	reti

t1_countdown_stop:
	push	r16
	clr		r16				; disables all interrupts from Timer 1
	sts		TIMSK1,r16
	sts		TCCR1B,r16		; stop the clock
	pop 	r16
	ret

t1_countdown:
	push	r16
	push	r17
	push	r18
	lds		r18,TIMSK1	; save current value
	clr		r16		; disables all interrupts from Timer 1
	sts		TIMSK1,r16
	sts		TCCR1B,r16	; temporarily stop the clock
	ldi		r16,0b00000000	; port A normal, port B normal, WGM=0000 (Normal)
	sts		TCCR1A,r16
	ldi		r17,HIGH(15625)
	ldi		r16,LOW(15625)
	sts		OCR1BH,r17
	sts		OCR1BL,r16
	clr		r16		; clear current count
	sts		TCNT1H,r16
	sts		TCNT1L,r16
	ldi		r16,0b00001101	; noise = 0, WGM=0000, clk = /1024
	sts		TCCR1B,r16
	ldi		r16,0b00000000
	sts		TCCR1C,r16
	ori		r18,0b00000100	; interrupt enabled when OCB match (and other interrupts)
	sts		TIMSK1,r18
	pop		r18
	pop		r17
	pop		r16
	ret


;  Used to make the speaker make a "whooping" noise in evacuate mode
TIM2_OVF:
	push 	r16
	clr 	r16
	out 	TCCR0B,r16
	in 		r16,OCR0A
	dec 	r16
	cpi 	r16,12
	brlt 	TIM2_OVF_Reset
	rjmp 	TIM2_OVF_Return
TIM2_OVF_Reset:
	ldi 	r16,50
TIM2_OVF_Return:
	out 	OCR0A,r16
	ldi 	r16,0b00000101 
	out  	TCCR0B,r16
	pop 	r16
	reti

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; LCD WRITING
; 
; Handles writing the state, countdown and sensor states to the screen
; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

LCD_init_String0:
	.db		0x0C,0x01
LCD_state_idle:
	.db 	"Idle.",0x00
LCD_state_alert:
	.db 	"Alert",0x00
LCD_state_evacuate:
	.db 	"EVACUATE!",0x00
LCD_state_all_clear:
	.db 	"All Clear",0x00
LCD_state_emergency:
	.db 	"EMERGENCY",0x00
LCD_state_0:
	.db 	"0:",0x00,0x00
LCD_state_1:
	.db 	"1:",0x00,0x00
LCD_state_2:
	.db 	"2:",0x00,0x00
LCD_state_3:
	.db 	"3:",0x00,0x00
LCD_state_clear:
	.db 	"C|",0x00,0x00
LCD_state_triggered:
	.db 	"T|",0x00,0x00
LCD_state_isolated:
	.db 	"I|",0x00,0x00
LCD_timer_3:
	.db 	"3",0x00
LCD_timer_2:
	.db 	"2",0x00
LCD_timer_1:
	.db 	"1",0x00
LCD_timer_0:
	.db 	"0",0x00

LCD_write_states:
	call 	LCD_Clear
	ldi 	r25,0x00
	call 	LCD_Position
	call 	checkStateAlert
	breq 	LCD_write_state_alert
	call 	checkStateEvacuate
	breq	LCD_write_state_evacuate
	call 	checkStateEmergency
	breq	LCD_write_state_emergency
	call 	checkStateAllClear
	breq 	LCD_write_state_all_clear
LCD_write_state_idle:
	ldi 	ZL,LOW(LCD_state_idle*2)
	ldi 	ZH,HIGH(LCD_state_idle*2)
	call 	LCD_Text
	jmp 	LCD_write_state_0
LCD_write_state_alert:
	ldi 	ZL,LOW(LCD_state_alert*2)
	ldi 	ZH,HIGH(LCD_state_alert*2)
	call 	LCD_Text
	call 	LCD_write_alert_countdown
	jmp 	LCD_write_state_0
LCD_write_state_evacuate:
	ldi 	ZL,LOW(LCD_state_evacuate*2)
	ldi 	ZH,HIGH(LCD_state_evacuate*2)
	call 	LCD_Text
	jmp 	LCD_write_state_0
LCD_write_state_emergency:
	ldi 	ZL,LOW(LCD_state_emergency*2)
	ldi 	ZH,HIGH(LCD_state_emergency*2)
	call 	LCD_Text
	jmp 	LCD_write_state_0
LCD_write_state_all_clear:
	ldi 	ZL,LOW(LCD_state_all_clear*2)
	ldi 	ZH,HIGH(LCD_state_all_clear*2)
	call 	LCD_Text
LCD_write_state_0:
	ldi 	r25,0x40
	call 	LCD_Position
	ldi		ZL,LOW(LCD_state_0*2)
	ldi		ZH,HIGH(LCD_state_0*2)
	call	LCD_Text
	sbrc 	r22,1
	jmp 	LCD_write_state_0_I
	sbrc 	r22,0
	jmp 	LCD_write_state_0_T
LCD_write_state_0_C:
	call 	LCD_write_state_C
	jmp 	LCD_write_state_1
LCD_write_state_0_T:
	call 	LCD_write_state_T
	jmp 	LCD_write_state_1
LCD_write_state_0_I:
	call 	LCD_write_state_I
	jmp 	LCD_write_state_1

LCD_write_state_1:
	ldi		ZL,LOW(LCD_state_1*2)
	ldi		ZH,HIGH(LCD_state_1*2)
	call	LCD_Text
	sbrc 	r22,3
	jmp 	LCD_write_state_1_I
	sbrc 	r22,2
	jmp 	LCD_write_state_1_T
LCD_write_state_1_C:
	call 	LCD_write_state_C
	jmp 	LCD_write_state_2
LCD_write_state_1_T:
	call 	LCD_write_state_T
	jmp 	LCD_write_state_2
LCD_write_state_1_I:
	call 	LCD_write_state_I
	jmp 	LCD_write_state_2

LCD_write_state_2:
	ldi		ZL,LOW(LCD_state_2*2)
	ldi		ZH,HIGH(LCD_state_2*2)
	call	LCD_Text
	sbrc 	r22,5
	jmp 	LCD_write_state_2_I
	sbrc 	r22,4
	jmp 	LCD_write_state_2_T
LCD_write_state_2_C:
	call 	LCD_write_state_C
	jmp 	LCD_write_state_3
LCD_write_state_2_T:
	call 	LCD_write_state_T
	jmp 	LCD_write_state_3
LCD_write_state_2_I:
	call 	LCD_write_state_I
	jmp 	LCD_write_state_3

LCD_write_state_3:
	ldi		ZL,LOW(LCD_state_3*2)
	ldi		ZH,HIGH(LCD_state_3*2)
	call	LCD_Text
	sbrc 	r22,7
	jmp 	LCD_write_state_3_I
	sbrc 	r22,6
	jmp 	LCD_write_state_3_T
LCD_write_state_3_C:
	call 	LCD_write_state_C
	jmp 	LCD_write_state_return
LCD_write_state_3_T:
	call 	LCD_write_state_T
	jmp 	LCD_write_state_return
LCD_write_state_3_I:
	call 	LCD_write_state_I
	jmp 	LCD_write_state_return

LCD_write_state_return:
	ret

LCD_write_state_C:
	ldi		ZL,LOW(LCD_state_clear*2)
	ldi		ZH,HIGH(LCD_state_clear*2)
	call	LCD_Text
	ret

LCD_write_state_T:
	ldi		ZL,LOW(LCD_state_triggered*2)
	ldi		ZH,HIGH(LCD_state_triggered*2)
	call	LCD_Text
	ret

LCD_write_state_I:
	ldi		ZL,LOW(LCD_state_isolated*2)
	ldi		ZH,HIGH(LCD_state_isolated*2)
	call	LCD_Text
	ret

LCD_write_alert_countdown:
	cpi 	r28,0x03
	brne 	LCD_write_state_alert_2
	ldi 	r25,0x0F
	call 	LCD_Position
	ldi 	ZL,LOW(LCD_timer_3*2)
	ldi 	ZH,HIGH(LCD_timer_3*2)
	call 	LCD_Text
	jmp 	LCD_write_state_0
LCD_write_state_alert_2:
	cpi 	r28,0x02
	brne 	LCD_write_state_alert_1
	ldi 	r25,0x0F
	call 	LCD_Position
	ldi 	ZL,LOW(LCD_timer_2*2)
	ldi 	ZH,HIGH(LCD_timer_2*2)
	call 	LCD_Text
	jmp 	LCD_write_state_0
LCD_write_state_alert_1:
	cpi 	r28,0x01
	brne 	LCD_write_state_alert_0
	ldi 	r25,0x0F
	call 	LCD_Position
	ldi 	ZL,LOW(LCD_timer_1*2)
	ldi 	ZH,HIGH(LCD_timer_1*2)
	call 	LCD_Text
	jmp 	LCD_write_state_0
LCD_write_state_alert_0:
	cpi 	r28,0x00
	ret
	ldi 	r25,0x0F
	call 	LCD_Position
	ldi 	ZL,LOW(LCD_timer_0*2)
	ldi 	ZH,HIGH(LCD_timer_0*2)
	call 	LCD_Text
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; GENERAL HELPER ROUTINES
; 
; Routines for general IO and tasks
; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;
; Send a command + byte to SPI interface
; CMD is in r20, DATA is in r21
; r16 is destroyed by this subroutine
SPI_Send_Command:
	cbi		PORTB,2		; SS low
	ldi		r16,0x40
	call	SPI_SendByte
	mov		r16,r20
	call	SPI_SendByte
	mov		r16,r21
	call	SPI_SendByte
	sbi		PORTB,2		; and SS back high
	ret
; Send a command + byte to SPI interface
; CMD is in r20, DATA is in r21 (if necessary)
;
SPI_Read_Command:
	cbi		PORTB,2		; SS low
	ldi		r16,0x41
	call	SPI_SendByte
	mov		r16,r20
	call	SPI_SendByte
	mov		r16,r21
	call	SPI_SendByte
	sbi		PORTB,2		; and SS back high
	;cpi 	r16,0x00
	;breq 	SPI_Read_Command
	ret
;
; Send one SPI byte (Returned data in r16)
;
SPI_SendByte:
	out		SPDR,r16
SPI_wait:
	in		r16,SPSR
	sbrs	r16,SPIF
	rjmp	SPI_wait
	in		r16,SPDR
	ret


;
; LCD Position - set the write position in the DRAM
; r24 holds the LCD I2C address
; r25 holds the address (0-127)
; r17 holds the lower 4 bits
;
LCD_Position:
	push 	r16
	push 	r17
	call	sendTWI_Start

	mov		r16,r25
	ori		r16,0x80		; set DDRAM address command
	ldi		r17,8			; backlight
	call	sendTWI_Byte
	
	pop 	r17
	pop 	r16
	rjmp	sendTWI_Stop
	
;
; LCD Clear - Clears the LCD and places the cursor at location 0
; r24 holds the LCD I2C address
; r17 holds the lower 4 bits
;
LCD_Clear:
	push 	r16
	push 	r17
	call	sendTWI_Start

	ldi		r16,0x01		; set DDRAM address command
	ldi		r17,8			; backlight
	call	sendTWI_Byte
	
	pop 	r17
	pop 	r16
	rjmp	sendTWI_Stop
;
; LCD_Text - send a string to the LCD for displaying
; Z points to the string,
; r24 is the address of the LCD
;
LCD_Text:
	push 	r16
	push 	r17
	call	sendTWI_Start

	ldi		r17,9			; backlight + data byte
LCD_Text_loop:
	lpm		r16,Z+

	cpi 	r16,0x00
	breq	LCD_Text_done

	call	sendTWI_Byte
	brne	LCD_serror
	jmp 	LCD_Text_loop

	

LCD_Text_done:
LCD_serror:
	pop 	r17
	pop 	r16
	rjmp	sendTWI_Stop
;
; LCDSetup - setup the LCD display connected at I2C port in r24
;
LCD_Setup:
	push 	r18
	call	sendTWI_Start
	clr		r18
	call	sendTWI_Nibble
	call	sendTWI_Stop
	
;
; Send the first of three 0x30 to the display
;
	call	sendTWI_Start
	ldi		r18,0x30
	call	sendTWI_Nibble
	call	sendTWI_Stop
	
;
; Send the second of three 0x30 to the display
;
	call	sendTWI_Start
	ldi		r18,0x30
	call	sendTWI_Nibble
	call	sendTWI_Stop
	
;
; Send the third of three 0x30 to the display
;
	call	sendTWI_Start
	ldi		r18,0x30
	call	sendTWI_Nibble
	call	sendTWI_Stop

	
;
; Send 0x28 to the display to reset to 4 bit mode
;
	call	sendTWI_Start
	ldi		r18,0x28
	call	sendTWI_Nibble
	call	sendTWI_Stop

	pop 	r18
	ret

;
; Send TWI start address.
; On return Z flag is set if completed correctly
; r15 and r16 destroyed
sendTWI_Start:
	push 	r16
	ldi		r16,(1<<TWINT) | (1<<TWSTA) | (1<<TWEN)
	sts		TWCR,r16

	call	waitTWI

	lds		r16,TWSR
	andi	r16,0xf8		; mask out 
	cpi		r16,0x08		; TWSR = START (0x08)
	brne	sendTWI_Start_err

	mov		r16,r24			; use this address
	add		r16,r16			; and move over the r/w bit
	sts		TWDR,r16
	ldi		r16,(1<<TWINT) | (1<<TWEN)
	sts		TWCR,r16

	call	waitTWI
	
	lds		r16,TWSR
	andi	r16,0xf8		; mask out 
	cpi		r16,0x18		; TWSR = SLA+W sent, ACK received (0x18)
	brne	sendTWI_Start_err
	pop 	r16
	ret
sendTWI_Start_err:
	pop 	r16
	ret
;
; Send 8 bits of data as two 4 bit nibbles.
; The data is in r16, the lower 4 bits are in r17
; we assume the TWI operation is waiting for data to be sent.
; r15, r18 and r19 all destroyed
sendTWI_Byte:
	push 	r18
	mov		r18,r16
	andi	r18,0xF0
	or		r18,r17
	call	sendTWI_Nibble
	mov		r18,r16
	swap	r18
	andi	r18,0xF0
	or		r18,r17
	call	sendTWI_Nibble
	pop 	r18
	ret

;
; send 4 bits of data, changing the enable bit as we send it.
; data is in r18. r15, r18 and r19 are destroyed
;
sendTWI_Nibble:
	push 	r19
	ori		r18,0x04
	sts		TWDR,r18
	ldi		r19,(1<<TWINT) | (1<<TWEN)
	sts		TWCR,r19

	call	waitTWI			; destroys r15
	
	lds		r19,TWSR
	andi	r19,0xf8		; mask out 
	cpi		r19,0x28		; TWSR = data sent, ACK received (0x28)
	brne	sendTWI_Nibble_exit

	andi	r18,0xFB		; set enable bit low
	
	sts		TWDR,r18
	ldi		r19,(1<<TWINT) | (1<<TWEN)
	sts		TWCR,r19

	call	waitTWI
	
	lds		r19,TWSR
	andi	r19,0xf8		; mask out 
	cpi		r19,0x28		; TWSR = data sent, ACK received (0x28)
sendTWI_Nibble_exit:
	pop 	r19
	ret

;
;	Send the data pointed to by the Z register to the TWI interface.
;	r25 contains the number of bytes to send
;	r24 contains the address of the I2C controller
;	r17 contains the lower 4 bits of each nibble to send
;
SendTWI_Data:
	push 	r16
	call	sendTWI_Start

	cpi		r25,0x00		; any bytes left?
	breq	sendTWI_done	; if not all done
sendTWI_loop:
	lpm		r16,Z+
	call	sendTWI_Byte
	brne	serror

	dec		r25
	brne	sendTWI_loop
	pop 	r16
sendTWI_done:
serror:
;
; send stop bit and we're done
;
sendTWI_Stop:
	push 	r16
	ldi		r16,(1<<TWINT) | (1<<TWEN) | (1<<TWSTO)		; and send stop
	sts		TWCR,r16
	ldi		r16,0
sendTWI_Delay:
	dec		r16
	brne	sendTWI_Delay
	pop 	r16
	ret
;
; Wait until the TWI (I2C) interface has sent the byte and received an ack/nak
; destroys r15
;
waitTWI:
	push 	r15
waitTWI_loop:
	lds		r15,TWCR
	sbrs	r15,TWINT		; wait until transmitted
	rjmp	waitTWI_loop
	pop 	r15
	ret

Str_NL:
	.db	0x0a, 0x0d, 0x00, 0x00

;
;	Send the string pointed to by Z register
;
puts:
	push	r16
puts0:
	lpm		r16,Z+
	tst		r16
	breq	puts1
	call	putc	; send the character in r16
	rjmp	puts0	; and loop until end of string
puts1:			; finished the string
	pop		r16
	ret
;
;	Send the character in r16
;
putc:
	push	r17
putc0:
	lds		r17,UCSR0A
	andi	r17,0x20	; check if there is space to put the character in the buffer
	breq	putc0
	sts		UDR0,r16
	pop		r17
	ret
;
;	Send the byte in r16 as a hexadecimal number
;
puth:
	push	r17
	push	r16
	swap	r16
	call	putn
	pop	 	r16
	call	putn
	pop		r17
	ret
;
;	send the lower 4 bits in r16 as a hexadecimal character
;
putn:
	push	r16
	andi	r16,0x0f
	cpi	 	r16,10
	brlt	putn0
	subi	r16,-7
putn0:
	subi	r16,-48
	call	putc
	pop	 	r16
	ret

putNewLine:
	ldi 	ZL,LOW(Str_NL*2)
	ldi 	ZH,HIGH(Str_NL*2)
	call	puts
	ret
