;Asm code 7-segment driver for 74HC595 

RCC_IOPENR		EQU		0x4002102c
GPIOA_BASE		EQU		0x50000000
GPIOA_MODER		EQU		0x50000000
GPIOA_OTYPER	EQU		0x50000004
GPIOA_OSPEEDR	EQU		0x50000008
GPIOA_ODR		EQU		0x50000014
GPIOB_BASE		EQU		0x50000400
RCC_CR			EQU		0x40021000
RCC_CFGR		EQU		0x4002100C
SEGMENT_DATA	EQU		0x20000000	;0x20000000 - 0x2000000F
DLY_MS			EQU		0x20000010	;0x20000010 - 0x20000014
SYSTICK			EQU		0x20000030

	AREA		RESET,DATA,READONLY
__Vectors
	DCD	0x20084000
	DCD	START
	ALIGN
		
	AREA	MYCODE, CODE, READONLY
	ENTRY
;=================================================		
;Setup section	
;=================================================
START
	BL	init_array
	BL	setup_clock		;Enable clock for GPIOA	
	BL	enable_clock_gpio
	BL	setup_mode		;PA5 as output
	BL	setup_otype		;Push-pull
	BL	setup_ospeed	;very slow	
	BL	setup_uart2
	BL	d1_off
	BL	d2_off
	BL	d3_off
	BL	d4_off
	BL	buz_off
	
	;Display OK 
	MOVS	R7, #'O'
	BL		uart_tx
	MOVS	R7, #'K'
	BL		uart_tx
	MOVS	R7, #'\n'
	BL		uart_tx				
;=================================================
;Loop section
; r7 use to passing param to subroutine
;=================================================
;Example1: 
;Press S1, D1 blinking else D1 stop blinking
;Press S2, decrease value 7-segment
;Press S3, increase value 7-segment
AGAIN
	;Read S1 state
	LDR		r5,=GPIOA_BASE
	LDR		r6,[r5,#0x10]	;0x10-IDR
	LSRS	r6, r6, #1
	MOVS	r0,#0x01
	ANDS	r6,r0,r6		;masking
	CMP		r6,#0			;check it press or not
	BEQ		blink_d1

	;Read S2 state
	LDR		r5,=GPIOA_BASE
	LDR		r6,[r5,#0x10]	;0x10-IDR
	LSRS	r6, r6, #4
	MOVS	r0,#0x01
	ANDS	r6,r0,r6		;masking
	CMP		r6,#0			;check it press or not
	BEQ		count_down
	
	;Read S3 state
	LDR		r5,=GPIOB_BASE
	LDR		r6,[r5,#0x10]	;0x10-IDR
	;LSRS	r6, r6, #0
	MOVS	r0,#0x01
	ANDS	r6,r0,r6		;masking
	CMP		r6,#0			;check it press or not
	BEQ		count_up
	
	B		AGAIN
	
blink_d1
	BL		d1_on
	ldr		r0,=DLY_MS
	movs	r1,#100
	str		r1,[r0]
	BL		delay
	BL		d1_off
	BL		delay
	B		AGAIN			;Branch to AGAIN

count_up
	LDR		r0,=0x2000000E
	LDRB	r1,[r0]
	CMP		r1, #9
	BLT		increase_r1
	MOVS	R1, #9
	B		store_value
increase_r1
	ADDS	r1, #1
	B		store_value
	
count_down
	LDR		r0,=0x2000000E
	LDRB	r1,[r0]
	CMP		r1,#0
	BGT		decrease_r1
	MOVS	r1, #0
	B		store_value
decrease_r1	
	SUBS	r1,#1
	B		store_value
	
store_value
	STRB	r1,[r0]
	MOVS	r1,#0
	STRB	r1,[r0,#1]
	B		WriteNumberToSegment 
return_WriteNumberToSegment 
	ldr		r0,=DLY_MS
	movs	r1,#200
	str		r1,[r0]
	BL		delay
	B		AGAIN			;Branch to AGAIN
;====================================================
;Subroutine section
;====================================================

;====================================================
;
;====================================================
init_array
;0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90
; 0  , 1  , 2  , 3  , 4  , 5  , 6  , 7  , 8  , 9
	LDR		R0, =0x20000000    ; Load address of array_values into R0
	MOVS	R1, #0xC0
	STRB	R1,[R0]
	MOVS	R1, #0xF9
	STRB	R1,[R0,#1]
	MOVS	R1, #0xA4
	STRB	R1,[R0,#2]
	MOVS	R1, #0xB0
	STRB	R1,[R0,#3]	
	MOVS	R1, #0x99
	STRB	R1,[R0,#4]	
	MOVS	R1, #0x92
	STRB	R1,[R0,#5]	
	MOVS	R1, #0x82
	STRB	R1,[R0,#6]	
	MOVS	R1, #0xF8
	STRB	R1,[R0,#7]
	MOVS	R1, #0x80
	STRB	R1,[R0,#8]	
	MOVS	R1, #0x90
	STRB	R1,[R0,#9]	
;0xF1,0xF2,0xF4,0xF8	
	MOVS	R1, #0xF1
	STRB	R1,[R0,#10]	;1st segment
	MOVS	R1, #0xF2	
	STRB	R1,[R0,#11]	;2nd segment
	MOVS	R1, #0xF4
	STRB	R1,[R0,#12]	;3rd segment
	MOVS	R1, #0xF8
	STRB	R1,[R0,#13]	;4th segment	
	BX 		LR
;====================================================
;==============================================
;7-segment Driver
;==============================================
;Data at R7 will be shift out
;void shiftOut(uint8_t data){
;uint8_t i;
;  for (i = 0; i < 8; i++)  {
;		if(data & ((1 << (7 - i))))
;			GPIOA->ODR |= (1UL<<9);		//SDI=HIGH
;		else
;			GPIOA->ODR &= ~(1UL<<9);	//SDI=LOW

;		GPIOA->ODR |= (1UL<<8);		//clkPin=HIGH
;		GPIOA->ODR &= ~(1UL<<8);	//clkPin=LOW
;  }
;}

shiftOut
    MOVS    R2, #0          ; Initialize i to 0
shiftOut_loop
    CMP     R2, #8          ; Compare i with 8
    BGE     shiftOut_done   ; If i >= 8, exit loop

    MOVS    R1, #7          ; Load 7 into R1
    SUBS    R1, R1, R2      ; R1 = 7 - i
    MOVS    R3, #1          ; Prepare 1
    LSLS    R3, R3, R1      ; 1 << (7 - i)
    TST     R7, R3          ; Test if (data & (1 << (7 - i)))

    BEQ     shiftOut_clear  ; If zero, clear SDI (PA9)	
	
shiftOut_set
    LDR     R3, =0x50000014 ; Address of GPIOA->ODR (Output Data Register)
    LDR     R1, [R3]        ; Load ODR value
	MOVS	R0, #0x01
	LSLS	R0, R0, #9
    ORRS     R1, R1, R0   ; Set bit 9 (SDI=HIGH)
    STR     R1, [R3]        ; Write back to ODR
    B       shiftOut_clock  ; Jump to clock sequence	
	
shiftOut_clear
    LDR     R3, =0x50000014 ; Address of GPIOA->ODR
    LDR     R1, [R3]        ; Load ODR value
	MOVS	R0, #0x01
	LSLS	R0, R0, #9	
    BICS     R1, R1, R0   ; Clear bit 9 (SDI=LOW)
    STR     R1, [R3]        ; Write back to ODR

shiftOut_clock
    ; Set clock pin (PA8) high
    LDR     R3, =0x50000014 ; Address of GPIOA->ODR
    LDR     R1, [R3]        ; Load ODR value
	MOVS	R0,#0x01
	LSLS	R0, R0, #8
    ORRS     R1, R1, R0   ; Set bit 8 (clkPin=HIGH)
    STR     R1, [R3]        ; Write back to ODR

    ; Set clock pin (PA8) low
    LDR     R1, [R3]        ; Load ODR value
	MOVS	R0,#0x01
	LSLS	R0, R0, #8
    BICS    R1, R1, R0   ; Clear bit 8 (clkPin=LOW)
    STR     R1, [R3]        ; Write back to ODR

    ADDS     R2, R2, #1      ; i++
    B       shiftOut_loop   ; Repeat loop

shiftOut_done
    BX      LR              ; Return

;========================================================
;
;========================================================
;void WriteNumberToSegment (uint8_t digit) {
;GPIOB -> ODR &= ~(1UL<<5);	//latchPin=LOW
;shiftOut (0xF9);
;shiftOut (0xF1);
;GPIOB -> ODR |= (1UL<<5);	//latchPin=HIGH
;}

WriteNumberToSegment
    ; Clear latchPin (PB5) - Set PB5 to LOW
    LDR     R1, =0x50000414  ; Address of GPIOB->ODR
    LDR     R2, [R1]         ; Load ODR value of GPIOB
	MOVS	R0, #0x01
	LSLS	R0,R0, #5
    BICS    R2, R2, R0 		; Clear bit 5 (latchPin=LOW)
    STR     R2, [R1]         ; Write back to ODR
	
	;0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,0xF1,0xF2,0xF4,0xF8,v,s
	;  0 ,  1 , 2  , 3  , 4  ,  5 ,  6 ,  7 ,  8 ,  9 , A  , B  , C  , D  ,E,F
	LDR		R5, =0x2000000E
	LDRB	R0,[R5]
	LDR     R5, =0x20000000
	LDRB    R7, [R5,R0]
	
    ;MOVS    R7, #0xB0        ; Move 0xF9 into R7 (data)
    BL      shiftOut         ; Call shiftOut subroutine

	;0xF1,0xF2,0xF4,0xF8
	;1st , 2nd, 3rd, 4th segment
	LDR		R5, =0x2000000F
	LDRB	R0,[R5]	
	LDR		R5,=0x2000000A
    LDRB    R7, [R5,R0]        ; Move 0xF1 into R7 (data)
    BL      shiftOut         ; Call shiftOut subroutine

    ; Set latchPin (PB5) - Set PB5 to HIGH
    LDR     R1, =0x50000414  ; Address of GPIOB->ODR
    LDR     R2, [R1]         ; Load ODR value of GPIOB
	MOVS	R0, #0x01
	LSLS	R0,R0, #5	
    ORRS    R2, R2, R0 ; Set bit 5 (latchPin=HIGH)
    STR     R2, [R1]         ; Write back to ODR
    B      return_WriteNumberToSegment  ; Return from function

;=========================================================
;Subroutine control LEDs and Buzzer
;=========================================================
d1_off
	LDR		R5,=GPIOA_ODR
	LDR		R6,[R5]
	MOVS	R0,#0x01
	LSLS	R0,R0, #5	;ODR5=1
	ORRS	R6,R6,R0
	STR		R6,[R5]
	BX		LR
	
d1_on
	LDR		R5,=GPIOA_ODR
	LDR		R6,[R5]
	MOVS	R0,#0x01
	LSLS	R0,R0,#5
	BICS	R6,R6,R0
	STR		R6,[R5]	
	BX		LR

d2_off
	LDR		R5,=GPIOA_ODR
	LDR		R6,[R5]
	MOVS	R0,#0x01
	LSLS	R0,R0, #6	;ODR6=1
	ORRS	R6,R6,R0
	STR		R6,[R5]
	BX		LR

d2_on
	LDR		R5,=GPIOA_ODR
	LDR		R6,[R5]
	MOVS	R0,#0x01
	LSLS	R0,R0,#6
	BICS	R6,R6,R0
	STR		R6,[R5]	
	BX		LR
	
d3_off
	LDR		R5,=GPIOA_ODR
	LDR		R6,[R5]
	MOVS	R0,#0x01
	LSLS	R0,R0, #7	;ODR7=1
	ORRS	R6,R6,R0
	STR		R6,[R5]
	BX		LR

d3_on
	LDR		R5,=GPIOA_ODR
	LDR		R6,[R5]
	MOVS	R0,#0x01
	LSLS	R0,R0,#7
	BICS	R6,R6,R0
	STR		R6,[R5]	
	BX		LR	
	
d4_off
	LDR		R5,=GPIOB_BASE
	LDR		R6,[R5,#0x14]
	MOVS	R0,#0x01
	LSLS	R0,R0, #6	;ODR6=1
	ORRS	R6,R6,R0
	STR		R6,[R5,#0x14]
	BX		LR

d4_on
	LDR		R5,=GPIOB_BASE
	LDR		R6,[R5,#0x14]
	MOVS	R0,#0x01
	LSLS	R0,R0,#6
	BICS	R6,R6,R0
	STR		R6,[R5,#0x14]	
	BX		LR	

buz_off
	LDR		R5,=GPIOB_BASE
	LDR		R6,[R5,#0x14]
	MOVS	R0,#0x01
	LSLS	R0,R0, #3	;ODR6=1
	ORRS	R6,R6,R0
	STR		R6,[R5,#0x14]
	BX		LR

buz_on
	LDR		R5,=GPIOB_BASE
	LDR		R6,[R5,#0x14]
	MOVS	R0,#0x01
	LSLS	R0,R0,#3
	BICS	R6,R6,R0
	STR		R6,[R5,#0x14]	
	BX		LR
;====================================================
;Subroutine Utilities: Delay
;====================================================
;Subroutine: Delay 1 ms
delay
	LDR		R0, =0x00001f78
	LDR		R1,=DLY_MS
	LDR		R7,[R1]
	MULS	R0,R7,R0
loop	
	SUBS	R0,#1
	BNE		loop
	LDR		R5, =SYSTICK
	LDR		R6, [R5]
	ADDS	R6,R6,#1
	STR		R6, [R5]
	BX		LR

;======================================
; Subroutine Initialize Peripheral: Sys Clock, I/O, USART
;======================================
; system clock = HSI16 x 3 / 2 = 24MHz
setup_clock
	; Enable HSI & Turn ON PLL
	LDR		R5, =RCC_CR
	LDR		R6, [R5]
	MOVS	R1, #0x01
	ORRS	R6,R6,R1	;set bit0-Use HSI
	STR		R6, [R5]
wait_hsi_ready
	LDR		R6,[R5, #0x00]
	MOVS    R2, #0x04	;Prepare value to check HSIRDY bit
	ANDS    R2, R2, R6	;AND with HSIRDY bit mask
	BEQ     wait_hsi_ready  ;If not set, loop */
	
	;Configure PLL
	LDR		R5, =RCC_CFGR
	LDR		R6, [R5]
	MOVS	R2, #0x01
	LSLS	R2, #16
	BICS	R6, R6, R2	;Clear PLLSRC - HSI as PLL input
	MOVS	R2, #0x07
	LSLS	R2,	#18
	BICS	R6,R6,R2	; Clear PLLMUL bits	x3
	MOVS	R2, #0x01
	LSLS	R2, #22
	ORRS	R6, R6, R2	; PLLVCO/2 
	STR		R6, [R5]
	
	;Enable PLL		
	LDR		R5, =RCC_CR
	LDR		R6, [R5]
	MOVS	R2, #0x01
	LSLS	R2, #24
	ORRS	R6,R6,R2	; Set PLLON bit 
	STR		R6, [R5]
wait_pll_ready
	LDR		R5, =RCC_CR
	LDR		R6, [R5]
	MOVS	R2, #0x02	; Prepare value to check PLLRDY bit
	LSLS	R2, #24
	ANDS	R2, R2, R6	; AND with PLLRDY bit mask
	BNE     wait_pll_ready	;If equal, PLLRDY bit is set, loop

	; sys clk = 16MHz x 3 / 2 = 24MHz
	; Select PLL as system clock source
	LDR		R5, =RCC_CFGR
	LDR		R6, [R5]
	MOVS	R2, #0x03
	ORRS	R6, R6, R2	; Set SW bits to select PLL
	STR		R6, [R5]
	
	;Configure peripheral clocks
	LDR		R5, =RCC_CR
	LDR		R6, [R5, #0x38]	;Read RCC_APB1ENR register
	MOVS	R2, #0x01
	LSLS	R2, #17			;Enable USART2 clock
	ORRS	R6, R6, R2
	STR		R6, [R5, #0x38] 
	
	BX		LR
	
;===============================================	
;Subroutine: Initialize GPIOA, GPIOB as Output 
;===============================================
;Step 1:Enable clock for GPIOA
enable_clock_gpio

	LDR		R5, =RCC_IOPENR
	LDR		R6, [R5]
	MOVS	R0, #0x03
	ORRS	R6,R6, R0	;Enable PA, PB
	STR		R6, [R5]
	
	BX		LR

;0x00 - MODER
;0x04 - OTYPER
;0x08 - OSPEEDR
;0x10 - IDR
;0x14 - ODR
;Step 2: Initialize PA5,PA6&PA7 as Output
setup_mode
	;mode-output
	LDR		R5, =GPIOA_MODER
	LDR		R6,[R5]
	;PA5-Led1, PA6-Led2, PA7-Led3
	MOVS	R0,#0x3F
	LSLS	R0,R0,#10
	BICS	R6,R6,R0
	MOVS	R0,#0x15
	LSLS	R0,R0, #10
	ORRS	R6,R6,R0
	;PA9-Data pin, PA8-Clk pin
	MOVS	R0, #0x0F
	LSLS	R0,R0,#16
	BICS	R6,R6,R0
	MOVS	R0, #0x05
	LSLS	R0,R0,#16
	ORRS	R6,R6,R0
	
	STR		R6,[R5]
	;mode-input
	LDR		R5, =GPIOA_MODER
	LDR		R6,[R5]
	MOVS	R0, #0x03
	LSLS	R0,R0,#2
	BICS	R6,R6,R0
	
	MOVS	R0, #0x03
	LSLS	R0,R0,#8
	BICS	R6,R6,R0	
	STR		R6,[R5]
	
	;PB6-LED D4, PB3-BUZ
	;mode-output
	LDR		R5,=GPIOB_BASE
	LDR		R6, [R5,#0x00]	;MODER
	;PB6-Led4, PB3-Buzzer
	MOVS	R0, #0x03
	LSLS	R0,R0,#12
	BICS	R6,R6,R0
	MOVS	R0, #0x03
	LSLS	R0,R0,#6
	BICS	R6,R6,R0	
	MOVS	R0, #0x01
	LSLS	R0, R0, #12
	ORRS	R6,R6,R0
	MOVS	R0, #0x01
	LSLS	R0, R0, #6
	ORRS	R6,R6,R0	
	
	;PB5-Latch Pin
	MOVS	R0, #0x03
	LSLS	R0,R0,#10
	BICS	R6,R6,R0
	MOVS	R0, #0x01
	LSLS	R0,R0,#10
	ORRS	R6,R6,R0
	
	STR		R6, [R5,#0x00]	;MODER
	
	;mode-input
	LDR		R5,=GPIOB_BASE
	LDR		R6, [R5,#0x00]	;MODER
	MOVS	R0, #0x03
	LSLS	R0,R0,#0
	BICS	R6,R6,R0	
	STR		R6, [R5,#0x00]	;MODER
	BX		LR

;Step 3: OTYPER
;		Open Drain :PA6, PA7, PB6
;		Push/Pull : PA5, PA8, PA9, PB5
setup_otype
	LDR		R5, =GPIOA_OTYPER
	LDR		R6, [R5]
	;PA6,PA7 - Open Drain
	MOVS	R0,#0x03
	LSLS	R0,R0, #6
	ORRS	R6,R6,R0
	; PA5, PA8, PA9 - Push/pull
	MOVS	R0, #0x19
	LSLS	R0, R0, #5
	BICS	R6,R6, R0
	STR		R6,[R5]
	
	LDR		R5, =GPIOB_BASE
	LDR		R6,[R5,#0x04]	;OTYPER
	;PB6 - open drain
	MOVS	R0,#0x01
	LSLS	R0,R0, #6		
	ORRS	R6,R6,R0
	;PB3 - open drain
	MOVS	R0,#0x01
	LSLS	R0,R0, #3		
	ORRS	R6,R6,R0
	;PB5 - push/pull
	MOVS	R0, #0x01
	LSLS	R0, R0, #5
	BICS	R6,R6, R0
	
	STR		R6,[R5,#0x04]	
	BX		LR
	
;Step 4: OSPEED=VeryLow
setup_ospeed
	LDR		R5, =GPIOA_OSPEEDR
	LDR		R6,[R5]
	MOVS	R0, #0x3F
	LSLS	R0,R0, #10
	BICS	R6,R6, R0
	STR		R6,[R5]
	
	LDR		R5, =GPIOB_BASE
	LDR		R6,[R5,#0x08]
	MOVS	R0, #0x03
	LSLS	R0,R0, #12
	BICS	R6,R6, R0
	
	MOVS	R0, #0x03
	LSLS	R0,R0, #6
	BICS	R6,R6, R0
	
	STR		R6,[R5,#0x08]	
	BX		LR

;=============================================
; Initialize USART2, Baud Rate = 115200bps
;=============================================
setup_uart2

	; Enable USART2 clock by setting the APB1ENR register in RCC
	LDR     R0, =0x40021000       ; RCC base address
	LDR     R1, [R0, #0x38]       ; Read RCC_APB1ENR (correct offset)

	; Split large immediate value into smaller parts
	MOVS    R2, #0x01             ; Load 0x01 into R2
	LSLS    R2, #17               ; Shift it to bit position 17
	ORRS    R1, R1, R2            ; Set bit 17 to enable USART2 clock

	STR     R1, [R0, #0x38]       ; Write back to RCC_APB1ENR

	; Configure PA2 (USART2_TX) and PA3 (USART2_RX) as alternate function
	LDR     R0, =0x50000000       ; Corrected GPIOA base address
	LDR     R1, [R0, #0x00]       ; Read GPIOA_MODER

	; Clear PA2 and PA3 mode bits (using smaller immediate values)
	MOVS    R2, #0xF              ; Load 0xF into R2
	LSLS    R2, #4                ; Shift it to bits 4-7
	BICS    R1, R1, R2            ; Clear bits 4-7 for PA2, PA3

	; Set PA2 and PA3 to alternate function mode
	MOVS    R2, #0xA              ; Load 0xA (binary 1010) into R2
	LSLS    R2, #4                ; Shift it to bits 4-7
	ORRS    R1, R1, R2            ; Set PA2 and PA3 to alternate function mode
	STR     R1, [R0, #0x00]       ; Write back to GPIOA_MODER
	
	; Set PA2, PA3 alternate function to USART2 (AF4)
	LDR     R1, [R0, #0x20]       ; Read GPIOA_AFRL
	MOVS	R2, #0xFF
	LSLS	R2, #8
	BICS	R1, R1, R2			; Clear alternate function bits for PA2, PA3
	MOVS	R2, #0x44
	LSLS	R2, #8
	ORRS	R1, R1, R2			; Set AF4 for PA2 and PA3 (USART2) 
	STR     R1, [R0, #0x20]       ; Write back to GPIOA_AFRL		

	; Configure USART2 for 115200 baud, 8-bit word length, 1 stop bit, no parity
	LDR     R0, =0x40004400       ; USART2 base address

	; Configure baud rate (115200, assuming 24 MHz clock)
	LDR     R1, =0xD0             ; Baud rate register value for 115200 baud
	STR     R1, [R0, #0x0C]       ; Write to USART_BRR (0x0C offset)
	
	; Set word length to 8 bits, no parity, enable TX and RX
	LDR     R1, [R0, #0x00]       ; Read USART_CR1
	MOVS	R2, #0x01
	LSLS	R2, #28
	BICS    R1, R1, R2    ; Clear M1 for 8-bit word length
	MOVS	R2, #0x01
	LSLS	R2, #12
	BICS    R1, R1, R2    ; Clear PCE for no parity
	MOVS	R2, #0x01
	LSLS	R2, #2
	ORRS    R1, R1, R2     ; Enable receiver
	MOVS	R2, #0x01
	LSLS	R2, #3
	ORRS    R1, R1, R2     ; Enable transmitter
	MOVS	R2, #0x01
	ORRS    R1, R1, R2     ; Enable USART
	STR     R1, [R0, #0x00]       ; Write back to USART_CR1		

	; Set 1 stop bit
	LDR     R1, [R0, #0x04]       ; Read USART_CR2
	MOVS	R2, #0x03
	LSLS	R2, #12
	BICS     R1, R1, R2    ; Clear stop bits (STOP[13:12])
	STR     R1, [R0, #0x04]       ; Write back to USART_CR2	

	; No hardware flow control, 16x oversampling
	LDR     R1, [R0, #0x08]       ; Read USART_CR3
	MOVS	R2, #0x01
	LSLS	R2, #15
	BICS    R1, R1, R2    		; Clear ONEBIT for 16x oversampling
	STR     R1, [R0, #0x08]     ; Write back to USART_CR3
	
	BX		LR

;=============================================
;Subroutine to Transmit data through USART (Tx)
;=============================================
uart_tx
; Transmit the character from r7
	LDR     R0, =0x40004400       ; USART2 base address
	;MOVS    R1, #'a'             ; Load 'a' into R1

wait_for_tx
	LDR     R2, [R0, #0x1C]       ; Read USART_SR (Status Register at offset 0x1C)
	MOVS	R3, #0x80
	ANDS    R2, R2, R3         	; Check TXE bit (bit 7)
	BEQ     wait_for_tx           ; If TXE bit is not set, loop and wait

	STR     R7, [R0, #0x28]       ; Transmit 'a' (USART_TDR register at offset 0x28)
	BX		LR

	
	END
