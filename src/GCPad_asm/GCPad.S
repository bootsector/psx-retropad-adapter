	.file	"GCPad.cpp"
__SREG__ = 0x3f
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__CCP__  = 0x34
__tmp_reg__ = 0
__zero_reg__ = 1
	.global __do_copy_data
	.global __do_clear_bss
	.section	.text._Z10GCPad_datav,"ax",@progbits
.global	_Z10GCPad_datav
	.type	_Z10GCPad_datav, @function
_Z10GCPad_datav:
	push r16
	push r17
	push r28
	push r29
/* prologue: function */
/* frame size = 0 */
	ldi r30,lo8(gc_joy_data)
	ldi r31,hi8(gc_joy_data)
	ldi r20,lo8(7)
	ldi r21,hi8(7)
	ldi r28,lo8(0)
	ldi r29,hi8(0)
	ldi r16,lo8(1)
	ldi r17,hi8(1)
	rjmp .L2
.L6:
	ld r24,X
	ld r18,Z
	tst r24
	breq .L3
	movw r24,r16
	mov r0,r20
	rjmp 2f
1:	lsl r24
	rol r25
2:	dec r0
	brpl 1b
	or r18,r24
	rjmp .L10
.L3:
	movw r24,r16
	mov r0,r20
	rjmp 2f
1:	lsl r24
	rol r25
2:	dec r0
	brpl 1b
	com r24
	and r18,r24
.L10:
	st Z,r18
	subi r20,lo8(-(-1))
	sbci r21,hi8(-(-1))
	sbrs r21,7
	rjmp .L5
	ldi r20,lo8(7)
	ldi r21,hi8(7)
.L5:
	subi r22,lo8(-(1))
	sbci r23,hi8(-(1))
	adiw r26,1
	cpi r22,8
	cpc r23,__zero_reg__
	brne .L6
	adiw r28,8
	adiw r30,1
	cpi r28,64
	cpc r29,__zero_reg__
	breq .L7
.L2:
	movw r26,r28
	subi r26,lo8(-(raw_joy_data))
	sbci r27,hi8(-(raw_joy_data))
	ldi r22,lo8(0)
	ldi r23,hi8(0)
	rjmp .L6
.L7:
	ldi r24,lo8(gc_joy_data)
	ldi r25,hi8(gc_joy_data)
/* epilogue start */
	pop r29
	pop r28
	pop r17
	pop r16
	ret
	.size	_Z10GCPad_datav, .-_Z10GCPad_datav
	.section	.text._Z11N64Pad_datav,"ax",@progbits
.global	_Z11N64Pad_datav
	.type	_Z11N64Pad_datav, @function
_Z11N64Pad_datav:
	push r16
	push r17
	push r28
	push r29
/* prologue: function */
/* frame size = 0 */
	ldi r30,lo8(n64_joy_data)
	ldi r31,hi8(n64_joy_data)
	ldi r20,lo8(7)
	ldi r21,hi8(7)
	ldi r28,lo8(0)
	ldi r29,hi8(0)
	ldi r16,lo8(1)
	ldi r17,hi8(1)
	rjmp .L12
.L16:
	ld r24,X
	ld r18,Z
	tst r24
	breq .L13
	movw r24,r16
	mov r0,r20
	rjmp 2f
1:	lsl r24
	rol r25
2:	dec r0
	brpl 1b
	or r18,r24
	rjmp .L20
.L13:
	movw r24,r16
	mov r0,r20
	rjmp 2f
1:	lsl r24
	rol r25
2:	dec r0
	brpl 1b
	com r24
	and r18,r24
.L20:
	st Z,r18
	subi r20,lo8(-(-1))
	sbci r21,hi8(-(-1))
	sbrs r21,7
	rjmp .L15
	ldi r20,lo8(7)
	ldi r21,hi8(7)
.L15:
	subi r22,lo8(-(1))
	sbci r23,hi8(-(1))
	adiw r26,1
	cpi r22,8
	cpc r23,__zero_reg__
	brne .L16
	adiw r28,8
	adiw r30,1
	cpi r28,32
	cpc r29,__zero_reg__
	breq .L17
.L12:
	movw r26,r28
	subi r26,lo8(-(raw_joy_data))
	sbci r27,hi8(-(raw_joy_data))
	ldi r22,lo8(0)
	ldi r23,hi8(0)
	rjmp .L16
.L17:
	ldi r24,lo8(n64_joy_data)
	ldi r25,hi8(n64_joy_data)
/* epilogue start */
	pop r29
	pop r28
	pop r17
	pop r16
	ret
	.size	_Z11N64Pad_datav, .-_Z11N64Pad_datav
	.section	.text._ZL10GCPad_sendPhh,"ax",@progbits
	.type	_ZL10GCPad_sendPhh, @function
_ZL10GCPad_sendPhh:
/* prologue: function */
/* frame size = 0 */
	movw r30,r24
	sbi 39-32,4
	ldi r25,lo8(-128)
.L28:
	cbi 40-32,4
	ld r24,Z
	and r24,r25
	mov r18,r25
	ldi r19,lo8(0)
	breq .L22
	sbi 40-32,4
	movw r24,r18
	asr r25
	ror r24
	mov r25,r24
	tst r24
	brne .L23
	adiw r30,1
	subi r22,lo8(-(-1))
	ldi r25,lo8(-128)
	rjmp .L24
.L23:
/* #APP */
 ;  59 "GCPad.cpp" 1
	nop
nop
nop
nop
nop

 ;  0 "" 2
/* #NOAPP */
.L24:
/* #APP */
 ;  62 "GCPad.cpp" 1
	nop
nop
nop
nop

 ;  0 "" 2
/* #NOAPP */
	rjmp .L25
.L22:
/* #APP */
 ;  64 "GCPad.cpp" 1
	nop
nop
nop

 ;  0 "" 2
/* #NOAPP */
	movw r24,r18
	asr r25
	ror r24
	mov r25,r24
	tst r24
	brne .L26
	adiw r30,1
	subi r22,lo8(-(-1))
	ldi r25,lo8(-128)
	rjmp .L27
.L26:
/* #APP */
 ;  73 "GCPad.cpp" 1
	nop
nop
nop
nop
nop

 ;  0 "" 2
/* #NOAPP */
.L27:
	sbi 40-32,4
/* #APP */
 ;  78 "GCPad.cpp" 1
	nop
nop

 ;  0 "" 2
/* #NOAPP */
.L25:
	tst r22
	breq .+2
	rjmp .L28
/* #APP */
 ;  83 "GCPad.cpp" 1
	nop

 ;  0 "" 2
/* #NOAPP */
	cbi 40-32,4
/* #APP */
 ;  88 "GCPad.cpp" 1
	nop
nop
nop
nop
nop
nop

 ;  0 "" 2
/* #NOAPP */
	sbi 40-32,4
/* epilogue start */
	ret
	.size	_ZL10GCPad_sendPhh, .-_ZL10GCPad_sendPhh
	.section	.text._ZL10GCPad_recvPhh,"ax",@progbits
	.type	_ZL10GCPad_recvPhh, @function
_ZL10GCPad_recvPhh:
/* prologue: function */
/* frame size = 0 */
	movw r30,r24
	cbi 39-32,4
	sbi 40-32,4
.L40:
	sbic 38-32,4
	rjmp .L40
/* #APP */
 ;  115 "GCPad.cpp" 1
	nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop

 ;  0 "" 2
/* #NOAPP */
	in r24,38-32
	andi r24,lo8(16)
	st Z,r24
	subi r22,lo8(-(-1))
	breq .L35
.L37:
	sbis 38-32,4
	rjmp .L37
	adiw r30,1
	rjmp .L40
.L35:
	ret
	.size	_ZL10GCPad_recvPhh, .-_ZL10GCPad_recvPhh
	.section	.text._Z11N64Pad_readb,"ax",@progbits
.global	_Z11N64Pad_readb
	.type	_Z11N64Pad_readb, @function
_Z11N64Pad_readb:
	push r17
	push r29
	push r28
	push __tmp_reg__
	in r28,__SP_L__
	in r29,__SP_H__
/* prologue: function */
/* frame size = 1 */
	mov r17,r24
	tst r24
	breq .L42
/* #APP */
 ;  228 "GCPad.cpp" 1
	cli
 ;  0 "" 2
/* #NOAPP */
.L42:
	ldi r24,lo8(1)
	std Y+1,r24
	movw r24,r28
	adiw r24,1
	ldi r22,lo8(1)
	call _ZL10GCPad_sendPhh
	ldi r24,lo8(raw_joy_data)
	ldi r25,hi8(raw_joy_data)
	ldi r22,lo8(32)
	call _ZL10GCPad_recvPhh
	tst r17
	breq .L44
/* #APP */
 ;  234 "GCPad.cpp" 1
	sei
 ;  0 "" 2
/* #NOAPP */
.L44:
/* epilogue start */
	pop __tmp_reg__
	pop r28
	pop r29
	pop r17
	ret
	.size	_Z11N64Pad_readb, .-_Z11N64Pad_readb
	.section	.text._Z10GCPad_readb,"ax",@progbits
.global	_Z10GCPad_readb
	.type	_Z10GCPad_readb, @function
_Z10GCPad_readb:
	push r17
	push r29
	push r28
	rcall .
	push __tmp_reg__
	in r28,__SP_L__
	in r29,__SP_H__
/* prologue: function */
/* frame size = 3 */
	mov r17,r24
	movw r26,r28
	adiw r26,1
	ldi r30,lo8(_ZZ10GCPad_readbE4C.12)
	ldi r31,hi8(_ZZ10GCPad_readbE4C.12)
	ldi r24,lo8(3)
.L46:
	ld r0,Z+
	st X+,r0
	subi r24,lo8(-(-1))
	brne .L46
	tst r17
	breq .L47
/* #APP */
 ;  191 "GCPad.cpp" 1
	cli
 ;  0 "" 2
/* #NOAPP */
.L47:
	movw r24,r28
	adiw r24,1
	ldi r22,lo8(3)
	call _ZL10GCPad_sendPhh
	ldi r24,lo8(raw_joy_data)
	ldi r25,hi8(raw_joy_data)
	ldi r22,lo8(64)
	call _ZL10GCPad_recvPhh
	tst r17
	breq .L49
/* #APP */
 ;  197 "GCPad.cpp" 1
	sei
 ;  0 "" 2
/* #NOAPP */
.L49:
/* epilogue start */
	pop __tmp_reg__
	pop __tmp_reg__
	pop __tmp_reg__
	pop r28
	pop r29
	pop r17
	ret
	.size	_Z10GCPad_readb, .-_Z10GCPad_readb
	.section	.text._Z10GCPad_initv,"ax",@progbits
.global	_Z10GCPad_initv
	.type	_Z10GCPad_initv, @function
_Z10GCPad_initv:
	push r17
	push r29
	push r28
	push __tmp_reg__
	in r28,__SP_L__
	in r29,__SP_H__
/* prologue: function */
/* frame size = 1 */
	ldi r30,lo8(raw_joy_data)
	ldi r31,hi8(raw_joy_data)
.L52:
	st Z+,__zero_reg__
	ldi r24,hi8(raw_joy_data+64)
	cpi r30,lo8(raw_joy_data+64)
	cpc r31,r24
	brne .L52
	ldi r30,lo8(gc_joy_data)
	ldi r31,hi8(gc_joy_data)
.L53:
	st Z+,__zero_reg__
	ldi r24,hi8(gc_joy_data+8)
	cpi r30,lo8(gc_joy_data+8)
	cpc r31,r24
	brne .L53
	sts n64_joy_data,__zero_reg__
	sts n64_joy_data+1,__zero_reg__
	sts n64_joy_data+2,__zero_reg__
	sts n64_joy_data+3,__zero_reg__
/* #APP */
 ;  146 "GCPad.cpp" 1
	cli
 ;  0 "" 2
/* #NOAPP */
	std Y+1,__zero_reg__
	movw r24,r28
	adiw r24,1
	ldi r22,lo8(1)
	call _ZL10GCPad_sendPhh
	cbi 39-32,4
	sbi 40-32,4
	ldi r17,lo8(64)
.L55:
	sbis 38-32,4
	rjmp .L54
	subi r17,lo8(-(-1))
	brne .L55
.L54:
/* #APP */
 ;  155 "GCPad.cpp" 1
	sei
 ;  0 "" 2
/* #NOAPP */
	ldi r24,lo8(500)
	ldi r25,hi8(500)
	call delayMicroseconds
	mov r24,r17
/* epilogue start */
	pop __tmp_reg__
	pop r28
	pop r29
	pop r17
	ret
	.size	_Z10GCPad_initv, .-_Z10GCPad_initv
.global	raw_joy_data
	.section	.bss.raw_joy_data,"aw",@nobits
	.type	raw_joy_data, @object
	.size	raw_joy_data, 64
raw_joy_data:
	.skip 64,0
.global	gc_joy_data
	.section	.bss.gc_joy_data,"aw",@nobits
	.type	gc_joy_data, @object
	.size	gc_joy_data, 8
gc_joy_data:
	.skip 8,0
.global	n64_joy_data
	.section	.bss.n64_joy_data,"aw",@nobits
	.type	n64_joy_data, @object
	.size	n64_joy_data, 4
n64_joy_data:
	.skip 4,0
	.section	.rodata._ZZ10GCPad_readbE4C.12,"a",@progbits
	.type	_ZZ10GCPad_readbE4C.12, @object
	.size	_ZZ10GCPad_readbE4C.12, 3
_ZZ10GCPad_readbE4C.12:
	.byte	64
	.byte	3
	.byte	0
