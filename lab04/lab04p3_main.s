// file: main_asm.s
// desc: lab04 p3
// author: <Evan Huizinga & Jared Fuller>

.global main
main:

.section  .text

    // Save registers r4-r7 on the stack before using them
    push {r4,r5,r6,r7}
	
	// r4 = base address of DS (start of our data section)
    ldr r4, =DS
	
	 // Load A (1 byte) into r5
	ldrb r5, [r4, #(A-DS)]
	
	// Load B (2 bytes) into r6
	ldrh r6, [r4, #(B-DS)]
	
	 // Add A + B and store result in r7
	add r7, r5, r6
	
	// Store result into C (4 bytes)
	str r7, [r4, #(C-DS)]

    // Placeholder instruction, marks end of main computation
    all_done: nop

    // Restore registers r4-r7 from the stack
    pop {r4,r5,r6,r7}

    // Return from main function
    bx lr

.section  .data
	.org 234
	.align 2, 0xa5

DS: .word 0xbbbbbbbb

A: .byte 123

	.align 1, 0xa5
B: .short 47587

	.align 2, 0xa5
C: .word ~0

	.align 3, 0xa5
	.word 0xeeeeeeee

.end