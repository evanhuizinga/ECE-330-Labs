// file: main_asm.s
// desc: lab04 p2a
// author: <Evan Huizinga & Jared Fuller>

.global main
main:

.section  .text

    // Save registers r4 and r5 on the stack before using them
    push {r4,r5}

    // Load the address of the label W into r4
    ldr r4, =W

    // Load the 32-bit value stored at address W into r5
    ldr r5, [r4]

    // Placeholder instruction, marks end of main computation
    all_done: nop

    // Restore registers r4 and r5 from the stack
    pop {r4,r5}

    // Return from main function
    bx lr

.section  .data

    // Arbitrary 32-bit value in the data section
    .word 0xbbbbbbbb

    // Label W holds a 32-bit integer
W:  .word 4277009102

    // Another 32-bit value immediately after W
    .word 0xeeeeeeee

.end