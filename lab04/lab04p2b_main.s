// file: Lab4_p2b.s
// desc: lab04 p2b
// author: <Jared Fuller & Evan Huizinga>


.global main
main:


.section  .text
  


    push {r4,r5,r6,r7} // Pushes r4-r7 registers


	ldr r4, =DS //Loads DS word into r4
	ldrb r5,[r4,#(B-DS)] //Loads the byte of B from the address assigned in r4
	ldr r6,[r4,#(W-DS)] //Loads the word of W from the address assigned in r4


	
    all_done: nop


    pop {r4,r5,r6,r7} // Pops r4-r7 registers
    
      bx lr


      
  .section .data


  .org 234
  .align 2,0xa5
  
DSECT:
    .word 0xbbbbbbbb //Provides a bound for the beginning of our memory address
	
B:	.byte 85 // Single byte data
	
	.align 2, 0xaa // Aligns next data to a 4-byte boundary (2^2 = 4)
				   // also pads with 0xAA bytes
DS: .word ~0	// 32-bit value, placed at the aligned address


	.align 2, 0xaa 	// Aligns next data to a 4-byte boundary (2^2 = 4)
				   	// also pads with 0xAA bytes
W:  .word 287454020	 // 32-bit integer at aligned address

	
	.word 0xeeeeeeee //Provides a bound for the end of our memory address


  .end
