#mask for interrupt:
.eqv EXT_INTTIME 0x0400 #mask for timer interrupt (external), bit
.eqv EXT_INTBUTTON 0x0800 #mask for button interrupt (external), bit 11
.eqv EXCHMASK 0x007C #mask for exceptions (internal), bits 2-6

#Timer:
.eqv ENABLE_TIMER_ADR 0xFFFF0012 #I/O enabling timer
.eqv ENABLE_TIMER 0x01 #mask for enabling timer

#Button:
.eqv BUTTONADDR 0xFFFF0013 #I/O button address
.eqv WALK_BUTTON 0x01 #mask for pedestrian button
.eqv DRIV_BUTTON 0x02 #mask for car button

#Clear:
.eqv CLEAR_TIME 0xFFFFFBFF #mask for clearing timer interrupt, bit 10
.eqv CLEAR_BUTTON 0xFFFFF7FF #mask for clearing button interrupt, bit 11
.eqv CLEAR 0xFFFFF3FF #mask for clearing time and button

#Lights:
.eqv WALKLIGHT 0xFFFF0010 #I/O walk light address
.eqv DRIVLIGHT 0xFFFF0011 #I/O driver light address
.eqv PRED_CGREEN 0x01 #state value
.eqv PGREEN_CRED 0x02 #state value
#Ped:
.eqv DARK 0x00 #traffic lights
.eqv STOP 0x01 #traffic lights (ped), bit 0
.eqv WALK 0x02 #traffic lights (ped), bit 1
#Cars:
.eqv RED 0x01 #traffic lights (car), bit 0
.eqv ORANGE 0x02 #traffic lights (car), bit 1
.eqv GREEN 0x04 #traffic lights (car), bit 2

.data
   space: .asciiz " "
   red:	.asciiz "\nRed\n"
   yel:	.asciiz "\nYellow\n"
   gre:	.asciiz "\nGreen\n"
   winkstr: .asciiz "\nWink\n"
   timer: 	.word 0
   state: .space 1
   buttonstr:.asciiz "Knapptryck!\n"

.ktext 0x80000180
   la $k0, int_routine
   jr $k0
   nop

.text
.globl main
main:
   mfc0 $t0, $12 #prepare status register for button interrupt
   ori $t0, $t0, EXT_INTBUTTON
   ori $t0, $t0, EXT_INTTIME
   ori $t0, $t0, 1
   mtc0 $t0, $12
   li $t0, ENABLE_TIMER #enable the timer (t0 > 0)
   sb $t0, ENABLE_TIMER_ADR
   li $t0, PRED_CGREEN #initialize state (red for ped and green for car)
   sb $t0, state
   li $t0, STOP #put traffic lights on (red for ped)
   sb $t0, WALKLIGHT
   li $t0, GREEN #put traffic lights on (green for car)
   sb $t0, DRIVLIGHT

loop: #infinite loop, waiting for interrupts
   nop
   b loop
   li $v0, 10 #exit
   syscall

.globl int_routine
int_routine:
   subu $sp, $sp, 16
   sw $at, 8($sp) #save registers used (not k0, k1)
   sw $t0, 4($sp)
   sw $t1, 0($sp)
   mfc0 $k1, $13 #extract EXCCODE field from Cause register
   andi $k0, $k1, EXCHMASK #extract EXCCODE (bits 2-6)
   bne $k0, $zero, restore #check EXCCODE (if nonzero leave)
   andi $k0, $k1, EXT_INTTIME #extract bit 10 (timer) from Cause register
   beq $k0, $zero, button #if no timer interrupt leave
   
   lw $a0, timer #if timer interrupt update and print timer
   addi $a0, $a0, 1
   sw $a0, timer
   
   li $v0, 1
   syscall
   
   la $a0, space
   li $v0, 4
   syscall
   
button:
   andi $k0, $k1, EXT_INTBUTTON #extract bit 11 (button) from Cause register
   beq $k0, $zero, restore #if no button interrupt leave
   
   lb $t0, BUTTONADDR #check which button
   andi $t1, $t0, WALK_BUTTON
   bne $t1, $zero, clean
   
   b restore
clean: 
   lw $a0, timer
   move $a0, $zero
   sw $a0, timer
   li	$v0, 4 			# print a space
   la 	$a0, yel
   syscall
  
orange:#Yellow for cars
   lb $t0, state #handling pedestrian button
   beq $t0, PGREEN_CRED, restore
   
   li $t0, ORANGE #switch traffic lights and state
   sb $t0, DRIVLIGHT
   
   lw $a0, timer #if timer interrupt update and print timer
   addi $a0, $a0, 1
   sw $a0, timer
   
   li $v0, 1
   syscall
      
   la $a0, space
   li $v0, 4
   syscall
   
   lw $t9, timer
   
   beq $t9, 3, clean_1
   
   j orange

clean_1:
   lw $a0, timer
   move $a0, $zero
   sw $a0, timer
   li	$v0, 4 			# print a space
   la 	$a0, gre
   syscall
      
walk_green:#Red for cars	
   li $t0, RED #switch traffic lights and state
   sb $t0, DRIVLIGHT

   li $t0, WALK
   sb $t0, WALKLIGHT
   
   li $t0, PGREEN_CRED
   sb $t0, state
   
   lw $a0, timer #if timer interrupt update and print timer
   addi $a0, $a0, 1
   sw $a0, timer
   
   li $v0, 1
   syscall
      
   la $a0, space
   li $v0, 4
   syscall
   
   lw $t9, timer
   beq $t9, 7, clean_2
   
   j walk_green

clean_2:
   lw $a0, timer
   move $a0, $zero
   sw $a0, timer 
   li $v0, 4 			# print a space
   la $a0, winkstr
   syscall
	      
wink: # PRED_CGREEN + wink
   lb $t0, state #handling car button
   beq $t0, PRED_CGREEN, restore

   li $t0, STOP #switch traffic lights and state
   sb $t0, WALKLIGHT
   	
   li $t0, DARK #switch traffic lights and state
   sb $t0, WALKLIGHT
    
   li $t0, STOP #switch traffic lights and state
   sb $t0, WALKLIGHT
 
   lw $a0, timer #if timer interrupt update and print timer
   addi $a0, $a0, 1
   sw $a0, timer
   
   li $v0, 1
   syscall
      
   la $a0, space
   li $v0, 4
   syscall
   
   lw $t9, timer
   
   beq $t9, 3, clean_3
   
   j wink

clean_3: 
   lw $a0, timer
   move $a0, $zero
   sw $a0, timer
   li	$v0, 4 			# print a space
   la 	$a0, yel
   syscall
     
orange_1:#orange for cars PRED_CGREEN
   li $t0, ORANGE
   sb $t0, DRIVLIGHT
   
   li $t0, PRED_CGREEN
   sb $t0, state
   
   lw $a0, timer #if timer interrupt update and print timer
   addi $a0, $a0, 1
   sw $a0, timer
   
   li $v0, 1
   syscall
   
   la $a0, space
   li $v0, 4
   syscall
   
   lw $t9, timer
   
   beq $t9, 3, clean_4
   
   j orange_1
   
clean_4:
   lw $a0, timer
   move $a0, $zero
   sw $a0, timer

car_green: #GREEN LIGHT for cars  PRED_CGREEN
   lb $t0, state #handling car button
   li $t0, STOP #switch traffic lights and state
   sb $t0, WALKLIGHT
   
   li $t0, GREEN
   sb $t0, DRIVLIGHT
   
   li $t0, PRED_CGREEN
   sb $t0, state

restore: #restore registers before leaving
   lw $at, 8($sp)
   lw $t0, 4($sp)
   lw $t1, 0($sp)
   addiu $sp, $sp, 16
   andi $k1, $k1, CLEAR_BUTTON #clear bit 11 (button) in Cause reg., set to 0
   andi $k1, $k1, CLEAR_TIME #clear bit 10 (timer) in Cause register, set to 0
   mtc0 $k1, $13 #clear Cause
   eret #return using EPC
