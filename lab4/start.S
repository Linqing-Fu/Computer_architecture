#include <asm.h>
#include <regdef.h>
#include <ucas_cde.h>

#define TEST_NUM 68

##s0, number
##s1, number adress 
##s2, exception use
##s3, score

	.set	noreorder
	.globl	_start
	.globl	start
	.globl	__main
_start:
start:
	j	locate
	nop

##avoid "j locate" not taken
    lui   t0, 0x8000
    addiu t1, t1, 1
    or    t2, t0, zero
    addu  t3, t5, t6
    lw    t4, 0(t0)
    nop

##avoid cpu run error
.org 0x0ec
    lui   t0, 0x8000
    addiu t1, t1, 1
    or    t2, t0, zero
    addu  t3, t5, t6
    lw    t4, 0(t0)
.org 0x100
test_finish:
    addiu t0, t0, 1
    b test_finish
    nop
##avoid cpu run error
    lui   t0, 0x8000
    addiu t1, t1, 1
    or    t2, t0, zero
    addu  t3, t5, t6
    lw    t4, 0(t0)
/*
 *  exception handle
 */
.org 0x380
	#LI (a0, 0xfffff008)
	#LI (t1, 0x0002)
	#sw t1, 0(a0)#led1

    mfc0  k0, $13 #cause
    LI(a0, 24999999)
    #addiu a0, zero, 24999999   #clean count
    mtc0  a0, $11  #compare
    mtc0  zero, $9 #count
    
    LI(t0, 0x00008000)
    and   k1, t0, k0 #ip7
    beq   k1, t0, clk_int
    nop

    LI(t0, 0x00000400)
    and   k1, t0, k0 #ip2
    beq   t0, k1, hard_int
    nop


int_end:
    .set mips32
    ERET 
    nop
    .set mips0

clk_int:
    #load every 4bit to reg
    LI(k0, 0xfffff010)
    lw    k1, (k0)

srl  t0, k1, 0
andi t0, t0, 0xf
srl  t1, k1, 4
andi t1, t1, 0xf
srl  t2, k1, 8
andi t2, t2, 0xf
srl  t3, k1, 12
andi t3, t3, 0xf
srl  t4, k1, 16
andi t4, t4, 0xf
srl  t5, k1, 20
andi t5, t5, 0xf


#t0
	addiu  a0, zero, 0x9
    beq   t0, a0, T0
    nop

    addiu t0, t0, 1
    j END
    nop

T0:
    LI(t0, 0)
    addiu  a0, zero, 0x5
    beq   t1, a0, T1
    nop

    addiu t1, t1, 1
    j END
    nop

T1:
    LI(t1, 0)
    addiu  a0, zero, 0x9
    beq   t2, a0, T2
    nop
    addiu t2, t2, 1
    j END
    nop

T2:
    LI(t2, 0)
    addiu  a0, zero, 0x5
    beq   t3, a0, T3
    nop
    addiu t3, t3, 1
    j END
    nop

T3:
    LI(t3, 0)
    addiu  a0, zero, 0x9
    bne   t4, a0, 3f          # !=9?
    nop
    add   t4, zero, zero   # ==9, 9->0
    addiu  t5, t5, 1    #t5+1
    j     END
    nop
 
 3:
    addiu  a0, zero, 0x3
    bne   t4, a0, 2f #!=3
    nop
    addiu  a1, zero, 0x2 
    bne   t5, a1, 2f # !=2
    nop
    add   t4, zero, zero
    add   t5, zero, zero  #23 -> 00
    j     END
    nop

 2:
    addiu  t4, t4, 1   #normal +1


END:
    sll t5, t5, 20
    sll t4, t4, 16
    sll t3, t3, 12
    sll t2, t2, 8
    sll t1, t1, 4
    or t5, t5, t4
    or t5, t5, t3
    or t5, t5, t2
    or t5, t5, t1
    or t5, t5, t0
    sw  t5, (k0)
    nop

	j     int_end
	nop
#####################################################################
hard_int:
	mtc0 zero, $11#clean compare clean TI
	LI (a0, 0xfffff004)
	LI (t1, 0x0001)
	sw t1, 0(a0)#led2
	nop
    LI(k0, 0xfffff024)  #key addr

wait_until_0:
    #wait until 1->0
    lw    k1, (k0)
    nop
    andi  k1, k1, 0x1
   	bne   zero, k1, wait_until_0
   	nop

   	


loop_num:
    LI(k0, 0xfffff024)
    lw    k1, (k0)   #get the key data
    addiu  a0, zero, 15  # 1111
    and   k1, k1, a0
    beq   zero, k1, loop_num  #  while [3] [2] [1]  == 0 loop
    nop
    addiu  a0, zero, 0x1  
    beq   k1, a0, wait_until_01   #exit set
    nop
    addiu  a0, zero, 0x2  
    beq   k1, a0, set_hour #set_hour
    nop
    addiu  a0, zero, 0x4
    beq   k1, a0, set_min #set min
    nop
    addiu  a0, zero, 0x8
    beq   k1, a0, set_sec #set second
    nop

################################################################
set_hour:

    LI(a0, 2000000)
hour_nop_loop:
	addiu a0, a0, -1
    bne zero, a0, hour_nop_loop
    nop

    LI(k0, 0xfffff010)
    lw    k1, (k0)
    srl   t0, k1, 16
    andi  t1, t0, 0xf   #t1 = 4 in 24
    srl   t0, t0, 4   #>>4
    andi  t2, t0, 0xf  #t2 = 2 in 24

    addiu  a0, zero, 0x9
    bne   t1, a0, 1f          # !=9?
    nop
    add   t1, zero, zero   # ==9, 9->0
    addiu  t2, t2, 1    #t2+1
    j     store_hour_num
    nop
 
 1:
    addiu  a0, zero, 0x3
    bne   t1, a0, 2f #!=3
    nop
    addiu  a1, zero, 0x2 
    bne   t2, a1, 2f # !=2
    nop
    add   t1, zero, zero
    add   t2, zero, zero  #23 -> 00
    j     store_hour_num
    nop

 2:
    addiu  t1, t1, 1   #normal +1


 store_hour_num:
	LI(t6, 0xff00ffff)
    #andi  k1, k1, 0xffff   #low 16bit
    and  k1, k1, t6
    sll   t1, t1, 0x10  #<<16
    sll   t2, t2, 0x14 #<<20
    or    k1, k1, t1
    or    k1, k1, t2  

    
    sw    k1, (k0)
    nop 

    j     loop_num
    nop
#################################################################
set_min:

    LI(a0, 2000000)#200_0000
 min_nop_loop:
    addiu a0, a0, -1
    bne zero, a0, min_nop_loop
    nop

    LI(k0, 0xfffff010)
    lw    k1, (k0)
    srl   t0, k1, 8
    andi  t1, t0, 0xf   #t1 = 9 in 59
    srl   t0, t0, 4   #>>4
    andi  t2, t0, 0xf  #t2 = 5 in 594

    addiu  a0, zero, 0x9
    bne   t1, a0, 2f          # !=9?
    nop
    add   t1, zero, zero   # == 9, t1:9->0
    addiu  a1, zero, 0x5
    bne   t2, a1, 1f     #!=59 j 1f
    nop
    add   t2, zero, zero    #t2<-0
    j     store_min_num
    nop

 1:
    addiu  t2, t2, 1
    j     store_min_num
    nop

 2:
    addiu  t1, t1, 1   #normal +1

 store_min_num:
    LI(t6, 0xffff00ff)
    and  k1, k1, t6   #32
    sll   t1, t1, 0x8  #<<8
    sll   t2, t2, 0xc #<<12
    or    k1, k1, t1
    or    k1, k1, t2  


    
    sw    k1, (k0)
    nop 
    j     loop_num
    nop

####################################################################
set_sec:

    LI(a0, 2000000)
    #200_0000
sec_nop_loop:
    addiu a0, a0, -1
    bne zero, a0, sec_nop_loop
    nop

    LI(k0, 0xfffff010)
    lw    k1, (k0)
    srl   t0, k1, 0
    andi  t1, t0, 0xf   #t1 = 9 in 59
    srl   t0, t0, 4   #>>4
    andi  t2, t0, 0xf  #t2 = 5 in 59

    addiu  a0, zero, 0x9
    bne   t1, a0, 2f          # !=9?
    nop
    add   t1, zero, zero   # == 9, t1:9->0
    addiu  a1, zero, 0x5
    bne   t2, a1, 1f     #!=59 j 1f
    nop
    add   t2, zero, zero    #t2<-0
    j     store_sec_num
    nop

 1:
    addiu  t2, t2, 1
    j     store_sec_num
    nop

 2:
    addiu  t1, t1, 1   #normal +1

 store_sec_num:
 	LI(t6, 0xffffff00)
    #andi  k1, k1, 0xffff   #low 16bit
    and  k1, k1, t6
    #andi  k1, k1, 0xff00   #low 16bit
    sll   t2, t2, 0x4 #<<12
    or    k1, k1, t1
    or    k1, k1, t2  



    sw    k1, (k0) 
    nop
    j     loop_num
    nop


wait_until_01:
    LI(k0, 0xfffff024)
	lw    k1, (k0)
	andi  k1, k1, 0x1
	bne   zero, k1, wait_until_01
	nop

	LI(a0, 24999999)
    #addiu a0, zero, 24999999   #clean count
    mtc0  a0, $11  #compare
    mtc0  zero, $9 #count
    
	j int_end
	nop



    

#1:  
#   addiu t0, t0, 1
#  b 1b
# nop


locate:
	.set noreorder

	mtc0  zero, $11
	LI(a0, 24999999)
    #addiu a0, zero, 24900000
    mtc0  a0, $11  #compare
    mtc0  zero, $9 #count

	mfc0  k0, $12 #status
	LI(k1, 0xffff7bf8)
	and   k0, k0, k1  #status.ERL EXL =0
	LI(k1, 0x00008401)
	or    k0, k0, k1  #open IM7 IM2
	mtc0  k0, $12 #status

    LI (a0, 0xfffff008)
    LI (a1, 0xfffff004)
    LI (a2, 0xfffff000)
    LI (s1, 0xfffff010)

    LI (t1, 0x0000)
    LI (t2, 0x0000)
    LI (t3, 0x0000ffff)
    lui s3, 0

    sw t1, 0(a0)
    sw t2, 0(a1)
    sw t3, 0(a2)
    sw s3, 0(s1)
    lui s0, 0
# LI (a0, 0xfffff008)
#LI (t1, 0x0002)
# sw t1, 0(a0)
    

 


wait_1s:
    addiu t0, zero, 1
    addiu t0, zero, 0xfff
    addiu t0, zero, 0x5
    addiu t0, zero, 1
    addiu t0, zero, 0xfff
    addiu t0, zero, 0x5
    addiu t0, zero, 0xfff
    addiu t0, zero, 0x5
    addiu t0, zero, 1
    addiu t0, zero, 0xfff
    addiu t0, zero, 0x5
    addiu t0, zero, 0xfff
    addiu t0, zero, 0x5
    addiu t0, zero, 1
    addiu t0, zero, 0xfff
    addiu t0, zero, 0x5
    addiu t0, zero, 0xfff
    addiu t0, zero, 0x5
    addiu t0, zero, 1
    addiu t0, zero, 0xfff
    addiu t0, zero, 0x5
    addiu t0, zero, 0xffff
    addiu t0, t0, 1
    j wait_1s
    nop

