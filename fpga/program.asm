li x15, 0x40001000
li x14, 200000
LOOP:
 addi x3,x3,0
 sw x3, 0x0(x15)
 li x5,0x1
 li x6,300000
 li x7,0
LOOP1:
sw x5, 0x3(x15)
 addi x7,x7,1
 bne x7,x6,LOOP1
 li x6,300000
 li x7,0
 li x5,0x0
LOOP2:
 sw x5, 0x3(x15)
 addi x7,x7,1
 bne x7,x6,LOOP2


 j LOOP