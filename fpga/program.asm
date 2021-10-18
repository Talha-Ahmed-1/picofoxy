li x15, 0x40001000 #base address
li x7 , 0x3 #data regitser
li x8 , 1048575 #data

sw x8 , 0x3(x15)