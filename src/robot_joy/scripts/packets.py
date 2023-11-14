# Servo On Packet
# import numpy as np


#### Protocol Constant ####
STX = 0x02
EXT = 0x03
ACM = 0x06
NAK = 0x15
RST = 0x12
DUMMY = 0xFF
# LGT is not an Constant. You should calculate it by XOR 
# from DUMMY to in front of EXT 



#### Packets ####

## Servo Motor On ##
SVON = ''.join([chr(STX),
                chr(DUMMY),
                chr(0x44),   # D
                chr(0x42),   # B
                chr(0x30),   # ch.1
                chr(0x31),   # on(1)
                chr(EXT),
                chr(0xF8),   # LGT
                chr(ACM)])


## Servo Motor Off ## 
SVOFF = ''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x44),   # D
                 chr(0x42),   # B
                 chr(0x30),   # ch.1
                 chr(0x30),   # off(0)
                 chr(EXT),
                 chr(0xF9),   # LGT
                 chr(ACM)])

