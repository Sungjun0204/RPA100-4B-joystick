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


############################# Packets #########################




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






###########################
## XYZ Coordinate Moving ##
########################### 

## X Coordi. Moving ##
MUVX = [''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x30),   # Axis(x)
                 chr(0x31),   # Direction (+)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xF8),   # LGT
                 chr(0xCF),
                 chr(ACM)]),

        ''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x30),   # Axis(X)
                 chr(0x30),   # Direction (-)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xF9),   # LGT
                 chr(0xCF),
                 chr(ACM)])
            ]


## Y Coordi. Moving ##
MUVY = [''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x31),   # Axis(Y)
                 chr(0x31),   # Direction (+)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xF9),   # LGT
                 chr(0xCF),
                 chr(ACM)]),

        ''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x31),   # Axis(Y)
                 chr(0x30),   # Direction (-)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xF8),   # LGT
                 chr(0xCF),
                 chr(ACM)])
            ]


## Z Coordi. Moving ##
MUVZ = [''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x32),   # Axis(Z)
                 chr(0x31),   # Direction (+)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xFA),   # LGT
                 chr(0xCF),
                 chr(ACM)]),

        ''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x32),   # Axis(Z)
                 chr(0x30),   # Direction (-)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xFB),   # LGT
                 chr(0xCF),
                 chr(ACM)])
            ]


## W Coordi. Moving ##
MUVW = [''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x33),   # Axis(W)
                 chr(0x31),   # Direction (+)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xFB),   # LGT
                 chr(0xCF),
                 chr(ACM)]),

        ''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x33),   # Axis(W)
                 chr(0x30),   # Direction (-)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xFA),   # LGT
                 chr(0xCF),
                 chr(ACM)])
            ]




## Servo Motor Speed
SPEED = [chr(STX),
         chr(DUMMY),
         chr(0x43),   # C
         chr(0x42),   # B
         chr(0x30),   # 10000
         chr(0x30),   # 1000
         chr(0x31),   # 100
         chr(0x30),   # 10
         chr(0x30),   # 1
         chr(EXT),
         chr(0xCF),
         chr(ACM)]