# Servo On Packet

#### Protocol Constant ####
STX = "0x02"
EXT = "0x03"
ACM = "0x06"
NAK = "0x15"
RST = "0x12"
DUMMY = "0xFF"
# LGT is not an Constant. You should calculate it by XOR 
# from DUMMY to in front of EXT 



#### Packets ####

## Servo Motor On ##
SVON = ''.join([STX,
                DUMMY,
                "0x44",   # D
                "0x42",   # B
                "0x30",   # ch.1
                "0x31",   # on(1)
                EXT,
                "0xF8",   # LGT
                ACM])


SVOFF = ''.join([STX,
                DUMMY,
                "0x44",   # D
                "0x42",   # B
                "0x30",   # ch.1
                "0x30",   # off(0)
                EXT,
                "0xF9",   # LGT
                ACM])

