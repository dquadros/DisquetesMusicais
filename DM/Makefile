PROGNAME=DM
GCCFLAGS=-g -Os -Wall -mcall-prologues -mmcu=attiny2313 -DF_CPU=16000000
LINKFLAGS=-Wl,-Map,${PROGNAME}.map
AVRDUDEFLAGS=-c usbtiny -p attiny2313
LFUSE = 0xcf
HFUSE = 0x9f
EFUSE = 0xff
LOCK  = 0x3f

all:	${PROGNAME}.hex

program: ${PROGNAME}-upload

fuse:
	avrdude ${AVRDUDEFLAGS} -u -U hfuse:w:$(HFUSE):m -U lfuse:w:$(LFUSE):m \
    -U efuse:w:$(EFUSE):m -U lock:w:$(LOCK):m

${PROGNAME}.hex: ${PROGNAME}.obj
	avr-objcopy -R .eeprom -O ihex ${PROGNAME}.obj ${PROGNAME}.hex

${PROGNAME}.obj: ${PROGNAME}.c
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o ${PROGNAME}.obj ${PROGNAME}.c
   
${PROGNAME}-upload:	${PROGNAME}.hex
	avrdude ${AVRDUDEFLAGS} -U flash:w:${PROGNAME}.hex:i
