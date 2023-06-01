# ADC Converter

STM32F7 ADC Converter using DMA (320 kHz), measured samples tranfered over Ethernet

## ADC

- ADC1 in3 used
- runs in background using DMA (circular mode)
- sampling frequency: 320 kHz
- resolution: 12 bit
- sampling time: 15 cycles

- ADC fills half of buffer in 1600 us (possible up to 10 UDP messages can be sent in between ADC half buffer ready events)

## Ethernet interface

- ip: static 192.168.100.173
- client ip: static 192.168.100.172
- UDP message:

| u8  | u8   | u8   | u8[1024]  |
|-----|------|------|-----------|
| SYN | ADDR | TYPE | DATA      |
    SYN = 22 - message starts with
    ADDR = 0...255 - an address of the signal
    TYPE
        8 - 1 byte integer value
        16 - 2 byte float value
        32 - u16[512] an array of 2 byte values of length 512

- UDP message sent in 140 us

## Compile & run

```bash
make && probe-run --chip STM32F767ZITx ./build/STM32F767_ADC_DMA.elf
```
