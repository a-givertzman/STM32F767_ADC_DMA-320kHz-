# ADC Converter

- sampling frequency: 320 kHz
- interface: Ethernet UDP

## ADC

## Ethernet interface

- ip: static 192.168.120.173
- client ip: static 192.168.120.173
- UDP message:
| u8  | u8   | u8   | u8[1024]  | 
| SYN | ADDR | TYPE | DATA      |
    SYN = 22 - message starts with
    ADDR = 0...255 - an address of the signal
    TYPE
        8 - 1 byte integer value
        16 - 2 byte float value
        32 - u16[1024] an array of 2 byte values of length 512

## Compile & run

```bash
make && probe-run --chip STM32F767ZITx ./build/STM32F767_ADC_DMA.elf
```
