# STM32 Panel Driver
The STM32 panel driver uses DMA to shift the data to the panels.

The board used is a BluePill with a STM32F103C8T6.

## Connections
| Panel Pin | STM32 GPIO |
| --------- | ---------- |
| GND       | GND        |
| OE        | PB4        |
| LAT       | PB5        |
| CLK       | PA8        |
| 1RD1      | PA0        |
| 2RD1      | PA1        |
| GD1       | PA2        |
| BD1       | PA3        |
| 1RD2      | PA4        |
| 2RD2      | PA5        |
| GD2       | PA6        |
| BD2       | PA7        |