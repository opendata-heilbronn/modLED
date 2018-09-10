# modLED
Modular LED panel

**Work in progress**

We picked up 60 used DI-P20F04D RGB LED panels and want to build something with them.

At 30W @ 5V, all 60 panels would draw a combined 1.8kW / 360A on full brightness. So a really beefy power supply, or many small ones are needed.

## LED Panels
- DI-P20F04D
- Operating voltage is 5V-ish, the VDD for the LEDs and VCC for the ICs are separated
- The panels are 16 x 8 pixel big and have RRGB subpixels.
- Near the input connector is a 74HC245D bus transceiver
- For every 16 LEDs of one color there is a TB62726 constant current LED driver (so you could increase the efficiency a lot if the led voltage is reduced to, say, 4V)
- More information can be found in its [datasheet](doc/DI-P20F04D_datasheet.pdf)

### Control
The LED drivers work like a shift register. They get data shifted in while the latch pin is low, and when the latch pin is high the data is forwarded to the led drivers (latch is not edge-triggered).

This makes driving them quite easy, but requires implementing individual LED brightness control in the controller.

Global brightness control should be feasible by PWM-ing the `output enable` pin on the panel (fast / slow enough so there is no visible interference pattern with the pixel pwm).

#### LED Order
The pixels are controlled in the following order (0 = first shifted in).
The top and bottom half have their own data pins for each color so both halves can be filled with data at the same time.
```
00 01 02 03  16 ...
04 05 06 07  ...
08 09 10 11  ...
12 13 14 15  ...

00 01 02 03  16 ...
04 05 06 07  ...
08 09 10 11  ...
12 13 14 15  ...
```