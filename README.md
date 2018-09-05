# modLED
Modular LED panel

**Work in progress**

We picked up 60 used DI-P20F04D RGB LED panels and want to build something with them.

On 30W @ 5V, all 60 panels would draw a combined 1.8kW / 360A on full brightness. So a really beefy power supply, or many small ones are needed.  


## LED Panels
- DI-P20F04D
- Operating voltage is 5V, the VDD for the LEDs and VCC for the ICs are seperated
- The panels are 16 x 8 pixel big and have RRGB subpixels.
- Near the input connector is a 74HC245D bus transceiver
- For every 16 LEDs of one color there is a TB62726 constant current LED driver
- More information can be found in its [dataheet](doc/DI-P20F04D_datasheet.pdf)

### Control
The LED drivers work like a shift register. They get data shifted in and upon a latch edge display the data on the LEDs.

This makes driving them quite easy, but requires implementing individual LED brightness control in the controller.

Global brightness control should be feasible by PWM-ing the `output enable` pin on the panel