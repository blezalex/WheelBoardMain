# WheelBoard Controller
Balancing controller for DIY mono wheeled skateboard.

[![](https://img.youtube.com/vi/-kyArozNmkw/0.jpg)](https://www.youtube.com/watch?v=-kyArozNmkw)

[![](https://img.youtube.com/vi/4-E7deaGyFs/0.jpg)](https://www.youtube.com/watch?v=4-E7deaGyFs)


Supported hardware:
CPU: STM32F103CBT6
[Flip32](http://www.readytoflyquads.com/the-flip32) or Naze32 control boards.

I run it with: [VESC ESC](http://vedder.se/2015/01/vesc-open-source-esc/).
My vesc config is below

## Wiring:

* PWM1 - main balancing signal. Wire it to VESC PPM input.
* RC_CH1, RC_CH2  - optional input for force sensors. These register pressure on foot pads and allow board to start only when both pads are pressed.
* BUZZER - optional buzzer, lets you know when you reach speed limit.

#### FootPad sensors
I use 4 (2 on each side) [force sensing resistors](https://www.pololu.com/product/1645).
Connect two on each side in parallel. Then one wire to 3.3v, the other to RC_CH1. Add a pull down 3k resistor from RC_CH1 to ground.  (Same for other side, RC_CH2).

### Optional:
* USART1 RX, TX - goes to a bluetooth module (NON-BLE, BLE wont work), baud 115200, no parity, 8bits, 1 stop
Phone app https://github.com/blezalex/WheelBoardAndroid
* RC_CH3, RC_CH4 - USART2 RX/TX - goes to VESC RX/TX (gets various stats from vesc)

## My Setup:
* Main board Flip32 Rev5
* ESC [VESC](http://diyelectricskateboard.com/diy-electric-skateboard-kits-parts/vesc-the-best-electric-skateboard-esc/)
* [Hub motor with tire 36v 500W 11in](https://www.aliexpress.com/item/11inch-350w-500w-wide-tire-hub-motor-phub-44/32536443206.html)
* 13s 5000mAh lipo

## VESC Settings

* FOC
* All current limits 60A
* APP: USART and PPM
* Control Type: Current
* Median Filter: False
* Safe Start: False
* Positive/Negative ramping time 0.02

#### Mapping
* Start: 1ms
* End: 2ms
* Center: 1.5010ms
* Input Deadband: 0%

#### Throttle curve
no expos


# Frame build log: http://imgur.com/a/ALTTR
![current frame](http://i.imgur.com/aFqdWp5.jpg)

# Warning
Use at your own risk. No guarantees. This software is not bug-free. Self balancing vehicles are inherently unstable.

