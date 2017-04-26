# WheelBoard Controller
Balancing controller for DIY mono wheeled skateboard.

[![](https://img.youtube.com/vi/4-E7deaGyFs/0.jpg)](https://www.youtube.com/watch?v=4-E7deaGyFs)


Supported hardware:
CPU: STM32F103CBT6
[Flip32](http://www.readytoflyquads.com/the-flip32) or Naze32 control boards.

I run it with: [VESC ESC](http://vedder.se/2015/01/vesc-open-source-esc/) and my custom firmware. You can run with vanila VESC in FOC mode.

## Wiring:

PWM1 - main balancing signal. Wire it to VESC PPM input.
RC_CH1, RC_CH2  - optional input for force sensors. These register pressure on foot pads and allow board to start only when both pads are pressed.
BUZZER - optional buzzer, lets you know when you reach speed limit.

Wiring foot pad sensors:
I use 4 (2 on each side) [force sensing resistors](https://www.pololu.com/product/1645).
Connect two on each side in parallel. Then one wire to 3.3v, the other to RC_CH1. Add a pull down 10k resistor from RC_CH1 to ground.  (Same for other side, RC_CH2).


## My Setup:
* Main board Flip32 Rev5
* ESC [VESC](http://diyelectricskateboard.com/diy-electric-skateboard-kits-parts/vesc-the-best-electric-skateboard-esc/)
* [Hub motor with tire 36v 500W 11in](https://www.aliexpress.com/item/11inch-350w-500w-wide-tire-hub-motor-phub-44/32536443206.html)
* 2x5s 4000mAh lipo


# Warning
Use at your own risk. No guarantees. This software is not bug-free. Self balancing vehicles are inherently unstable.

