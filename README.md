# PixelBuds
This is a little program which spawns pure red, green, and blue pixels on a 32 x 64 RGB LED Matrix.

The pixels will share colors with neighbors causing a wider range of colors to be displayed.

The pixels themselves are wrapped in a class which handles their movement and interactions

#### Usage
I built this to run on a ESP32-WRoom dev board and I added a rotary encoder to change settings while its running, make sure you get one with a switch, or add a button and map it to a switch pin. When the button is pressed, you enter setting selection where active red LEDs on the matrix represent each setting, 1 led on = setting 1, 0 leds on = setting 0. Once you select a setting by pressing the switch, turn the encoder to change its value, the value of the setting can be seen on the serial monitor.

The code itself contains the pins I used for the LED Matrix and the encoder, but they can be changed to pretty much any other pin



##### Required Libraries
 - FastLED
 - ArxContainer
Once these are setup and your hardware is in place, just flash and enjoy