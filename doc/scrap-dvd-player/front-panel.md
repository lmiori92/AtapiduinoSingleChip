# Front Panel
The front panel includes a VFD display, 6 buttons and a red-green power LED (frame for the power button).

# Pinout - Data signals

1. (Red) IR receiver signal
2. GND
3. DIN
4. DOUT
5. CLK
6. STB

# Pinout - Power supply

1. (Blue) Filament
2. Filament
3. GND
4. Vee (negative for VFD)
5. (Red) +5V

IMPORTANT NOTE: the power supply stays in a (standby?) state if no load is detected on the +5V rail. Hence, even for initial testing it is important to attach e.g. a 100ohm load.
