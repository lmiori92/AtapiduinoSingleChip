# AtapiduinoSingleChip
Atapiduino running and interfacing with display and CD-ROM drive all in a single Atmega328 chip - http://singlevalve.web.fc2.com/Atapiduino/atapiduino.htm

# Introduction

Atapiduino (http://singlevalve.web.fc2.com/Atapiduino/atapiduino.htm) is a very nifty project that allows you to interface with a CDROM unit and command it for playing audio CDs.

Even though I very much like modular components like he did I have figured out that it uses too many (and pricey) components and the same functionality can be achieded by a single atmega328 chip, including display driving (PT6311).

# Context

I want to repurpose an old DVD/Multimediaish player with all chinese components and super complicated output stages.
The DVD drive is faulty and when/if it works it doesn't sound superb either...
So, I took the opportunity to invent something myself to "glue" atapiduino's functionality essentially lowering project's cost.

# Overall solution

Basically, we make use of *every* single available I/O that is usable as I/O :-)
The CDROM functionality only can be achieved without using the RESET pin as output and using AREF pin (more details later on) as nDIOW output.
Now, to support the display some sort of chip selection must be performed, otherwise bus data for the IDE goes to the display making it work unpredictably. Therefore:
- if no usage of the RESET pin is possible or is unwanted (I have a parallel programmer, so I can always restore a chip but someone might not and one mistake causes the chip to be rendered useless) one component is necessary: a NAND port that connects to the CS0 and CS1 outputs so that the output of the NAND can be fed to a active-low ChipSelection on the target display. So, this solution uses an extra chip (which is something I want to avoid as a challenge!)
- if usage of the RESET pin is desirable and possible (attention: makes ISP programming impossible, only parallel programming possible). 

# First solution

The trick is to use AREF as a digital output (!). Yes, it is definitely possible. Output current is negligible and needs external amplification with a BC548 configured open collector i.e. switching to ground. IDE interface makes use of +5V active pull-ups, hence by driving the transistor we set the output as "low" and when we don't drive the transistor pull-up voltages sets the output to "high".

# License

The original atapiduino is distributed with the GPLv3 license, hence this project is also licensed under the terms of the GPLv3 license.
As per license text I cannot be liable of any damage that the software nor by any documented instruction can cause.
