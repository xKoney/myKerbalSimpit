# My Kerbal Controller

## Background
Here's a list of links that I referenced while creating. Some were purely inspirational, while others were copy-pasted code.

__Master Documents__

https://kerbalsimpitrevamped-arduino.readthedocs.io/en/latest/index.html

https://github.com/Simpit-team/KerbalSimpitRevamped

https://github.com/Simpit-team/KerbalSimpitRevamped-Arduino


__Build Guides and References__

https://www.instructables.com/Kerbal-Controller-the-Basics/

https://github.com/ErinaceusLinnaeus/Kontrol

https://github.com/jacobcargen/KSPController

https://github.com/c0untzer0/MyKerbalController

https://github.com/CodapopKSP/Coda-Kerbal-Controller

https://github.com/Richi0D/KerbalController-Simpit

## What is this?

This is a custom peripheral/controller that is intended for use with Kerbal Space Program (1). This utilizes a mod called KerbalSimpitRevamped, available in the Kerbal mod manager CKAN. This controller has a wide array of built-in functions like SAS mode selection, throttle controller, stage & abort arming and triggering, LED bar graph indicators for each of the fuel types, and more.

Due to the absurd amount of IOs (70+ LEDs and more than 40 buttons or joystick inputs), this project utilizes Shift Registers to minimize the amount of pins required on the Arduino. With the number of shift registers used, I decided to design a circuit board that will house all the shift registers and provide pads for soldering all the IO connections required. 

## Disclaimer

As of the writing of this READ ME, I have not completed ordering or building this controller. I believe the design is in a finalized state, but I don't have the time to pursue this project at this moment. I hope to be able to build this controller soon. That being said, it would be prudent to verify the easyEDA file before shipping to a PCB manufacturer and do some dry fit test prints of the holes and components.

Also, please forgive my terrible programming practices. This is my first programming project, and I'm a mechanical engineer, not a programmer. You'll notice a lot of commented out "test code", lack of comments, and probably some inefficient methods.
