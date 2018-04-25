# Introduction
This project is no longer maintained, it is only a semi-finished products. We use it to participate in the competition, and for better performance we are going to start a new project. That project will based on highly self-made materials include PCB and RTOS.
For this project it's basically developing on Archlinux(amd64) with STLINK V2 & OpenOCD supports.
To move forward you will require the following materials:
- A computer running Linux of course, If you run Windows only, please don't be dissuaded. Manufacturer support is actually better for Windows, and also you can find lots of admiring IDEs on Windows. Google will help you.
- A STLinkV2 Clone. Looks like a USB flash disk.
- A development board with STM32F103C8 chip.

## Compiling & Software Debugging 
If you are using Archlinux just run the following command to install a GCC ARM Embedded utils.  
`sudo pacman -Syu arm-none-eabi-binutils arm-none-eabi-gcc arm-none-eabi-gdb`
startup script and link file were edited to solve the printf float number issue.
## Flash & Hardware Debuging
OpenOCD:  
Install it from pacman.Click [here](https://github.com/ntfreak/openocd) for more guidance.  STLinkV2:  Clone [this](https://github.com/texane/stlink) repo and follow its installation guidance.  

