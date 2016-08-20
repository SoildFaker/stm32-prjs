# Working Around
Basically this project is developing on Archlinux(amd64) with STLINK V2 & OpenOCD supports.
##Materials needed
To move forward you will require the following materials:
- A computer running Linux of course, If you run Windows only, please don't be dissuaded. Manufacturer support is actually better for Windows since you can Google for it. And also there are lots of admiring IDEs on Windows to you.
- A STLinkV2 Clone. Looks link a USB flash disk.
- A development board with STM32F103C8 chip.

##Compiling & Software Debugging 
If you are using Archlinux just run the following command to install a GCC ARM Embedded utils.  
`sudo pacman -Syu arm-none-eabi-binutils arm-none-eabi-gcc arm-none-eabi-gdb`
##Flash & Hardware Debuging
OpenOCD:  
Install it from pacman.Click [here](https://github.com/ntfreak/openocd) for more guidance.  STLinkV2:  Clone [this](https://github.com/texane/stlink) repo and follow its installation guidance.  
