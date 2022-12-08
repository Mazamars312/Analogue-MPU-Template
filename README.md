# Analogue Pocket MPU Template

## The current Specs of the MPU

Specs:
* Running at 74.2mhz (Same as the APF bus)
* Connected to the core via a 16bit HPS Bus 
* Two timers - one for interrupts and the other for user timing.
* Currently, 32K of memory/ram for the MPU's program (Will be set to 16K for a smaller footprint)
* A 8Kbyte swap buffer for both the APF and MPU to do transfers - can do up to 32K with changing the bram size. Anything larger would require some 
* The address of the swap memory starts at 0x0000_8000 on the MPU side and on the APF address it is at 0x8000_0000 (This can be changed on the .top_address parameter in the verilog code to any address you want on the APF bus)
* Boot Vector is at 0x0000_0000 and the interrupt is at 0x0000_0020 - the startup.S already has the code there for initialation of the MPU with the interrupts turned on.

## How to setup

### C Compiler for RISCV
* For this you will need the GCC compiler on your computer
* The best resource for setting this up is documented in there under "Build the RISC-V GCC" - https://github.com/SpinalHDL/VexRiscv
* Once there you can look under the \src\VexRiscv location for the main files that are used.
* Main.cpp contains the main loop and initialation of the MPU once the reset line from the APF has assurted the reset_l signal
* apf.cpp/h contains some APF fuctions to request or send data to the buffer area
* timer.cpp/h contains both the interrupt timer and a user timer function
* spi.cpp/h contains the HPS bus interface fuctions that where used to interface the mister framework
* printf.cpp/h the printf function is used with the Analogue debug cart's USB uart port to display debugging commands. The core has a 1024 sized fifo to make sure the CPU is not waiting for sending requests.
* interrupts.cpp/h this will currently turn on and off the timing interrupt mask in the core.
* core.cpp/h this is where the main core could should be for polling data and reseting the core. 

### The core itself
* The core will automativly start once and run the mpu.bin file when uploaded

Addresses: 
* MPU Address 0x00000000 - 0x00008000 / APF Address 0x80000000 - 0x80008000 - Program Ram - locked from the APF bus when running
* MPU Address 0x00008000 - 0x80010000 / APF Address 0x80008000 - 0x80010000 - 8K Buffer mirrored
* MPU Address 0xFFFFFF20 - 0xFFFFFF4c - These are for the reading the joystick inputs from the APF Framework
APF Target Dataslot commands
* MPU Address 0xFFFFFF80 - target_dataslot_id 
* MPU Address 0xFFFFFF84 - target_dataslot_bridgeaddr
* MPU Address 0xFFFFFF88 - target_dataslot_length
* MPU Address 0xFFFFFF8C - target_dataslot_slotoffset
* MPU Address 0xFFFFFF90 - {target_dataslot_ack, target_dataslot_done, target_dataslot_err[2:0]}

UART Timing
* MPU Address 0xFFFFFF94 - uart_divisor = This is calculated in the UART.c program with the formula ((sys_clock * 1000)/uart_rate)
* MPU Address 0xFFFFFF94 - sysclk_frequency = This is to advise the UART and timers what the CPU clock rate is in MHZ

Timers
* MPU Address 0xFFFFFFC4 - millisecond_counter_1 - This will show the interupt counter in Milliseconds
* MPU Address 0xFFFFFFC8 - interupt_counter_1 - This will set the interrupt timer to interrupt the CPU when both the interrupt mask is on and the counter is greater or equal to this number
* MPU Address 0xFFFFFFCC - millisecond_counter_2 - This will show the interupt counter in Milliseconds

HPS Bus control
* MPU Address 0xFFFFFFD0 - {io_ss2, io_ss1,io_ss0,io_clk,1'b0,IO_DOUT[15:0]}
* MPU Address 0xFFFFFFD0 - {io_ack, IO_WIDE, IO_DIN}

Interrupt Mask
* MPU Address 0xFFFFFFF4 - {interupt_output_1, interupt_output_1_reg, timerenabled}

## Why do things take some time to load or save?

* I have recently learnt that the APF does time based checks on requests from the MPU/Target dataslots. So make sure you can do some caching or make the buffer size larger to lower timing for the transfers.
* Try and keep the transfer sizes below 32Kbytes as the APF can change how much it sends if larger then this size

## When will writes from the core to the APF bus 

Right now the APF file system will re-write data to the file but will resize it to that last data packet you have sent. A bug fix will be coming soon

## To Be Done:
* Right now getting malloc and calls working so the swap memory can be cleared when needed
* Write access when the next release canadate is out

## Credits

* Analogue for the Pocket and giving me a lot of support on this.
* MiSTer, Mist, DeMiSTify projects and the Communities :-) Love you all
* Mist and their Community.
* VexRISCV Team - https://github.com/SpinalHDL/VexRiscv
* Robinsonb5 for his time answering my questions on his 832 CPU - Tho I did not use this excellent-looking CPU, I was not able to get the C Compiler working 100% on my builds
* Boogerman for some coding questions - You still owe me on the use of my JTAG tho LOL
* Electron Ash, AGG23 as a beta testers.
* And many more, so please message me and I would happily add them to this!!
