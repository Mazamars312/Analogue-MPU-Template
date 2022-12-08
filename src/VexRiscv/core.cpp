/*
 * Copyright 2022 Murray Aickin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>


#include "core.h"
#include "printf.h"
#include "hardware.h"
#include "apf.h"
#include "timer.h"
#include "spi.h"
#define DATASLOT_FDD_BASE 320 // From 320-323 data slots I have advised that are used for harddrive access
#define DATASLOT_HDD_BASE 310 // From 310-313 data slots I have advised that are used for harddrive access
#define DATASLOT_BIOS_BASE 210 // BIOS Update data slot if you want the MPU to update the bios.


uint8_t sector_buffer0[512] __attribute__((section("SRAM"))) ; // We have to set an attribute to help point the program in to the 2K buffer that the APF bus has access to after the MPU has been restarted
uint8_t sector_buffer1[512] __attribute__((section("SRAM"))) ; // This is because the main ram is locked to only being used by the MPU for the instruction and data access.

// Sends a reset via the HPS Bus
void core_reset(int reset){
		HPS_spi_uio_cmd8(UIO_MM2_RST, reset);
		printf("reset %d\r\n",reset);
};
// Update dataslots

void core_update_dataslots(){
  int tmp = DATASLOT_UPDATE_REG(1);
	printf("dataslot Update %d\r\n", tmp);
	if (tmp == DATASLOT_BIOS_BASE){
		core_reset(7); // reset the core via the HPS bus
		usleep(200);
		core_reset(0); // Turn off the reset for the core 
	} else if (tmp >= DATASLOT_FDD_BASE && tmp <= DATASLOT_FDD_BASE + 3 ) {
		// This will update the internal 
	}
};

void core_poll_io(){

      // Here is where you do your polling on the core - eg Floppy and CDROM data

};


void core_reg_update(){
	// This can be used for polling the APF regs from the interaction menu to change core settings
};

void core_restart_first(){
	// what to do to start up the core if required
};

void core_restart_running_core() {
	// this can be used for restarting the core 
	
};

void core_input_setup() {
	// this is used for controller setup 
	
};

void core_input_update() {
	// this is called via the interrupts.c for controller updates to the core 
	
};


