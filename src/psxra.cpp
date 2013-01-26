/*
 * PSX RetroPad Adapter - PSX/PS2 adapter for retro-controllers!
 * Copyright (c) 2012 Bruno Freitas - bootsector@ig.com.br
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <WProgram.h>

#include "PSPadEmu.h"
#include "PS2Pad.h"
#include "genesis.h"
#include "saturn.h"
#include "NESPad.h"
#include "WiiCC.h"
#include "GCPad.h"

// PSX Buttons
int up = 0;
int down = 0;
int left = 0;
int right = 0;
int sqre = 0;
int cross = 0;
int circle = 0;
int triangle = 0;
int select = 0;
int start = 0;
int l1 = 0;
int r1 = 0;
int l2 = 0;
int r2 = 0;
int l3 = 0;
int r3 = 0;
int lx = 0x7F;
int ly = 0x7F;
int rx = 0x7F;
int ry = 0x7F;

// Extension cable detection pins
#define DETPIN0 1 // DB9 pin 4
#define DETPIN1 4
#define DETPIN2 5
#define DETPIN3 6

// Possible values (as of today) returned by the detectPad() routine
// Normal pads
#define PAD_GENESIS		0b0111
#define PAD_NES 		0b0110
#define PAD_SNES 		0b0101
#define PAD_PS2 		0b0100
#define PAD_GC	 		0b0011
#define PAD_N64			0b0010
#define PAD_NEOGEO		0b0001
#define PAD_WIICC		0b0000 // Wii Classic Controller
// Extended pads (uses DB9 pin 4 for identification)
#define PAD_SATURN		0b1111
#define PAD_DFU_DONGLE	0b1110 // Reserved for USBRA DFU dongle
#define PAD_DO_NOT_USE	0b1100 // 3 LSB overlaps with PS2 pad, which uses DB9 pin 4 for CLK.

/*
 * This is the new auto-detect function (non jumper based) which detects the extension
 * cable plugged in the DB9 port. It uses grounded pins from DB9 (4, 6, 7 and 9) for
 * the detection.
 *
 *  -1 - Arcade
 * 0111 - Sega Genesis (Default)
 * 0110 - NES
 * 0101 - SNES
 * 0100 - PS2
 * 0011 - Game Cube
 * 0010 - Nintendo 64
 * 0001 - Neo Geo
 * 0000 - Reserved 1
 * 1111 - Sega Saturn
 */
int detectPad() {
	int pad;

	// Set pad detection pins as input, turning pull-ups on
	pinMode(DETPIN1, INPUT);
	digitalWrite(DETPIN1, HIGH);

	pinMode(DETPIN2, INPUT);
	digitalWrite(DETPIN2, HIGH);

	pinMode(DETPIN3, INPUT);
	digitalWrite(DETPIN3, HIGH);

	// Read extension detection pins statuses
	pad = (digitalRead(DETPIN1) << 2) | (digitalRead(DETPIN2) << 1) | (digitalRead(DETPIN3));

	// Check if pad is not PS2 pad, that uses DB9 pin 4.
	// If not, then use pin 4 for additional pads
	if(pad != PAD_PS2) {
		pinMode(DETPIN0, INPUT);
		digitalWrite(DETPIN0, HIGH);

		pad |= ((!digitalRead(DETPIN0)) << 3);

		digitalWrite(DETPIN0, LOW);
	}

	return pad;
}

// Genesis pad loop
void genesis_loop() {
	int button_data;

	genesis_init();

	for (;;) {
		button_data = genesis_read();

		left = button_data & GENESIS_LEFT;
		right = button_data & GENESIS_RIGHT;
		up = button_data & GENESIS_UP;
		down = button_data & GENESIS_DOWN;
		circle = button_data & GENESIS_C;
		cross = button_data & GENESIS_B;
		triangle = button_data & GENESIS_Y;
		sqre = button_data & GENESIS_A;
		l1 = button_data & GENESIS_X;
		r1 = button_data & GENESIS_Z;
		select = button_data & GENESIS_MODE;
		start = button_data & GENESIS_START;

		pspad_set_pad_state(left, right, up, down, sqre, triangle, circle, cross,
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry);

	}
}

// NES pad loop
void nes_loop() {
	int button_data;

	NESPad::init();

	for (;;) {

		button_data = NESPad::read(8);

		left = button_data & 64;
		right = button_data & 128;
		up = button_data & 16;
		down = button_data & 32;
		cross = button_data & 1;
		sqre = button_data & 2;
		select = button_data & 4;
		start = button_data & 8;

		pspad_set_pad_state(left, right, up, down, sqre, triangle, circle, cross,
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry);
	}
}

// SNES pad loop
void snes_loop() {
	int button_data;

	NESPad::init();

	for (;;) {
		button_data = NESPad::read(16);

		left = button_data & 64;
		right = button_data & 128;
		up = button_data & 16;
		down = button_data & 32;
		cross = button_data & 1;
		sqre = button_data & 2;
		select = button_data & 4;
		start = button_data & 8;
		circle = button_data & 256;
		triangle = button_data & 512;
		l1 = button_data & 1024;
		r1 = button_data & 2048;

		pspad_set_pad_state(left, right, up, down, sqre, triangle, circle, cross,
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry);
	}
}

// PS2 pad loop
void ps2_loop() {

	while (PS2Pad::init(false));

	PS2Pad::read();

	for (;;) {
		PS2Pad::read();

		left = PS2Pad::button(PSB_PAD_LEFT);
		right = PS2Pad::button(PSB_PAD_RIGHT);
		up = PS2Pad::button(PSB_PAD_UP);
		down = PS2Pad::button(PSB_PAD_DOWN);

		sqre = PS2Pad::button(PSB_SQUARE);
		cross = PS2Pad::button(PSB_CROSS);
		triangle = PS2Pad::button(PSB_TRIANGLE);
		circle = PS2Pad::button(PSB_CIRCLE);

		l1 = PS2Pad::button(PSB_L1);
		r1 = PS2Pad::button(PSB_R1);

		l2 = PS2Pad::button(PSB_L2);
		r2 = PS2Pad::button(PSB_R2);

		l3 = PS2Pad::button(PSB_L3);
		r3 = PS2Pad::button(PSB_R3);

		select = PS2Pad::button(PSB_SELECT);
		start = PS2Pad::button(PSB_START);

		lx = PS2Pad::stick(PSS_LX);
		ly = PS2Pad::stick(PSS_LY);
		rx = PS2Pad::stick(PSS_RX);
		ry = PS2Pad::stick(PSS_RY);

		pspad_set_pad_state(left, right, up, down, sqre, triangle, circle, cross,
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry);
	}
}

void neogeo_loop() {
	int button_data;

	NESPad::init();

	for (;;) {
		button_data = NESPad::read(16);

		left = button_data & 0x02;
		right = button_data & 0x800;
		up = button_data & 0x04;
		down = button_data & 0x1000;
		cross = button_data & 0x01;
		sqre = button_data & 0x8000;
		select = button_data & 0x100;
		start = button_data & 0x4000;
		circle = button_data & 0x400;
		cross = button_data & 0x200; // D button is also 0x2000

		pspad_set_pad_state(left, right, up, down, sqre, triangle, circle, cross,
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry);
	}
}

void saturn_loop() {
	int button_data;

	saturn_init();

	for (;;) {
		button_data = saturn_read();

		left = button_data & SATURN_LEFT;
		right = button_data & SATURN_RIGHT;
		up = button_data & SATURN_UP;
		down = button_data & SATURN_DOWN;
		circle = button_data & SATURN_C;
		cross = button_data & SATURN_B;
		triangle = button_data & SATURN_Y;
		sqre = button_data & SATURN_A;
		l1 = button_data & SATURN_X;
		r1 = button_data & SATURN_Z;

		l2 = button_data & SATURN_L;
		r2 = button_data & SATURN_R;

		start = button_data & SATURN_START;

		pspad_set_pad_state(left, right, up, down, sqre, triangle, circle, cross,
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry);
	}
}

void wiicc_loop() {
	byte *button_data;
	byte *cal_data;
	byte _lx, _ly, _rx, _ry;
	byte lx_min, lx_max,rx_min, rx_max;
	byte ly_min, ly_max, ry_min, ry_max;

	while(!wiicc_init());

	cal_data = wiicc_calibration_data();

	lx_min = cal_data[1] >> 2;
	lx_max = cal_data[0] >> 2;

	ly_min = cal_data[4] >> 2;
	ly_max = cal_data[3] >> 2;

	rx_min = cal_data[7] >> 3;
	rx_max = cal_data[6] >> 3;

	ry_min = cal_data[10] >> 3;
	ry_max = cal_data[9] >> 3;

	for(;;) {
		button_data = wiicc_update();

		up = ~button_data[5] & (1 << 0);
		left = ~button_data[5] & (1 << 1);
		r2 = ~button_data[5] & (1 << 2);
		triangle = ~button_data[5] & (1 << 3);
		circle = ~button_data[5] & (1 << 4);
		sqre = ~button_data[5] & (1 << 5);
		cross = ~button_data[5] & (1 << 6);
		l2 = ~button_data[5] & (1 << 7);
		r1 = ~button_data[4] & (1 << 1);
		start = ~button_data[4] & (1 << 2);
		r3 = ~button_data[4] & (1 << 3);
		select = ~button_data[4] & (1 << 4);
		l1 = ~button_data[4] & (1 << 5);
		down = ~button_data[4] & (1 << 6);
		right = ~button_data[4] & (1 << 7);

		_rx = ((button_data[2] & 0x80) >> 7) | ((button_data[1] & 0xC0) >> 5) | ((button_data[0] & 0xC0) >> 3);
		_ry = (button_data[2] & 0x1F);
		_lx = (button_data[0] & 0x3F);
		_ly = (button_data[1] & 0x3F);

		if(_rx < rx_min) {
			_rx = rx_min;
		} else if(_rx > rx_max) {
			_rx = rx_max;
		}

		if(_ry < ry_min) {
			_ry = ry_min;
		} else if(_ry > ry_max) {
			_ry = ry_max;
		}

		if(_lx < lx_min) {
			_lx = lx_min;
		} else if(_lx > lx_max) {
			_lx = lx_max;
		}

		if(_ly < ly_min) {
			_ly = ly_min;
		} else if(_ly > ly_max) {
			_ly = ly_max;
		}

		rx = map(_rx, rx_min, rx_max, 0, 255);
		ry = ~map(_ry, ry_min, ry_max, 0, 255);
		lx = map(_lx, lx_min, lx_max, 0, 255);
		ly = ~map(_ly, ly_min, ly_max, 0, 255);

		pspad_set_pad_state(left, right, up, down, sqre, triangle, circle, cross,
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry);

	}

}

void gc_loop_helper(void) {
	GCPad_read(false);
}

void gc_loop() {
	byte *button_data;

	GCPad_init();

	GCPad_read(true);
	button_data = GCPad_data();

	pspad_set_spi_callback(gc_loop_helper);

	for(;;) {
		button_data = GCPad_data();

		left = button_data[1] & 0x01;
		right = button_data[1] & 0x02;
		up = button_data[1] & 0x08;
		down = button_data[1] & 0x04;

		sqre = button_data[0] & 0x08;
		cross = button_data[0] & 0x02;
		triangle = button_data[0] & 0x04;
		circle = button_data[0] & 0x01;

		start = button_data[0] & 0x10;

		l1 = button_data[1] & 0x40;
		r1 = button_data[1] & 0x20;

		l2 = r2 = button_data[1] & 0x10;

		lx = button_data[2];
		ly = ~button_data[3];
		rx = button_data[4];
		ry = ~button_data[5];

		pspad_set_pad_state(left, right, up, down, sqre, triangle, circle, cross,
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry);

	}
}

void n64_loop_helper(void) {
	N64Pad_read(false);
}

void n64_loop() {
	byte *button_data;

	GCPad_init();

	N64Pad_read(true);
	button_data = N64Pad_data();

	pspad_set_spi_callback(n64_loop_helper);

	for(;;) {
		button_data = N64Pad_data();

		left = button_data[0] & 0x02;
		right = button_data[0] & 0x01;
		up = button_data[0] & 0x08;
		down = button_data[0] & 0x04;

		sqre = button_data[0] & 0x40;
		cross = button_data[0] & 0x80;

		start = button_data[0] & 0x10;

		r1 = button_data[1] & 0x10;
		l1 = button_data[1] & 0x20;
		l2 = r2 = button_data[0] & 0x20;


		ry = 0x7F;

		if(button_data[1] & 0x08) { // C Up
			ry = 0x00;
		} else if(button_data[1] & 0x04) { // C Down
			ry = 0xFF;
		}

		rx = 0x7F;

		if(button_data[1] & 0x02) { // C Left
			rx = 0x00;
		} else if(button_data[1] & 0x01) { // C Right
			rx = 0xFF;
		}

		triangle = button_data[1] & 0x02; // triangle == C Left
		circle = button_data[1] & 0x01; // circle == C Right

		lx = ((button_data[2] >= 128) ? button_data[2] - 128 : button_data[2] + 128);
		ly = ~(((button_data[3] >= 128) ? button_data[3] - 128 : button_data[3] + 128));

		pspad_set_pad_state(left, right, up, down, sqre, triangle, circle, cross,
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry);

	}
}


void setup() {
	// Init PS Pad emulation
	pspad_init();
}

void loop() {
	// Select pad loop based on pad auto-detection routine. Genesis pad is the default.
	switch (detectPad()) {
	case PAD_NES:
		nes_loop();
		break;
	case PAD_SNES:
		snes_loop();
		break;
	case PAD_PS2:
		ps2_loop();
		break;
	case PAD_GC:
		gc_loop();
		break;
	case PAD_N64:
		n64_loop();
		break;
	case PAD_NEOGEO:
		neogeo_loop();
		break;
	case PAD_SATURN:
		saturn_loop();
		break;
	case PAD_WIICC:
		wiicc_loop();
		break;
	default:
		genesis_loop();
		break;
	}
}
