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
#include "tg16.h"

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
#define DETPIN0 A5 // DB9P2
#define DETPIN1	1  // DB9P4
#define DETPIN2	4  // DB9P6
#define DETPIN3	5  // DB9P7
#define DETPIN4	6  // DB9P9

// Possible values (as of today) returned by the detectPad() routine
// Normal pads
#define PAD_GENESIS		0b00111
#define PAD_NES 		0b00110
#define PAD_SNES 		0b00101
#define PAD_PS2 		0b00100
#define PAD_GC	 		0b00011
#define PAD_N64			0b00010
#define PAD_NEOGEO		0b00001
#define PAD_WIICC		0b00000
// Extended pads (uses DB9 pin 4 and/or 2 for identification)
#define PAD_SATURN		0b01111
#define PAD_TG16		0b10111
#define PAD_DFU_DONGLE	0b01110 // Reserved for USBRA DFU dongle

/*
 * This is the new auto-detect function (non jumper based) which detects the extension
 * cable plugged in the DB9 port. It uses grounded pins from DB9 (4, 6, 7 and 9) for
 * the detection.
 *
 * 00111 - Sega Genesis (Default)
 * 00110 - NES
 * 00101 - SNES
 * 00100 - PS2
 * 00011 - Game Cube
 * 00010 - Nintendo 64
 * 00001 - Neo Geo
 * 00000 - Reserved 1
 * 01111 - Sega Saturn
 * 10111 - TurboGrafx 16
 */
int detectPad() {
	int pad;

	// Set pad/arcade detection pins as input, turning pull-ups on
	pinMode(DETPIN0, INPUT);
	digitalWrite(DETPIN0, HIGH);

	pinMode(DETPIN1, INPUT);
	digitalWrite(DETPIN1, HIGH);

	pinMode(DETPIN2, INPUT);
	digitalWrite(DETPIN2, HIGH);

	pinMode(DETPIN3, INPUT);
	digitalWrite(DETPIN3, HIGH);

	pinMode(DETPIN4, INPUT);
	digitalWrite(DETPIN4, HIGH);

	pad = (!digitalRead(DETPIN0) << 4) | (!digitalRead(DETPIN1) << 3) | (digitalRead(DETPIN2) << 2) | (digitalRead(DETPIN3) << 1) | (digitalRead(DETPIN4));

	if((pad >> 3) & 0b11) {
		switch(pad) {
		case 0b11011:
		case 0b10111:
			return PAD_TG16;
			break;
		case 0b11111:
		case 0b01111:
			return PAD_SATURN;
			break;
		case 0b11100:
			return PAD_PS2;
			break;
		default:
			return PAD_GENESIS;
			break;
		}
	}

	return (pad & 0b111);
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
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry, up && start);

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
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry, select && start);
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
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry, select && start);
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
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry, select && start);
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
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry, select && start);
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
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry, up && start);
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
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry, select && start);

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
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry, up && start);

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
				select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry, up && start);

	}
}

// TG16 pad loop
void tg16_loop() {
	int button_data;

	tg16_init();

	for (;;) {

		button_data = tg16_read();

		left = button_data & (1 << TG16_LEFT);
		right = button_data & (1 << TG16_RIGHT);
		up = button_data & (1 << TG16_UP);
		down = button_data & (1 << TG16_DOWN);
		circle = button_data & (1 << TG16_I);
		cross = button_data & (1 << TG16_II);
		select = button_data & (1 << TG16_SELECT);
		start = button_data & (1 << TG16_RUN);

		pspad_set_pad_state(left, right, up, down, sqre, triangle, circle, cross,
						select, start, l1, l2, r1, r2, l3, r3, lx, ly, rx, ry, select && start);
	}
}

void setup() {
	// disable timer 0 overflow interrupt (enabled by Arduino's init() function).
	// WARNING: This will mess up with micros(), millis() and delay() Arduino functions!
	// Use alternate timer functions instead!
#if defined(TIMSK) && defined(TOIE0)
	(_SFR_BYTE(TIMSK) &= ~_BV(TOIE0));
#elif defined(TIMSK0) && defined(TOIE0)
	(_SFR_BYTE(TIMSK0) &= ~_BV(TOIE0));
#endif

	// Init PS Pad emulation
	pspad_init(PSPADEMU_MODE_ANALOG);
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
	case PAD_TG16:
		tg16_loop();
		break;
	case PAD_WIICC:
		wiicc_loop();
		break;
	default:
		genesis_loop();
		break;
	}
}
