/*
 * PSX RetroPad Adapter - PSX/PS2 adapter for retro-controllers!
 * Copyright (c) 2012 Bruno Freitas - bruno@brunofreitas.com
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
#include <Wire.h>

static byte button_data[16];
static byte calibration_data[12];

void wiicc_i2c_send_array(byte *data, int size) {
	Wire.beginTransmission(0x52);

	for(int i = 0; i < size; i++) {
		Wire.send(data[i]);
	}

	Wire.endTransmission();

	delayMicroseconds(1000);
}

void wiicc_i2c_send_byte(byte data) {
	Wire.beginTransmission(0x52);

	Wire.send(data);

	Wire.endTransmission();

	delayMicroseconds(1000);
}

void wiicc_i2c_recv_array(byte *data, int size) {
	int i = 0;

	Wire.beginTransmission(0x52);

	Wire.requestFrom(0x52, size);

	while (Wire.available()) {
		data[i] = Wire.receive();
		i++;
	}

	Wire.endTransmission();

	delayMicroseconds(1000);
}

bool wiicc_init(void) {
	static byte disable_encryption_cmd[] = { 0xF0, 0x55, 0xFB, 0x00 };
	byte extension_id[6];
	static byte cc_id[] = { 0x00, 0x00, 0xa4, 0x20, 0x01, 0x01 };

	memset(button_data, 0x00, 16);

	Wire.begin();

	wiicc_i2c_send_array(&disable_encryption_cmd[0], 2);
	wiicc_i2c_send_array(&disable_encryption_cmd[2], 2);

	// read the extension type
	wiicc_i2c_send_byte(0xFA);
	wiicc_i2c_recv_array(extension_id, 6);

	if(memcmp(extension_id, cc_id, 6) != 0) {
		return false;
	}

	// read calibration data
	wiicc_i2c_send_byte(0x20);
	wiicc_i2c_recv_array(calibration_data, 6);
	wiicc_i2c_send_byte(0x26);
	wiicc_i2c_recv_array(calibration_data + 6, 6);

	delayMicroseconds(1000);

	return true;
}

byte *wiicc_update(void) {
	wiicc_i2c_send_byte(0x00);
	wiicc_i2c_recv_array(button_data, 6);

	return button_data;
}

byte *wiicc_calibration_data(void) {
	return calibration_data;
}

