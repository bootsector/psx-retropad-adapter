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

#include "tg16.h"
#include "digitalWriteFast.h"

/*
 * WRA -  DB9  - PSXRA
 * 2   - DB9P1 - A4
 * 4   - DB9P3 - 0
 * 5   - DB9P4 - 1
 * 6   - DB9P6 - 4
 * 7   - DB9P7 - 5
 * 8   - DB9P9 - 6
 */

void tg16_init(void) {
	// Configure Data pins
	pinModeFast(A4, INPUT);
	digitalWriteFast(A4, HIGH);

	pinModeFast(0, INPUT);
	digitalWriteFast(0, HIGH);

	pinModeFast(1, INPUT);
	digitalWriteFast(1, HIGH);

	pinModeFast(4, INPUT);
	digitalWriteFast(4, HIGH);

	// Configure Data Select and /OE pins
	pinModeFast(5, OUTPUT);
	digitalWriteFast(5, HIGH);

	pinModeFast(6, OUTPUT);
	digitalWriteFast(6, LOW);
}

int tg16_read(void) {
	int retval = 0;

	// Data Select HIGH
	digitalWriteFast(5, HIGH);

	// /OE LOW
	digitalWriteFast(6, LOW);

	for(int i = 0; i < 2; i++) {
		// /OE LOW
		digitalWriteFast(6, LOW);
		delayMicroseconds(1);

		// If four directions are low, then it's an Avenue6 Pad
		if(!digitalReadFast(A4) && !digitalReadFast(0) && !digitalReadFast(1) && !digitalReadFast(4)) {
			// Data Select LOW
			digitalWriteFast(5, LOW);
			delayMicroseconds(1);

			retval |= (!digitalReadFast(A4) << 8);  // III
			retval |= (!digitalReadFast(0) << 9);  // IV
			retval |= (!digitalReadFast(1) << 10); // V
			retval |= (!digitalReadFast(4) << 11); // VI
		} else {
			// Normal pad reading
			retval |= (!digitalReadFast(A4) << 0); // UP
			retval |= (!digitalReadFast(0) << 1); // RIGHT
			retval |= (!digitalReadFast(1) << 2); // DOWN
			retval |= (!digitalReadFast(4) << 3); // LEFT

			// Data Select LOW
			digitalWriteFast(5, LOW);
			delayMicroseconds(1);

			retval |= (!digitalReadFast(A4) << 4); // I
			retval |= (!digitalReadFast(0) << 5); // II
			retval |= (!digitalReadFast(1) << 6); // SELECT
			retval |= (!digitalReadFast(4) << 7); // RUN
		}

		// Data Select HIGH
		digitalWriteFast(5, HIGH);

		// /OE HIGH
		digitalWriteFast(6, HIGH);
	}

	return retval;
}
