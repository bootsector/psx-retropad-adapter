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

#ifndef PSPADEMU_H_
#define PSPADEMU_H_

void pspad_init(int mode);
void pspad_ss_int_handler(void);
void pspad_set_pad_state(int left, int right, int up, int down, int square, int triangle, int circle, int cross, int select, int start, int l1, int l2, int r1, int r2, int l3, int r3, int lx, int ly, int rx, int ry);
void pspad_set_spi_callback(void (*callback)(void));
void pspad_set_mode(int mode);
void pspad_toggle_mode(void);
int pspad_get_mode(void);

#define PSPADEMU_MODE_ANALOG  0
#define PSPADEMU_MODE_DIGITAL 1

#endif /* PSPADEMU_H_ */
