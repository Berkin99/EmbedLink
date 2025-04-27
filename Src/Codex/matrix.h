/*
 *       ______          __             ____    _       __
 *      / ____/___ ___  / /_  ___  ____/ / /   (_)___  / /__
 *     / __/ / __ `__ \/ __ \/ _ \/ __  / /   / / __ \/ //_/
 *    / /___/ / / / / / /_/ /  __/ /_/ / /___/ / / / / ,<
 *   /_____/_/ /_/ /_/_.___/\___/\__,_/_____/_/_/ /_/_/|_|
 *
 *  EmbedLink Firmware
 *  Copyright (c) 2024 Yeniay RD, All rights reserved.
 *  _________________________________________________________
 *
 *  EmbedLink Firmware is free software: you can redistribute
 *  it and/or  modify it under  the  terms of the  GNU Lesser
 *  General Public License as  published by the Free Software
 *  Foundation,  either version 3 of the License, or (at your
 *  option) any later version.
 *
 *  EmbedLink  Firmware is  distributed  in the  hope that it
 *  will be useful, but  WITHOUT  ANY  WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.  See  the GNU  Lesser  General Public
 *  License for more details.
 *
 *  You should have received a copy of the GNU Lesser General
 *  Public License along with EmbedLink Firmware. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include <stdint.h>

typedef struct{
	uint8_t row;       /* Number of rows */
	uint8_t col;       /* Number of columns */
	float   mx[3][3];  /* mx[rowno][colno] */
}matrix_t;

matrix_t mnew    (uint8_t row, uint8_t col);
matrix_t mrepeat (uint8_t row, uint8_t col, float v);
matrix_t mzero   (uint8_t row, uint8_t col);
matrix_t mdiag   (uint8_t  n, float v);
matrix_t meye    (uint8_t  n);
matrix_t madd    (matrix_t m1, matrix_t m2);
matrix_t msub    (matrix_t m1, matrix_t m2);
matrix_t mdot    (matrix_t m1, matrix_t m2);
matrix_t mtrans  (matrix_t m1);
matrix_t mscl    (matrix_t m1, float scl);
matrix_t minv	 (matrix_t m);

#endif
