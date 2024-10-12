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

#include "matrix.h"

matrix_t mnew(uint8_t row, uint8_t col){
	matrix_t m;
	m.row = row;
	m.col = col;
	return m;
}

matrix_t mrepeat(uint8_t row, uint8_t col, float v){
	matrix_t m = mnew(row, col);
	for (uint8_t r = 0; r < row; r++) {
		for (uint8_t c = 0; c < col; c++) {
			m.mx[r][c] = v;
		}
	}
	return m;
}

matrix_t mzero(uint8_t row, uint8_t col){
	return mrepeat(row, col, 0);
}

matrix_t mdiag(uint8_t  n, float v){
	matrix_t m = mzero(n, n);
	for (uint8_t i = 0; i < n; i++) {
		m.mx[i][i] = v;
	}
	return m;
}

matrix_t meye(uint8_t n){
	return mdiag(n, 1);
}

matrix_t madd(matrix_t m1, matrix_t m2){
	matrix_t m = mnew(m1.row, m1.col);
	for (uint8_t r = 0; r < m1.row; r++) {
		for (uint8_t c = 0; c < m1.col; c++) {
			m.mx[r][c] = m1.mx[r][c] + m2.mx[r][c];
		}
	}
	return m;
}

matrix_t msub (matrix_t m1, matrix_t m2){
	matrix_t m = mnew(m1.row, m1.col);
	for (uint8_t r = 0; r < m1.row; r++) {
		for (uint8_t c = 0; c < m1.col; c++) {
			m.mx[r][c] = m1.mx[r][c] - m2.mx[r][c];
		}
	}
	return m;
}

matrix_t mdot (matrix_t m1, matrix_t m2){
	matrix_t m = mnew(m1.row, m2.col);
	for (uint8_t r = 0; r < m1.row; r++) {
		for (uint8_t c = 0; c < m2.col; c++) {
			float sum = 0;
			for (uint8_t i = 0; i < m1.col; i++) {
				sum += m1.mx[r][i] * m2.mx[i][c];
			}
			m.mx[r][c] = sum;
		}
	}
	return m;
}

matrix_t mtrans (matrix_t m1){
	matrix_t m = mnew(m1.col, m1.row);
    for (uint8_t r = 0; r < m1.row; r++) {
        for (uint8_t c = 0; c < m1.col; c++) {
            m.mx[c][r] = m1.mx[r][c];
        }
    }
    return m;
}

matrix_t mscl (matrix_t m1, float scl){
	matrix_t m = mnew(m1.row, m1.col);
    for (uint8_t r = 0; r < m1.row; r++) {
        for (uint8_t c = 0; c < m1.col; c++) {
            m.mx[r][c] = m1.mx[r][c]*scl;
        }
    }
    return m;
}

