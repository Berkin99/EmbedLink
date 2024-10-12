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

#ifndef KALMAN1D_H_
#define KALMAN1D_H_

#include "matrix.h"

typedef struct{
	float dt;
	matrix_t Xn;
	matrix_t Xn1;
	matrix_t Kn;
	matrix_t Pn;
	matrix_t Pn1;
	matrix_t F;
	matrix_t G;
	matrix_t Q;
	matrix_t R;
	matrix_t I;
	matrix_t H;
}kalman_t;

void kalmanInit(kalman_t* self, float dt, float sigz, float sigu);
void kalmanKGain(kalman_t* self);
void kalmanStateUpdate(kalman_t* self, float zn);
void kalmanCovUpdate(kalman_t* self);
void kalmanStateExtrapolation(kalman_t* self, float un);
void kalmanCovExtrapolation(kalman_t* self);
void kalmanIterate(kalman_t* self, float zn, float un);

#endif
