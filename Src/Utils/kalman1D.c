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

#include "kalman1D.h"

void kalmanInit(kalman_t* self, float dt, float sigz, float sigu){
	self->dt  = dt;
	self->Xn  = mzero(2, 1);
	self->Xn1 = mzero(2, 1);

	self->Kn  = mzero(2, 1);

	self->Pn  = meye(2);
	self->Pn1 = mzero(2, 2);

	self->F = meye(2);
	self->F.mx[0][1] = dt;

	self->G = mzero(2, 1);
	self->G.mx[0][0] = dt*dt*0.5f;
	self->G.mx[1][0] = dt;

	self->Q = mnew(2, 2);
	self->Q.mx[0][0] = dt*dt*dt*dt / 4.0f;
	self->Q.mx[0][1] = dt*dt*dt    / 3.0f;
	self->Q.mx[1][0] = dt*dt*dt    / 3.0f;
	self->Q.mx[1][1] = dt*dt       / 2.0f;
	self->Q = mscl(self->Q, sigu*sigu);

	self->R = mnew(1, 1);
	self->R.mx[0][0] = sigz*sigz;

	self->I = meye(2);
	self->H = mzero(1, 2);
	self->H.mx[0][0] = 1;
}

void kalmanIterate(kalman_t* self, float zn, float un){
	kalmanKGain(self);
	kalmanStateUpdate(self, zn);
	kalmanCovUpdate(self);
	kalmanStateExtrapolation(self, un);
	kalmanCovExtrapolation(self);
}

void kalmanKGain(kalman_t* self){
	matrix_t A  = mdot(self->Pn, mtrans(self->H)); /* 2x2 * 2x1 = 2x1 */
	matrix_t B  = mdot(self->H,   A);              /* 1x2 * 2x1 = 1x1 */
	B.mx[0][0]  = 1.0f/(B.mx[0][0] + self->R.mx[0][0]); /* Inverse */
	self->Kn    = mdot(A, B);
}

void kalmanStateUpdate(kalman_t* self, float zn){
	matrix_t A = mdot(self->H, self->Xn1); /* 1x2 * 2x1 = 1x1 */
	A.mx[0][0] = zn - A.mx[0][0];
    self->Xn = madd(self->Xn1, mdot(self->Kn, A));
}

void kalmanCovUpdate(kalman_t* self){
	matrix_t A = msub(self->I, mdot(self->Kn, self->H)); /* 2x1 * 1x2 = 2x2 */
	self->Pn = madd(mdot(A, mdot(self->Pn1, mtrans(A)))  ,  mdot(self->Kn, mdot(self->R, mtrans(self->Kn))));
}

void kalmanStateExtrapolation(kalman_t* self, float un){
	self->Xn1 = madd(mdot(self->F, self->Xn), mscl(self->G, un));
}

void kalmanCovExtrapolation(kalman_t* self){
	self->Pn1 = madd(mdot(self->F, mdot(self->Pn, mtrans(self->F))) , self->Q);
}
