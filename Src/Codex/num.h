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

#ifndef NUM_H_
#define NUM_H_

#include <stdint.h>
#include <math.h>

static inline float constrainFloat(float val, const float min, const float max){
	return fminf(max, fmaxf(min,val));
}

static inline int constrainInt(int val,const int min, const int max){
    if (val < min)return min;
    if (val > max)return max;
    return val;
}

static inline float deadbandFloat(float val, const float threshold){
  if (fabsf(val) < threshold) val = 0;
  else if (val > 0) val -= threshold;
  else if (val < 0) val += threshold;
  return val;
}

static inline float deadbandInt(int val, const int threshold){
  if (fabsf(val) < threshold)  val = 0;
  else if (val > 0) val -= threshold;
  else if (val < 0) val += threshold;
  return val;
}

static inline float weightedSum(float w1, float w2, float value1, float value2){
	return (w1 * value1 + w2 * value2);
}

static inline float weightedAvarage(float w1, float w2, float value1, float value2){
	return (w1 * value1 + w2 * value2) / (w1 + w2);
}

/* Degrees of angles course deviation. @target -170, @current 170 : out +20 */
static inline float courseDeviation(float target, float current){
	float course = target - current;
	if (course < -180) return (course + 360);
	if (course >  180) return (course - 360);
	return course;
}

#endif /* NUM_H_ */
