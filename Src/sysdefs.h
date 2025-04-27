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

#ifndef SYSDEFS_H_
#define SYSDEFS_H_

#define TRUE           (1)
#define FALSE          (0)
#define LOW		       FALSE
#define HIGH	       TRUE
#define INPUT          (0)
#define OUTPUT         (1)

#define OK             (0)
#define E_ERROR        (-1)
#define E_NULL_PTR     (-2)
#define E_OVERWRITE    (-3)
#define E_OVERFLOW     (-4)
#define E_CONNECTION   (-5)
#define E_CONF_FAIL    (-6)
#define E_NOT_FOUND    (-7)
#define E_TIMEOUT      (-8)
#define UNDEFINED      NULL

#endif /* SYSDEFS_H_ */
