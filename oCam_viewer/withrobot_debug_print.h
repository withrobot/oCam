/*******************************************************************************#
#                                                                               #
# Withrobot Camera API                                                          #
#                                                                               #
# Copyright (C) 2015 Withrobot. Inc.                                            #
#                                                                               #
# This program is free software: you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation, either version 3 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
# You should have received a copy of the GNU General Public License             #
# along with this program.  If not, see <http://www.gnu.org/licenses/>          #
#                                                                               #
********************************************************************************/

/*
 * withrobot_debug_print.h
 *
 *  Created on: Oct 6, 2015
 *      Author: gnohead
 */

#ifndef DEBUG_PRINT_H_
#define DEBUG_PRINT_H_

#include <iostream>
#include <stdio.h>

#if 0
#define DBG_PRINT(...) { fflush(stdout); std::cout << "DBG: [" << __FILE__ << ", " << __LINE__ << ", " << __func__ << "] " << __VA_ARGS__ << std::endl; fflush(stdout);}

#define DBG_PRINTF(...) { fflush(stdout); printf("DBG: [%s, %d, %s] ", __FILE__, __LINE__, __func__); fflush(stdout); printf(__VA_ARGS__); printf("\n"); fflush(stdout);}
#else
#define DBG_PRINT(...) {}
#define DBG_PRINTF(...) {}
#endif


#define DBG_PERROR(...) { fflush(stdout); printf("DBG_ERR: [%s, %d, %s] ", __FILE__, __LINE__, __func__); fflush(stdout); perror(__VA_ARGS__); fflush(stdout); }


#endif /* DEBUG_PRINT_H_ */
