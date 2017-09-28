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

#include <stdio.h>
#include <stdarg.h>

//#define PRINT_DEBUG_MSG

#ifdef PRINT_DEBUG_MSG

#define DBG_PRINTF(...) {\
    fprintf(stdout, "DBG: [%s, %d, %s] ", __FILE__, __LINE__, __FUNCTION__); \
    fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); fflush(stdout); \
}

#define DBG_PRINTF_MSG(...) { fprintf(stdout, __VA_ARGS__); fflush(stdout); }

#else

#define DBG_PRINTF(...)
#define DBG_PRINTF_MSG(...)

#endif /* PRINT_DEBUG_MSG */

#define DBG_PERROR(...) { fflush(stdout); printf("DBG_ERR: [%s, %d, %s] ", __FILE__, __LINE__, __func__); fflush(stdout); perror(__VA_ARGS__); fflush(stdout); }


#endif /* DEBUG_PRINT_H_ */
