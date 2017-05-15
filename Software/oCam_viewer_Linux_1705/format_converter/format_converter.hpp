/*******************************************************************************#
#           format_converter      http://www.withrobot.com                      #
#                                                                               #
#           Daehong Min <daehong@withrobot.com>                                 #
#                                                                               #
# This program is free software; you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation; either version 2 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
#                                                                               #
# You should have received a copy of the GNU General Public License             #
# along with this program; if not, write to the Free Software                   #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA     #
#                                                                               #
********************************************************************************/

/*
 * format_converter.hpp
 *
 *  Created on: Dec 7, 2015
 *      Author: gnohead
 */

#ifndef FORMAT_CONVERTER_HPP_
#define FORMAT_CONVERTER_HPP_


/**
 * C++ wrapping class of Guvcview's format converter.
 *
 * M/JPEG decode and yuyv decode taken from guvcview
 */
#include "jpeg_decoder.h"
#include "colorspaces.h"

class GuvcviewFormatConverter
{
private:
    int width;
    int height;

	unsigned char* yuyv_buffer;

public:
    //pix_order: bayer pixel order (0=gb/rg   1=gr/bg  2=bg/gr  3=rg/bg)
    enum _BAYER_PIX_ORDER {
        BAYER2RGB_GBRG = 0,
        BAYER2RGB_GRBG,
        BAYER2RGB_BGGR,
        BAYER2RGB_RGBG,
    };

public:
    GuvcviewFormatConverter(int width, int height) : width(width), height(height), yuyv_buffer(0) {
		jpeg_init_decoder(width, height);
	}

	~GuvcviewFormatConverter() {
		jpeg_close_decoder();
		delete_buffers();
	}

	void yuyv_to_bgr(unsigned char* bgr, unsigned char* yuyv) {
        yuyv2bgr(yuyv, bgr, width, height);
	}

	void jpeg_to_bgr(unsigned char* bgr, unsigned char* jpeg, const unsigned int jpeg_size) {
		init_buffers();
		jpeg_decode(yuyv_buffer, jpeg, jpeg_size);
		yuyv_to_bgr(bgr, yuyv_buffer);
	}

	void yuyv_to_rgb(unsigned char* rgb, unsigned char* yuyv) {
        yuyv2rgb(yuyv, rgb, width, height);
	}

	void jpeg_to_rgb(unsigned char* rgb, unsigned char* jpeg, const unsigned int jpeg_size) {
		init_buffers();
		jpeg_decode(yuyv_buffer, jpeg, jpeg_size);
		yuyv_to_rgb(rgb, yuyv_buffer);
	}

    void grey_to_rgb(unsigned char* rgb, unsigned char* grey) {
        init_buffers();

        for(int h=0; h<height; h++)
        {
            int offset = width * h;
            for(int w=0; w<width; w++)
            {
                rgb[(w + offset)*3]   = grey[w + offset];
                rgb[(w + offset)*3+1] = grey[w + offset];
                rgb[(w + offset)*3+2] = grey[w + offset];
            }
        }
    }

    void grey_to_bgr(unsigned char* bgr, unsigned char* grey) {
        init_buffers();
        grey_to_yuyv(yuyv_buffer, grey, width, height);
        yuyv_to_bgr(bgr, yuyv_buffer);
    }

    void bayer_to_rgb(unsigned char* bayer, unsigned char* rgb, int pix_order) {
        /*
         * convert bayer raw data to rgb24
         * args:
         *   pBay: pointer to buffer containing Raw bayer data
         *   pRGB24: pointer to buffer containing rgb24 data
         *   width: picture width
         *   height: picture height
         *   pix_order: bayer pixel order (0=gb/rg   1=gr/bg  2=bg/gr  3=rg/bg)
         *
         * asserts:
         *   none
         *
         * returns: none
         */
        bayer_to_rgb24(bayer, rgb, width, height, pix_order);
    }

    void bgr_to_rgb(unsigned char* bgr, unsigned char* rgb) {
        for (int i=0; i < width*height*3; i+=3) {
            rgb[i]   = bgr[i+2];
            rgb[i+1] = bgr[i+1];
            rgb[i+2] = bgr[i];
        }
    }

private:
	void init_buffers() {
		if (yuyv_buffer == 0) {
            yuyv_buffer = new unsigned char[width*height*2];
		}
	}

	void delete_buffers() {
		if (yuyv_buffer != 0) {
			delete[] yuyv_buffer;
		}
		yuyv_buffer = 0;
	}
};


#endif /* FORMAT_CONVERTER_HPP_ */
