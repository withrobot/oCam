/*******************************************************************************#
#           guvcview              http://guvcview.sourceforge.net               #
#                                                                               #
#           Paulo Assis <pj.assis@gmail.com>                                    #
#           Nobuhiro Iwamatsu <iwamatsu@nigauri.org>                            #
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

#include "colorspaces.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>


/*------------------------------- Color space conversions --------------------*/

/*---------------- raw bayer -------------*/

/* raw bayer functions
 *  from libv4l bayer.c, (C) 2008 Hans de Goede <j.w.r.degoede@hhs.nl>
 *  Note: original bayer_to_bgr24 code from :
 *     1394-Based Digital Camera Control Library
 *
 * Bayer pattern decoding functions
 *  Written by Damien Douxchamps and Frederic Devernay
 *
 */
static void convert_border_bayer_line_to_bgr24( uint8_t* bayer, uint8_t* adjacent_bayer,
	uint8_t *bgr, int width, uint8_t start_with_green, uint8_t blue_line)
{
	int t0, t1;

	if (start_with_green)
	{
	/* First pixel */
		if (blue_line)
		{
			*bgr++ = bayer[1];
			*bgr++ = bayer[0];
			*bgr++ = adjacent_bayer[0];
		}
		else
		{
			*bgr++ = adjacent_bayer[0];
			*bgr++ = bayer[0];
			*bgr++ = bayer[1];
		}
		/* Second pixel */
		t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
		t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
		if (blue_line)
		{
			*bgr++ = bayer[1];
			*bgr++ = t0;
			*bgr++ = t1;
		}
		else
		{
			*bgr++ = t1;
			*bgr++ = t0;
			*bgr++ = bayer[1];
		}
		bayer++;
		adjacent_bayer++;
		width -= 2;
	}
	else
	{
		/* First pixel */
		t0 = (bayer[1] + adjacent_bayer[0] + 1) >> 1;
		if (blue_line)
		{
			*bgr++ = bayer[0];
			*bgr++ = t0;
			*bgr++ = adjacent_bayer[1];
		}
		else
		{
			*bgr++ = adjacent_bayer[1];
			*bgr++ = t0;
			*bgr++ = bayer[0];
		}
		width--;
	}

	if (blue_line)
	{
		for ( ; width > 2; width -= 2)
		{
			t0 = (bayer[0] + bayer[2] + 1) >> 1;
			*bgr++ = t0;
			*bgr++ = bayer[1];
			*bgr++ = adjacent_bayer[1];
			bayer++;
			adjacent_bayer++;

			t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
			t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
			*bgr++ = bayer[1];
			*bgr++ = t0;
			*bgr++ = t1;
			bayer++;
			adjacent_bayer++;
		}
	}
	else
	{
		for ( ; width > 2; width -= 2)
		{
			t0 = (bayer[0] + bayer[2] + 1) >> 1;
			*bgr++ = adjacent_bayer[1];
			*bgr++ = bayer[1];
			*bgr++ = t0;
			bayer++;
			adjacent_bayer++;

			t0 = (bayer[0] + bayer[2] + adjacent_bayer[1] + 1) / 3;
			t1 = (adjacent_bayer[0] + adjacent_bayer[2] + 1) >> 1;
			*bgr++ = t1;
			*bgr++ = t0;
			*bgr++ = bayer[1];
			bayer++;
			adjacent_bayer++;
		}
	}

	if (width == 2)
	{
		/* Second to last pixel */
		t0 = (bayer[0] + bayer[2] + 1) >> 1;
		if (blue_line)
		{
			*bgr++ = t0;
			*bgr++ = bayer[1];
			*bgr++ = adjacent_bayer[1];
		}
		else
		{
			*bgr++ = adjacent_bayer[1];
			*bgr++ = bayer[1];
			*bgr++ = t0;
		}
		/* Last pixel */
		t0 = (bayer[1] + adjacent_bayer[2] + 1) >> 1;
		if (blue_line)
		{
			*bgr++ = bayer[2];
			*bgr++ = t0;
			*bgr++ = adjacent_bayer[1];
		}
		else
		{
			*bgr++ = adjacent_bayer[1];
			*bgr++ = t0;
			*bgr++ = bayer[2];
		}
	}
	else
	{
		/* Last pixel */
		if (blue_line)
		{
			*bgr++ = bayer[0];
			*bgr++ = bayer[1];
			*bgr++ = adjacent_bayer[1];
		}
		else
		{
			*bgr++ = adjacent_bayer[1];
			*bgr++ = bayer[1];
			*bgr++ = bayer[0];
		}
	}
}

/*
 * From libdc1394, which on turn was based on OpenCV's Bayer decoding
 */
static void bayer_to_rgbbgr24(uint8_t *bayer,
	uint8_t *bgr, int width, int height,
	uint8_t start_with_green, uint8_t blue_line)
{
	/* render the first line */
	convert_border_bayer_line_to_bgr24(bayer, bayer + width, bgr, width,
		start_with_green, blue_line);
	bgr += width * 3;

	/* reduce height by 2 because of the special case top/bottom line */
	for (height -= 2; height; height--)
	{
		int t0, t1;
		/* (width - 2) because of the border */
		uint8_t *bayerEnd = bayer + (width - 2);

		if (start_with_green)
		{
			/* OpenCV has a bug in the next line, which was
			t0 = (bayer[0] + bayer[width * 2] + 1) >> 1; */
			t0 = (bayer[1] + bayer[width * 2 + 1] + 1) >> 1;
			/* Write first pixel */
			t1 = (bayer[0] + bayer[width * 2] + bayer[width + 1] + 1) / 3;
			if (blue_line)
			{
				*bgr++ = t0;
				*bgr++ = t1;
				*bgr++ = bayer[width];
			}
			else
			{
				*bgr++ = bayer[width];
				*bgr++ = t1;
				*bgr++ = t0;
			}

			/* Write second pixel */
			t1 = (bayer[width] + bayer[width + 2] + 1) >> 1;
			if (blue_line)
			{
				*bgr++ = t0;
				*bgr++ = bayer[width + 1];
				*bgr++ = t1;
			}
			else
			{
				*bgr++ = t1;
				*bgr++ = bayer[width + 1];
				*bgr++ = t0;
			}
			bayer++;
		}
		else
		{
			/* Write first pixel */
			t0 = (bayer[0] + bayer[width * 2] + 1) >> 1;
			if (blue_line)
			{
				*bgr++ = t0;
				*bgr++ = bayer[width];
				*bgr++ = bayer[width + 1];
			}
			else
			{
				*bgr++ = bayer[width + 1];
				*bgr++ = bayer[width];
				*bgr++ = t0;
			}
		}

		if (blue_line)
		{
			for (; bayer <= bayerEnd - 2; bayer += 2)
			{
				t0 = (bayer[0] + bayer[2] + bayer[width * 2] +
					bayer[width * 2 + 2] + 2) >> 2;
				t1 = (bayer[1] + bayer[width] +
					bayer[width + 2] + bayer[width * 2 + 1] +
					2) >> 2;
				*bgr++ = t0;
				*bgr++ = t1;
				*bgr++ = bayer[width + 1];

				t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
				t1 = (bayer[width + 1] + bayer[width + 3] +
					1) >> 1;
				*bgr++ = t0;
				*bgr++ = bayer[width + 2];
				*bgr++ = t1;
			}
		}
		else
		{
			for (; bayer <= bayerEnd - 2; bayer += 2)
			{
				t0 = (bayer[0] + bayer[2] + bayer[width * 2] +
					bayer[width * 2 + 2] + 2) >> 2;
				t1 = (bayer[1] + bayer[width] +
					bayer[width + 2] + bayer[width * 2 + 1] +
					2) >> 2;
				*bgr++ = bayer[width + 1];
				*bgr++ = t1;
				*bgr++ = t0;

				t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
				t1 = (bayer[width + 1] + bayer[width + 3] +
					1) >> 1;
				*bgr++ = t1;
				*bgr++ = bayer[width + 2];
				*bgr++ = t0;
			}
		}

		if (bayer < bayerEnd)
		{
			/* write second to last pixel */
			t0 = (bayer[0] + bayer[2] + bayer[width * 2] +
				bayer[width * 2 + 2] + 2) >> 2;
			t1 = (bayer[1] + bayer[width] +
				bayer[width + 2] + bayer[width * 2 + 1] +
				2) >> 2;
			if (blue_line)
			{
				*bgr++ = t0;
				*bgr++ = t1;
				*bgr++ = bayer[width + 1];
			}
			else
			{
				*bgr++ = bayer[width + 1];
				*bgr++ = t1;
				*bgr++ = t0;
			}
			/* write last pixel */
			t0 = (bayer[2] + bayer[width * 2 + 2] + 1) >> 1;
			if (blue_line)
			{
				*bgr++ = t0;
				*bgr++ = bayer[width + 2];
				*bgr++ = bayer[width + 1];
			}
			else
			{
				*bgr++ = bayer[width + 1];
				*bgr++ = bayer[width + 2];
				*bgr++ = t0;
			}
			bayer++;
		}
		else
		{
			/* write last pixel */
			t0 = (bayer[0] + bayer[width * 2] + 1) >> 1;
			t1 = (bayer[1] + bayer[width * 2 + 1] + bayer[width] + 1) / 3;
			if (blue_line)
			{
				*bgr++ = t0;
				*bgr++ = t1;
				*bgr++ = bayer[width + 1];
			}
			else
			{
				*bgr++ = bayer[width + 1];
				*bgr++ = t1;
				*bgr++ = t0;
			}
		}

		/* skip 2 border pixels */
		bayer += 2;

		blue_line = !blue_line;
		start_with_green = !start_with_green;
	}

	/* render the last line */
	convert_border_bayer_line_to_bgr24(bayer + width, bayer, bgr, width,
		!start_with_green, !blue_line);
}

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
void bayer_to_rgb24(uint8_t *pBay, uint8_t *pRGB24, int width, int height, int pix_order)
{
	switch (pix_order)
	{
		//conversion functions are build for bgr, by switching b and r lines we get rgb
		case 0: /* gbgbgb... | rgrgrg... (V4L2_PIX_FMT_SGBRG8)*/
			bayer_to_rgbbgr24(pBay, pRGB24, width, height, TRUE, FALSE);
			break;

		case 1: /* grgrgr... | bgbgbg... (V4L2_PIX_FMT_SGRBG8)*/
			bayer_to_rgbbgr24(pBay, pRGB24, width, height, TRUE, TRUE);
			break;

		case 2: /* bgbgbg... | grgrgr... (V4L2_PIX_FMT_SBGGR8)*/
			bayer_to_rgbbgr24(pBay, pRGB24, width, height, FALSE, FALSE);
			break;

		case 3: /* rgrgrg... ! gbgbgb... (V4L2_PIX_FMT_SRGGB8)*/
			bayer_to_rgbbgr24(pBay, pRGB24, width, height, FALSE, TRUE);
			break;

		default: /* default is 0*/
			bayer_to_rgbbgr24(pBay, pRGB24, width, height, TRUE, FALSE);
			break;
	}
}

/*------------------ YU12 ----------------------*/

/*
 *convert from packed 422 yuv (yuyv) to 420 planar (yu12)
 * args:
 *    out - pointer to output yu12 planar data buffer
 *    in - pointer to input yuyv packed data buffer
 *    width - frame width
 *    height - frame height
 *
 * asserts:
 *    in is not null
 *    out is not null
 *
 * returns: none
 */
void yuyv_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);

	int w = 0, h = 0;
	int y_sizeline = width;
	//int c_sizeline = width/2;
	
	uint8_t *in1 = in; //first line
	uint8_t *in2 = in1 + (width * 2); //second line in yuyv buffer

	uint8_t *py1 = out; // first line
	uint8_t *py2 = py1 + y_sizeline; //second line
	uint8_t *pu = py1 + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);

	for(h = 0; h < height; h+=2)
	{
		for(w = 0; w < width; w+=2) //yuyv 2 bytes per sample
		{
			//printf("decoding: h:%i w:%i\n", h, w);
			*py1++ = *in1++;
			*py2++ = *in2++;
			*pu++ = ((*in1++) + (*in2++)) /2; //average u samples
			*py1++ = *in1++;
			*py2++ = *in2++;
			*pv++ = ((*in1++) + (*in2++)) /2; //average v samples
		}
		in1 = in + (h * width * 2);
		in2 = in + ((h+1) * width * 2);
		py1 = out + (h * width);
		py2 = out + ((h+1) * width); 
	}

}

/*
 *convert from packed 422 yuv (yvyu) to 420 planar (yu12)
 * args:
 *    out - pointer to output yu12 planar data buffer
 *    in - pointer to input yvyu packed data buffer
 *    width - frame width
 *    height - frame height
 *
 * asserts:
 *    in is not null
 *    out is not null
 *
 * returns: none
 */
void yvyu_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);

	int w = 0, h = 0;
	int y_sizeline = width;
	//int c_sizeline = width/2;
	
	uint8_t *in1 = in; //first line
	uint8_t *in2 = in1 + (width * 2); //second line in yuyv buffer

	uint8_t *py1 = out; // first line
	uint8_t *py2 = py1 + y_sizeline; //second line
	uint8_t *pu = py1 + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);

	for(h = 0; h < height; h+=2)
	{
		for(w = 0; w < width*2; w++) //yuyv 2 bytes per sample
		{
			*py1++ = *in1++;
			*py2++ = *in2++;
			*pv++ = ((*in1++) + (*in2++)) /2; //average v samples
			*py1++ = *in1++;
			*py2++ = *in2++;
			*pu++ = ((*in1++) + (*in2++)) /2; //average u samples
		}
		in1 = in + (h * width * 2);
		in2 = in + ((h+1) * width * 2);
		py1 = out + (h * width);
		py2 = out + ((h+1) * width); 
	}

}

/*
 *convert from packed 422 yuv (uyvy) to 420 planar (yu12)
 * args:
 *    out - pointer to output yu12 planar data buffer
 *    in - pointer to input uyvy packed data buffer
 *    width - frame width
 *    height - frame height
 *
 * asserts:
 *    in is not null
 *    out is not null
 *
 * returns: none
 */
void uyvy_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);

	int w = 0, h = 0;
	int y_sizeline = width;
	//int c_sizeline = width/2;
	
	uint8_t *in1 = in; //first line
	uint8_t *in2 = in1 + (width * 2); //second line in yuyv buffer

	uint8_t *py1 = out; // first line
	uint8_t *py2 = py1 + y_sizeline; //second line
	uint8_t *pu = py1 + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);

	for(h = 0; h < height; h+=2)
	{
		for(w = 0; w < width; w+=2) //yuyv 2 bytes per sample
		{
			*pu++ = ((*in1++) + (*in2++)) /2; //average u samples
			*py1++ = *in1++;
			*py2++ = *in2++;
			*pv++ = ((*in1++) + (*in2++)) /2; //average v samples
			*py1++ = *in1++;
			*py2++ = *in2++;
		}
		in1 = in + (h * width * 2);
		in2 = in + ((h+1) * width * 2);
		py1 = out + (h * width);
		py2 = out + ((h+1) * width); 
	}

}

/*
 *convert from 422 planar yuv to 420 planar (yu12)
 * args:
 *    out - pointer to output yu12 planar data buffer
 *    in - pointer to input 422 planar data buffer
 *    width - frame width
 *    height - frame height
 *
 * asserts:
 *    in is not null
 *    out is not null
 *
 * returns: none
 */
void yuv422p_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);

    /*copy y data*/
    memcpy(out, in, width*height);

	int w = 0, h = 0;
	int c_sizeline = width/2;
	
	uint8_t *pu = out + (width * height);
    uint8_t *inu1 = in + (width * height);
    uint8_t *inu2 = inu1 + (width/2);

	uint8_t *pv = pu + ((width * height) / 4);
    uint8_t *inv1 = inu1 + ((width * height) / 2);
    uint8_t *inv2 = inv1 + (width / 2);

	for(h = 0; h < height; h+=2)
	{
		for(w = 0; w < width/2; w++) 
		{
			*pu++ = ((*inu1++) + (*inu2++)) /2; //average u sample
			*pv++ = ((*inv1++) + (*inv2++)) /2; //average v samples
		}
        inu1 = in + (width * height) + (h * c_sizeline);
		inu2 = in + (width * height) + ((h+1) * c_sizeline);
		inv1 = inu1 + ((width * height) / 2);
		inv2 = inv1 + (width / 2);
	}

}

/*
 * convert yyuv (packed) to yuv420 planar (yu12)
 * args:
 *    out: pointer to output buffer (yu12)
 *    in: pointer to input buffer containing yyuv packed data frame
 *    width: picture width
 *    height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void yyuv_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);

	int w = 0, h = 0;
	int y_sizeline = width;
	//int c_sizeline = width/2;
	
	uint8_t *in1 = in; //first line
	uint8_t *in2 = in1 + (width * 2); //second line in yyuv buffer

	uint8_t *py1 = out; // first line
	uint8_t *py2 = py1 + y_sizeline; //second line
	uint8_t *pu = py1 + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);

	for(h = 0; h < height; h+=2)
	{
		for(w = 0; w < width; w+=2) //yyuv 2 bytes per sample
		{
			*py1++ = *in1++;
			*py1++ = *in1++;
			*py2++ = *in2++;
			*py2++ = *in2++;
			*pu++ = ((*in1++) + (*in2++)) /2; //average v samples
			*pv++ = ((*in1++) + (*in2++)) /2; //average u samples
		}
		in1 = in + (h * width * 2);
		in2 = in + ((h+1) * width * 2);
		py1 = out + (h * width);
		py2 = out + ((h+1) * width); 
	}

}

/*
 *convert from 420 planar (yv12) to 420 planar (yu12)
 * args:
 *    out - pointer to output yu12 planar data buffer
 *    in - pointer to input yv12 planar data buffer
 *    width - frame width
 *    height - frame height
 *
 * asserts:
 *    in is not null
 *    out is not null
 *
 * returns: none
 */
void yv12_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);

    /*copy y data*/
    memcpy(out, in, width*height);
	/*copy u data*/
	memcpy(out+(width*height), in+((width * height * 5) / 4), width * height / 4);
	/*copy v data*/
	memcpy(out+((width * height * 5) / 4), in+(width * height), width * height / 4);
}

/*
 * convert nv12 planar (uv interleaved) to yuv420 planar (yu12)
 * args:
 *    out: pointer to output buffer (yu12)
 *    in: pointer to input buffer containing nv12 planar data frame
 *    width: picture width
 *    height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void nv12_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);

	/*copy y data*/
    memcpy(out, in, width*height);
	
	uint8_t *puv = in + (width * height);
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);

	/*uv plane*/
	int i = 0;
	for(i=0; i< width * height /2; i+=2)
	{
		*pu++ = *puv++;
		*pv++ = *puv++;
	}
}

/*
 * convert nv21 planar (vu interleaved) to yuv420 planar (yu12)
 * args:
 *    out: pointer to output buffer (yu12)
 *    in: pointer to input buffer containing nv21 planar data frame
 *    width: picture width
 *    height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void nv21_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);

	/*copy y data*/
    memcpy(out, in, width*height);
	
	uint8_t *puv = in + (width * height);
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);

	/*uv plane*/
	int i = 0;
	for(i=0; i< width * height /2; i+=2)
	{
		*pv++ = *puv++;
		*pu++ = *puv++;
	}

}

/*
 * convert yuv 422 planar (uv interleaved) (nv16) to yuv420 planar (yu12)
 * args:
 *   out: pointer to output buffer (yu12)
 *   in: pointer to input buffer containing yuv422 (nv16) planar data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void nv16_to_yu12 (uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);

	/*copy y data*/
    memcpy(out, in, width*height);
	
	/*uv plane*/
	uint8_t *puv1 = in + (width * height); //first line
	uint8_t *puv2 = puv1 + width; //second line
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);

	int h = 0;
	int w = 0;
	for(h=0; h < height; h+=2)
	{
		for(w=0; w < width; w+=2)
		{
			*pu++ = ((*puv1++) + (*puv2++)) / 2; //average
			*pv++ = ((*puv1++) + (*puv2++)) / 2; //average
		}
		puv1 = in + (width + height) + (h * width);
		puv2 = puv1 + width;
	}
}

/*
 * convert yuv 422 planar (vu interleaved) (nv61) to yuv420 planar (yu12)
 * args:
 *   out: pointer to output buffer (yu12)
 *   in: pointer to input buffer containing yuv422 (nv61) planar data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void nv61_to_yu12 (uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);

	/*copy y data*/
    memcpy(out, in, width*height);
	
	/*uv plane*/
	uint8_t *puv1 = in + (width * height); //first line
	uint8_t *puv2 = puv1 + width; //second line
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);

	int h = 0;
	int w = 0;
	for(h=0; h < height; h+=2)
	{
		for(w=0; w < width; w+=2)
		{
			*pv++ = ((*puv1++) + (*puv2++)) / 2; //average
			*pu++ = ((*puv1++) + (*puv2++)) / 2; //average
		}
		puv1 = in + (width + height) + (h * width);
		puv2 = puv1 + width;
	}
}

/*
 * Unpack buffer of (vw bit) data into padded 16bit buffer.
 * args:
 *    raw - pointer to input raw packed data buffer
 *    unpacked - pointer to unpacked output data buffer
 *    vw - vw bit
 *    unpacked_len - length
 *
 * asserts:
 *    none
 *
 * returns: none
 */
static inline void convert_packed_to_16bit(uint8_t *raw, uint16_t *unpacked, int vw, int unpacked_len)
{
	int mask = (1 << vw) - 1;
	uint32_t buffer = 0;
	int bitsIn = 0;
	while (unpacked_len--) {
		while (bitsIn < vw) {
			buffer = (buffer << 8) | *(raw++);
			bitsIn += 8;
		}
		bitsIn -= vw;
		*(unpacked++) = (buffer >> bitsIn) & mask;
	}
}

/*
 * convert y10b (bit-packed array greyscale format) to yu12
 * args:
 *   out: pointer to output buffer (yu12)
 *   in: pointer to input buffer containing y10b (bit-packed array) data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void y10b_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);
	
	uint16_t *unpacked_buffer = NULL;
	uint16_t *ptmp;
	uint8_t *py = out;
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);
	
	int h = 0;

	unpacked_buffer = malloc(width * height * sizeof(uint16_t));
	
	if (unpacked_buffer == NULL)
	{
		fprintf(stderr, "V4L2_CORE: FATAL memory allocation failure (y10b_to_yu12): %s\n", strerror(errno));
		exit(-1);
	}
	
	convert_packed_to_16bit(in, unpacked_buffer, 10, width * height);

	ptmp = unpacked_buffer;

	for (h = 0; h < height; h++)
	{
		int w=0;
		for (w = 0; w < width; w++)
		{
			/* Y */
			*py++ = (uint8_t) ((*ptmp++ & 0x3FF) >> 2);
		}
	}
	
	for(h=0; h < (width * height / 4); h++)
	{
		/* U */
		*pu++ = 0x80;
		/* V */
		*pv++ = 0x80;
	}


	free(unpacked_buffer);
}

/*
 * convert yuv 411 packed (y41p) to planar yuv 420 (yu12)
 * args:
 *    out: pointer to output buffer (yu12)
 *    in: pointer to input buffer containing y41p data frame
 *    width: picture width
 *    height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void y41p_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);
	
	uint8_t *py1 = out;
	uint8_t *py2 = out + width;
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);
	
	int h=0;
	int linesize = width * 3 /2;

	
	for(h = 0; h < height; h += 2)
	{
		py1 = out + (h * width); // first line
		py2 = out + ((h + 1) * width); // second line
		int offset1 = linesize * h; //line 1
		int offset2 = linesize * (h + 1); //line 2
		int w = 0;
		for(w = 0; w < linesize; w += 12)
		{
			/* y first line */
			*py1++ = in[w + 1 + offset1]; //Y00
			*py1++ = in[w + 3 + offset1]; //Y01
			*py1++ = in[w + 5 + offset1]; //Y02
			*py1++ = in[w + 7 + offset1]; //Y03
			*py1++ = in[w + 8 + offset1]; //Y04
			*py1++ = in[w + 9 + offset1]; //Y05
			*py1++ = in[w + 10 + offset1]; //Y06
			*py1++ = in[w + 11 + offset1]; //Y07
			/* y second line */
			*py2++ = in[w + 1 + offset2]; //Y10
			*py2++ = in[w + 3 + offset2]; //Y11
			*py2++ = in[w + 5 + offset2]; //Y12
			*py2++ = in[w + 7 + offset2]; //Y13
			*py2++ = in[w + 8 + offset2]; //Y14
			*py2++ = in[w + 9 + offset2]; //Y15
			*py2++ = in[w + 10 + offset2]; //Y16
			*py2++ = in[w + 11 + offset2]; //Y17 
			
			/*U0 and U1 average first and second lines*/
			*pu++ = (in[w + offset1] + in[w + offset2]) / 2;         //U00 + U10 /2
			*pu++ = (in[w + offset1] + in[w + offset2]) / 2;         //U00 + U10 /2
			/*U2 and U3 average first and second lines*/
			*pu++ = (in[w + 4 + offset1] + in[w + 4 + offset2]) / 2; //U01 + U11 /2
			*pu++ = (in[w + 4 + offset1] + in[w + 4 + offset2]) / 2; //U01 + U11 /2
			/*V0 and V1 average first and second lines*/
			*pv++ = (in[w + 2 + offset1] + in[w + 2 + offset2]) / 2; //V00 + V10 /2
			*pv++ = (in[w + 2 + offset1] + in[w + 2 + offset2]) / 2; //V00 + V10 /2
			/*V2 and V3 average first and second lines*/
			*pv++ = (in[w + 6 + offset1] + in[w + 6 + offset2]) / 2; //V01 + V11 /2
			*pv++ = (in[w + 6 + offset1] + in[w + 6 + offset2]) / 2; //V01 + V11 /2	
		}
	}
}

/*
 * convert yuv mono (grey) to yuv 420 planar (yu12)
 * args:
 *   out: pointer to output buffer (yu12)
 *   in: pointer to input buffer containing grey (y only) data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *   out is not null
 *   in is not null
 *
 * returns: none
 */
void grey_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);
	
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);
	
	int h=0;

	/* Y */
	memcpy(out, in, width * height);
	
	/* U and V */
	for (h=0; h < (width * height / 4); h++)
	{
		*pu++ = 0x80;
		*pv++ = 0x80;
	}
}

/*
 * convert y16 (16 bit greyscale format) to yu12
 * args:
 *   out: pointer to output buffer (yu12)
 *   in: pointer to input buffer containing y16 (16 bit greyscale) data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void y16_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);
	
	uint16_t *ptmp = (uint16_t *) in;
	uint8_t *py = out;
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);
	
	int h = 0;

	for (h = 0; h < height; h++)
	{
		int w=0;
		for (w = 0; w < width; w++)
		{
			/* Y */
			*py++ = (uint8_t) ((*ptmp++ & 0xFF00) >> 8);
		}
	}
	
	for(h=0; h < (width * height / 4); h++)
	{
		/* U */
		*pu++ = 0x80;
		/* V */
		*pv++ = 0x80;
	}
	
}

/*
 * convert SPCA501 (s501) to yuv 420 planar (yu12)
 *   s501  |Y0..width..Y0|U..width/2..U|Y1..width..Y1|V..width/2..V|
 *   signed values (-128;+127) must be converted to unsigned (0; 255)
 * args:
 *   out: pointer to output buffer (yu12)
 *   in: pointer to input buffer containing s501 data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void s501_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);
	
	/*assertions*/
	assert(in);
	assert(out);
	
	int h = 0;

	int8_t *pin = (int8_t *) in;
	
	uint8_t *py = out;
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height ) / 4);

	for (h = 0; h < height; h += 2 )
	{
		int w = 0;
		/* Y */
		for (w = 0; w < width; w++)
		{
			*py++ = (uint8_t) 0x80 + *pin++;
		}
		
		/* U */
		for (w = 0; w < width /2; w++)
		{
			*pu++ = (uint8_t) 0x80 + *pin++;
		}
		
		/* Y */
		for (w = 0; w < width; w++)
		{
			*py++ = (uint8_t) 0x80 + *pin++;
		}
		
		/* V */
		for (w = 0; w < width /2; w++)
		{
			*pv++ = (uint8_t) 0x80 + *pin++;
		}
	}
}

/*
 * convert SPCA505 (s505) to yuv 420 planar (yu12)
 *   s505  |Y0..width..Y0|Y1..width..Y1|U..width/2..U|V..width/2..V|
 *   signed values (-128;+127) must be converted to unsigned (0; 255)
 * args:
 *   out: pointer to output buffer (yu12)
 *   in: pointer to input buffer containing s501 data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void s505_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);
	
	int h = 0;

	int8_t *pin = (int8_t *) in;
	
	uint8_t *py = out;
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height ) / 4);

	for (h = 0; h < height; h += 2 )
	{
		int w = 0;
		/* Y */
		for (w = 0; w < width * 2; w++) // 2 lines
		{
			*py++ = (uint8_t) 0x80 + *pin++;
		}
		
		/* U */
		for (w = 0; w < width /2; w++)
		{
			*pu++ = (uint8_t) 0x80 + *pin++;
		}
		
		/* V */
		for (w = 0; w < width /2; w++)
		{
			*pv++ = (uint8_t) 0x80 + *pin++;
		}
	}
}

/*
 * convert SPCA508 (s508) to yuv 420 planar (yu12)
 *   s508  |Y0..width..Y0|U..width/2..U|V..width/2..V|Y1..width..Y1|
 *   signed values (-128;+127) must be converted to unsigned (0; 255)
 * args:
 *   out: pointer to output buffer (yu12)
 *   in: pointer to input buffer containing s501 data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void s508_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(in);
	assert(out);
	
	int h = 0;

	int8_t *pin = (int8_t *) in;
	
	uint8_t *py = out;
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height ) / 4);

	for (h = 0; h < height; h += 2 )
	{
		int w = 0;
		/* Y */
		for (w = 0; w < width; w++)
		{
			*py++ = (uint8_t) 0x80 + *pin++;
		}
		
		/* U */
		for (w = 0; w < width /2; w++)
		{
			*pu++ = (uint8_t) 0x80 + *pin++;
		}
		
		/* V */
		for (w = 0; w < width /2; w++)
		{
			*pv++ = (uint8_t) 0x80 + *pin++;
		}
		
		/* Y */
		for (w = 0; w < width; w++)
		{
			*py++ = (uint8_t) 0x80 + *pin++;
		}
	}
}

/*
 * convert rgb24 to yu12
 * args:
 *   out: pointer to output buffer containing yu12 data
 *   in: pointer to input buffer containing rgb24 data
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *   out is not null
 *   in is not null
 *
 * returns: none
 */
void rgb24_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(out);
	assert(in);
	
	uint8_t *py = out;
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);
	
	uint8_t *in1 = in; //first line
	uint8_t *in2 = in + (width * 3); //second line
	
	int i=0;
	for(i = 0; i < (width * height * 3); i += 3)
	{
		/* y */
		*py++ =CLIP(0.299 * (in1[i] - 128) + 0.587 * (in1[i+1] - 128) + 0.114 * (in1[i+2] - 128) + 128);
	}
	
	int h = 0;
	for(h = 0; h < height; h += 2)
	{
		in1 = in + (h * width * 3);
		in2 = in1 + (width * 3);
		
		for(i = 0; i < (width * 3); i += 6)
		{
			/* u v */
			uint8_t u1 = CLIP(((- 0.147 * (in1[i] - 128) - 0.289 * (in1[i+1] - 128) + 0.436 * (in1[i+2] - 128) + 128) +
				(- 0.147 * (in1[i+3] - 128) - 0.289 * (in1[i+4] - 128) + 0.436 * (in1[i+5] - 128) + 128))/2);
			uint8_t v1 =CLIP(((0.615 * (in1[i] - 128) - 0.515 * (in1[i+1] - 128) - 0.100 * (in1[i+2] - 128) + 128) +
				(0.615 * (in1[i+3] - 128) - 0.515 * (in1[i+4] - 128) - 0.100 * (in1[i+5] - 128) + 128))/2);
				
			uint8_t u2 = CLIP(((- 0.147 * (in2[i] - 128) - 0.289 * (in2[i+1] - 128) + 0.436 * (in2[i+2] - 128) + 128) +
				(- 0.147 * (in2[i+3] - 128) - 0.289 * (in2[i+4] - 128) + 0.436 * (in2[i+5] - 128) + 128))/2);
			uint8_t v2 =CLIP(((0.615 * (in2[i] - 128) - 0.515 * (in2[i+1] - 128) - 0.100 * (in2[i+2] - 128) + 128) +
				(0.615 * (in2[i+3] - 128) - 0.515 * (in2[i+4] - 128) - 0.100 * (in2[i+5] - 128) + 128))/2);	
				
			*pu++ = (u1 + u2) / 2;
			*pv++ = (v1 + v2) / 2;
		}
	}
}

/*
 * convert bgr24 to yu12
 * args:
 *   out: pointer to output buffer containing yu12 data
 *   in: pointer to input buffer containing bgr24 data
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *   out is not null
 *   in is not null
 *
 * returns: none
 */
void bgr24_to_yu12(uint8_t *out, uint8_t *in, int width, int height)
{	
	/*assertions*/
	assert(out);
	assert(in);
	
	uint8_t *py = out;
	uint8_t *pu = out + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);
	
	uint8_t *in1 = in; //first line
	uint8_t *in2 = in + (width * 3); //second line
	
	int i = 0;
	for(i = 0; i < (width * height * 3); i += 3)
	{
		/* y */
		*py++ =CLIP(0.299 * (in1[i+2] - 128) + 0.587 * (in1[i+1] - 128) + 0.114 * (in1[i] - 128) + 128);
	}
	
	int h = 0;
	for(h = 0; h < height; h += 2)
	{
		in1 = in + (h * width * 3);
		in2 = in1 + (width * 3);
		
		for(i = 0; i < (width * 3); i += 6)
		{
			/* u */
			uint8_t u1 = CLIP(((- 0.147 * (in1[i+2] - 128) - 0.289 * (in1[i+1] - 128) + 0.436 * (in1[i] - 128) + 128) +
				(- 0.147 * (in1[i+5] - 128) - 0.289 * (in1[i+4] - 128) + 0.436 * (in1[i+3] - 128) + 128))/2);
			uint8_t u2 = CLIP(((- 0.147 * (in2[i+2] - 128) - 0.289 * (in2[i+1] - 128) + 0.436 * (in2[i] - 128) + 128) +
				(- 0.147 * (in2[i+5] - 128) - 0.289 * (in2[i+4] - 128) + 0.436 * (in2[i+3] - 128) + 128))/2);
	
			/* v*/
			uint8_t v1 =CLIP(((0.615 * (in1[i+2] - 128) - 0.515 * (in1[i+1] - 128) - 0.100 * (in1[i] - 128) + 128) +
				(0.615 * (in1[i+5] - 128) - 0.515 * (in1[i+4] - 128) - 0.100 * (in1[i+3] - 128) + 128))/2);
			uint8_t v2 =CLIP(((0.615 * (in2[i+2] - 128) - 0.515 * (in2[i+1] - 128) - 0.100 * (in2[i] - 128) + 128) +
				(0.615 * (in2[i+5] - 128) - 0.515 * (in2[i+4] - 128) - 0.100 * (in2[i+3] - 128) + 128))/2);
			
			*pu++ = (u1 + u2) / 2;
			*pv++ = (v1 + v2) / 2;
		}
	}
	
	
}

/*
 * yu12 to rgb24
 * args:
 *    out - pointer to output rgb data buffer
 *    in - pointer to input yu12 data buffer
 *    width - buffer width (in pixels)
 *    height - buffer height (in pixels)
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void yu12_to_rgb24 (uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(out);
	assert(in);
	
	uint8_t *py1 = in; //line 1
	uint8_t *py2 = py1 + width; //line 2
	uint8_t *pu = in + (width * height);
	uint8_t *pv = pu + ((width * height) / 4);
	
	uint8_t *pout1 = out; //first line
	uint8_t *pout2 = out + (width * 3); //second line
	
	int h=0, w=0;
	
	for(h=0; h < height; h+=2) //every two lines
	{
		py1 = in + (h * width);
		py2 = py1 + width;
		
		pout1 = out + (h * width * 3);
		pout2 = pout1 + (width * 3);
		
		for(w=0; w<width; w+=2) //every 2 pixels
		{
			/* standart: r = y0 + 1.402 (v-128) */
			/* logitech: r = y0 + 1.370705 (v-128) */
			*pout1++=CLIP(*py1 + 1.402 * (*pv-128));
			*pout2++=CLIP(*py2 + 1.402 * (*pv-128));	
			/* standart: g = y0 - 0.34414 (u-128) - 0.71414 (v-128)*/
			/* logitech: g = y0 - 0.337633 (u-128)- 0.698001 (v-128)*/
			*pout1++=CLIP(*py1 - 0.34414 * (*pu-128) -0.71414*(*pv-128));
			*pout2++=CLIP(*py2 - 0.34414 * (*pu-128) -0.71414*(*pv-128));
			/* standart: b = y0 + 1.772 (u-128) */
			/* logitech: b = y0 + 1.732446 (u-128) */
			*pout1++=CLIP(*py1 + 1.772 *( *pu-128));
			*pout2++=CLIP(*py2 + 1.772 *( *pu-128));
			
			py1++;
			py2++;
			
			/* standart: r1 =y1 + 1.402 (v-128) */
			/* logitech: r1 = y1 + 1.370705 (v-128) */
			*pout1++=CLIP(*py1 + 1.402 * (*pv-128));
			*pout2++=CLIP(*py2 + 1.402 * (*pv-128));
			/* standart: g1 = y1 - 0.34414 (u-128) - 0.71414 (v-128)*/
			/* logitech: g1 = y1 - 0.337633 (u-128)- 0.698001 (v-128)*/
			*pout1++=CLIP(*py1 - 0.34414 * (*pu-128) -0.71414 * (*pv-128));
			*pout2++=CLIP(*py2 - 0.34414 * (*pu-128) -0.71414 * (*pv-128));
			/* standart: b1 = y1 + 1.772 (u-128) */
			/* logitech: b1 = y1 + 1.732446 (u-128) */
			*pout1++=CLIP(*py1 + 1.772 * (*pu-128));
			*pout2++=CLIP(*py2 + 1.772 * (*pu-128));
			
			py1++;
			py2++;
			pu++;
			pv++;
		}
	}
}

/*
 * FIXME:  yu12 to bgr24 with lines upsidedown
 *   used for bitmap files (DIB24)
 * args:
 *    out - pointer to output bgr data buffer
 *    in - pointer to input yu12 data buffer
 *    width - buffer width (in pixels)
 *    height - buffer height (in pixels)
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void yu12_to_dib24 (uint8_t *out, uint8_t *in, int width, int height)
{
	/*assertions*/
	assert(out);
	assert(in);
	
	uint8_t *py1 = in + (height * width) - width; //begin of last line
	uint8_t *py2 = py1 - width; //last line -1
	
	uint8_t *pu = in + ((width * height * 5) / 4) - (width/2); //begin of last line
	uint8_t *pv = pu + ((width * height) / 4); //begin of last line
	
	uint8_t *pout1 = out; //first line
	uint8_t *pout2 = pout1 + (width * 3); //second line
	
	int h=0, w=0;
	int uvline = height/2;
	
	for(h=height; h >0 ; h-=2) //every two lines
	{
		uvline--; //begin of uv line
		
		py1 = in + ((h-1) * width);
		py2 = py1 - width;
		
		pu = in + (width * height) + ((uvline * width)/2);
		pv = pu + ((width * height) / 4);
		
		pout1 = out + ((height-h) * width * 3);
		pout2 = pout1 + (width * 3);
		
		for(w=0; w<width; w+=2) //every 2 pixels
		{
			/* standart: b = y0 + 1.772 (u-128) */
			/* logitech: b = y0 + 1.732446 (u-128) */
			*pout1++=CLIP(*py1 + 1.772 *( *pu-128));
			*pout2++=CLIP(*py2 + 1.772 *( *pu-128));	
			/* standart: g = y0 - 0.34414 (u-128) - 0.71414 (v-128)*/
			/* logitech: g = y0 - 0.337633 (u-128)- 0.698001 (v-128)*/
			*pout1++=CLIP(*py1 - 0.34414 * (*pu-128) -0.71414*(*pv-128));
			*pout2++=CLIP(*py2 - 0.34414 * (*pu-128) -0.71414*(*pv-128));
			/* standart: r = y0 + 1.402 (v-128) */
			/* logitech: r = y0 + 1.370705 (v-128) */
			*pout1++=CLIP(*py1 + 1.402 * (*pv-128));
			*pout2++=CLIP(*py2 + 1.402 * (*pv-128));
			
			py1++;
			py2++;
			
			/* standart: b = y0 + 1.772 (u-128) */
			/* logitech: b = y0 + 1.732446 (u-128) */
			*pout1++=CLIP(*py1 + 1.772 *( *pu-128));
			*pout2++=CLIP(*py2 + 1.772 *( *pu-128));	
			/* standart: g = y0 - 0.34414 (u-128) - 0.71414 (v-128)*/
			/* logitech: g = y0 - 0.337633 (u-128)- 0.698001 (v-128)*/
			*pout1++=CLIP(*py1 - 0.34414 * (*pu-128) -0.71414*(*pv-128));
			*pout2++=CLIP(*py2 - 0.34414 * (*pu-128) -0.71414*(*pv-128));
			/* standart: r = y0 + 1.402 (v-128) */
			/* logitech: r = y0 + 1.370705 (v-128) */
			*pout1++=CLIP(*py1 + 1.402 * (*pv-128));
			*pout2++=CLIP(*py2 + 1.402 * (*pv-128));
			
			py1++;
			py2++;
			pu++;
			pv++;
		}
	}
}

/*
 * convert yuv 420 planar (yu12) to yuv 422
 * args:
 *    out- pointer to output buffer (yuyv)
 *    in- pointer to input buffer (yuv420 planar data frame (yu12))
 *    width- picture width
 *    height- picture height
 *
 * asserts:
 *    out is not null
 *    in is not null
 *
 * returns: none
 */
void yu12_to_yuyv (uint8_t *out, uint8_t *in, int width, int height)
{
	uint8_t *py;
	uint8_t *pu;
	uint8_t *pv;

	int linesize = width * 2;
	int uvlinesize = width / 2;

	py=in;
	pu=py+(width*height);
	pv=pu+(width*height/4);

	int h=0;
	int huv=0;

	for(h=0;h<height;h+=2)
	{
		int wy = 0;
		int wuv = 0;
		int offset = h * linesize;
		int offset1 = (h + 1) * linesize;
		int offsety = h * width;
		int offsety1 = (h + 1) * width;
		int offsetuv = huv * uvlinesize;
		int w = 0;

		for(w=0;w<linesize;w+=4)
		{
			/*y00*/
			out[w + offset] = py[wy + offsety];
			/*u0*/
			out[(w + 1) + offset] = pu[wuv + offsetuv];
			/*y01*/
			out[(w + 2) + offset] = py[(wy + 1) + offsety];
			/*v0*/
			out[(w + 3) + offset] = pv[wuv + offsetuv];

			/*y10*/
			out[w + offset1] = py[wy + offsety1];
			/*u0*/
			out[(w + 1) + offset1] = pu[wuv + offsetuv];
			/*y11*/
			out[(w + 2) + offset1] = py[(wy + 1) + offsety1];
			/*v0*/
			out[(w + 3) + offset1] = pv[wuv + offsetuv];

			wuv++;
			wy+=2;
		}
		huv++;
	}
}

/*------------------- YUYV --------------------*/

/*
 * regular yuv (YUYV) to rgb24
 * args:
 *    pyuv - pointer to input yuyv data buffer
 *    prgb - pointer to converted output rgb data buffer
 *    width - buffer width (in pixels)
 *    height - buffer height (in pixels)
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void yuyv2rgb (uint8_t *pyuv, uint8_t *prgb, int width, int height)
{
	int l=0;
	int SizeYUV=height * width * 2; /* 2 bytes per pixel*/
	for(l=0;l<SizeYUV;l=l+4)
	{	/*iterate every 4 bytes*/
		/* standart: r = y0 + 1.402 (v-128) */
		/* logitech: r = y0 + 1.370705 (v-128) */
		*prgb++=CLIP(pyuv[l] + 1.402 * (pyuv[l+3]-128));
		/* standart: g = y0 - 0.34414 (u-128) - 0.71414 (v-128)*/
		/* logitech: g = y0 - 0.337633 (u-128)- 0.698001 (v-128)*/
		*prgb++=CLIP(pyuv[l] - 0.34414 * (pyuv[l+1]-128) -0.71414*(pyuv[l+3]-128));
		/* standart: b = y0 + 1.772 (u-128) */
		/* logitech: b = y0 + 1.732446 (u-128) */
		*prgb++=CLIP(pyuv[l] + 1.772 *( pyuv[l+1]-128));
		/* standart: r1 =y1 + 1.402 (v-128) */
		/* logitech: r1 = y1 + 1.370705 (v-128) */
		*prgb++=CLIP(pyuv[l+2] + 1.402 * (pyuv[l+3]-128));
		/* standart: g1 = y1 - 0.34414 (u-128) - 0.71414 (v-128)*/
		/* logitech: g1 = y1 - 0.337633 (u-128)- 0.698001 (v-128)*/
		*prgb++=CLIP(pyuv[l+2] - 0.34414 * (pyuv[l+1]-128) -0.71414 * (pyuv[l+3]-128));
		/* standart: b1 = y1 + 1.772 (u-128) */
		/* logitech: b1 = y1 + 1.732446 (u-128) */
		*prgb++=CLIP(pyuv[l+2] + 1.772*(pyuv[l+1]-128));
	}
}

/*
 * used for rgb video (fourcc="RGB ")
 *   lines are on correct order
 * args:
 *    pyuv - pointer to input yuyv data buffer
 *    prgb - pointer to converted output bgr data buffer
 *    width - buffer width (in pixels)
 *    height - buffer height (in pixels)
 *
 * asserts:
 *    none
 *
 * returns: none
 */
//void yuyv2bgr1 (uint8_t *pyuv, uint8_t *pbgr, int width, int height)
//{
//
//	int l=0;
//	int SizeYUV=height * width * 2; /* 2 bytes per pixel*/
//	for(l=0;l<SizeYUV;l=l+4)
//	{	/*iterate every 4 bytes*/
//		/* standart: b = y0 + 1.772 (u-128) */
//		/* logitech: b = y0 + 1.732446 (u-128) */
//		*pbgr++=CLIP(pyuv[l] + 1.772 *( pyuv[l+1]-128));
//		/* standart: g = y0 - 0.34414 (u-128) - 0.71414 (v-128)*/
//		/* logitech: g = y0 - 0.337633 (u-128)- 0.698001 (v-128)*/
//		*pbgr++=CLIP(pyuv[l] - 0.34414 * (pyuv[l+1]-128) -0.71414*(pyuv[l+3]-128));
//		/* standart: r = y0 + 1.402 (v-128) */
//		/* logitech: r = y0 + 1.370705 (v-128) */
//		*pbgr++=CLIP(pyuv[l] + 1.402 * (pyuv[l+3]-128));
//		/* standart: b1 = y1 + 1.772 (u-128) */
//		/* logitech: b1 = y1 + 1.732446 (u-128) */
//		*pbgr++=CLIP(pyuv[l+2] + 1.772*(pyuv[l+1]-128));
//		/* standart: g1 = y1 - 0.34414 (u-128) - 0.71414 (v-128)*/
//		/* logitech: g1 = y1 - 0.337633 (u-128)- 0.698001 (v-128)*/
//		*pbgr++=CLIP(pyuv[l+2] - 0.34414 * (pyuv[l+1]-128) -0.71414 * (pyuv[l+3]-128));
//		/* standart: r1 =y1 + 1.402 (v-128) */
//		/* logitech: r1 = y1 + 1.370705 (v-128) */
//		*pbgr++=CLIP(pyuv[l+2] + 1.402 * (pyuv[l+3]-128));
//	}
//}

/*
 * yuv (YUYV) to bgr with lines upsidedown
 *   used for bitmap files (DIB24)
 * args:
 *    pyuv - pointer to input yuyv data buffer
 *    prgb - pointer to converted output bgr data buffer
 *    width - buffer width (in pixels)
 *    height - buffer height (in pixels)
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void yuyv2bgr (uint8_t *pyuv, uint8_t *pbgr, int width, int height)
{

	int l=0;
	uint8_t *preverse;
	int SizeBGR=height * width * 3; /* 3 bytes per pixel*/
	/* BMP byte order is bgr and the lines start from last to first*/
	preverse=pbgr+SizeBGR;/*start at the end and decrement*/
	for(l=0;l<height;l++)
	{	/*iterate every 1 line*/
		preverse-=width*3;/*put pointer at begin of unprocessed line*/
		int bytesUsed=l*width*2;
		int k = 0;
		for (k=0;k<(width*2);k=k+4)/*iterate every 4 bytes in the line*/
		{
			/* standart: b = y0 + 1.772 (u-128) */
			/* logitech: b = y0 + 1.732446 (u-128) */
			*preverse++=CLIP(pyuv[k+bytesUsed] + 1.772 *( pyuv[k+1+bytesUsed]-128));
			/* standart: g = y0 - 0.34414 (u-128) - 0.71414 (v-128)*/
			/* logitech: g = y0 - 0.337633 (u-128)- 0.698001 (v-128)*/
			*preverse++=CLIP(pyuv[k+bytesUsed] - 0.34414 * (pyuv[k+1+bytesUsed]-128)
				-0.71414*(pyuv[k+3+bytesUsed]-128));
			/* standart: r = y0 + 1.402 (v-128) */
			/* logitech: r = y0 + 1.370705 (v-128) */
			*preverse++=CLIP(pyuv[k+bytesUsed] + 1.402 * (pyuv[k+3+bytesUsed]-128));
			/* standart: b1 = y1 + 1.772 (u-128) */
			/* logitech: b1 = y1 + 1.732446 (u-128) */
			*preverse++=CLIP(pyuv[k+2+bytesUsed] + 1.772*(pyuv[k+1+bytesUsed]-128));
			/* standart: g1 = y1 - 0.34414 (u-128) - 0.71414 (v-128)*/
			/* logitech: g1 = y1 - 0.337633 (u-128)- 0.698001 (v-128)*/
			*preverse++=CLIP(pyuv[k+2+bytesUsed] - 0.34414 * (pyuv[k+1+bytesUsed]-128)
				-0.71414 * (pyuv[k+3+bytesUsed]-128));
			/* standart: r1 =y1 + 1.402 (v-128) */
			/* logitech: r1 = y1 + 1.370705 (v-128) */
			*preverse++=CLIP(pyuv[k+2+bytesUsed] + 1.402 * (pyuv[k+3+bytesUsed]-128));
		}
		preverse-=width*3;/*get it back at the begin of processed line*/
	}
	preverse=NULL;
}

/*
 * convert y10b (bit-packed array greyscale format) to yuyv (packed)
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing y10b (bit-packed array) data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void y10b_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint16_t *unpacked_buffer = NULL;
	uint16_t *ptmp;
	int h = 0;

	unpacked_buffer = malloc(width * height * sizeof(uint16_t));
	
	if (unpacked_buffer == NULL)
	{
		fprintf(stderr, "V4L2_CORE: FATAL memory allocation failure (y10b_to_yuyv): %s\n", strerror(errno));
		exit(-1);
	}
	
	convert_packed_to_16bit(tmpbuffer, unpacked_buffer, 10, width * height);

	ptmp = unpacked_buffer;

	for (h = 0; h < height; h++)
	{
		int w=0;
		for (w = 0; w < width; w += 2)
		{
			/* Y0 */
			*framebuffer++ = (uint8_t) ((ptmp[0] & 0x3FF) >> 2);
			/* U */
			*framebuffer++ = 0x80;
			/* Y1 */
			*framebuffer++ = (uint8_t) ((ptmp[1] & 0x3FF) >> 2);
			/* V */
			*framebuffer++ = 0x80;

			ptmp += 2;
		}
	}

	free(unpacked_buffer);
}

/*
 * convert y16 (grey) to yuyv (packed)
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing y16 (grey) data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *   none
 *
 * returns: none
 */
void y16_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint16_t *ptmp= (uint16_t *) tmpbuffer;

	int h=0;

	for(h=0;h<height;h++)
	{
		int w=0;
		for(w=0;w<width;w+=2)
		{
			/* Y0 */
			*framebuffer++ = (uint8_t) ((ptmp[0] & 0xFF00) >> 8);
			/* U */
			*framebuffer++ = 0x80;
			/* Y1 */
			*framebuffer++ = (uint8_t) ((ptmp[1] & 0xFF00) >> 8);
			/* V */
			*framebuffer++ = 0x80;

			ptmp += 2;
		}
	}
}

/*
 * convert yyuv (packed) to yuyv (packed)
 * args:
 *    framebuffer: pointer to frame buffer (yuyv)
 *    tmpbuffer: pointer to temp buffer containing yyuv packed data frame
 *    width: picture width
 *    height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void yyuv_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint8_t *ptmp=NULL;
	uint8_t *pfmb=NULL;
	ptmp = tmpbuffer;
	pfmb = framebuffer;

	int h=0;

	for(h=0;h<height;h++)
	{
		int w=0;
		for(w=0;w<(width*2);w+=4)
		{
			/* Y0 */
			pfmb[0] = ptmp[0];
			/* U */
			pfmb[1] = ptmp[2];
			/* Y1 */
			pfmb[2] = ptmp[1];
			/* V */
			pfmb[3] = ptmp[3];

			ptmp += 4;
			pfmb += 4;
		}
	}
}

/*
 * convert uyvy (packed) to yuyv (packed)
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing uyvy packed data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void uyvy_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint8_t *ptmp = tmpbuffer;
	uint8_t *pfmb = framebuffer;
	int h=0;

	for(h=0;h<height;h++)
	{
		int w=0;
		for(w=0;w<(width*2);w+=4)
		{
			/* Y0 */
			pfmb[0] = ptmp[1];
			/* U */
			pfmb[1] = ptmp[0];
			/* Y1 */
			pfmb[2] = ptmp[3];
			/* V */
			pfmb[3] = ptmp[2];

			ptmp += 4;
			pfmb += 4;
		}
	}
}

/*
 * convert yvyu (packed) to yuyv (packed)
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing yvyu packed data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void yvyu_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint8_t *ptmp=NULL;
	uint8_t *pfmb=NULL;
	ptmp = tmpbuffer;
	pfmb = framebuffer;

	int h=0;

	for(h=0;h<height;h++)
	{
		int w = 0;
		for(w=0;w<(width*2);w+=4)
		{
			/* Y0 */
			pfmb[0] = ptmp[0];
			/* U */
			pfmb[1] = ptmp[3];
			/* Y1 */
			pfmb[2] = ptmp[2];
			/* V */
			pfmb[3] = ptmp[1];

			ptmp += 4;
			pfmb += 4;
		}
	}
}

/*
 * convert yuv 422 planar (yuv422p) to yuv 422
 * args:
 *    output- pointer to output buffer (yuyv)
 *    input- pointer to input buffer (yuv420 planar data frame)
 *    width- picture width
 *    height- picture height
 *
 * asserts:
 *    input not null
 *
 * returns: none
 */
void yuv422_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint8_t *py;
	uint8_t *pu;
	uint8_t *pv;

	int linesize = width * 2;
	int uvlinesize = width / 2;

	py=tmpbuffer;
	pu=py+(width*height);
	pv=pu+(width*height/2);

	int h=0;

	for(h=0;h<height;h++)
	{
		int wy = 0;
		int wuv = 0;
		int offset = h * linesize;
		int offsety = h * width;
		int offsetuv = h * uvlinesize;
		int w = 0;

		for(w=0;w<linesize;w+=4)
		{
			/*y00*/
			framebuffer[w + offset] = py[wy + offsety];
			/*u0*/
			framebuffer[(w + 1) + offset] = pu[wuv + offsetuv];
			/*y01*/
			framebuffer[(w + 2) + offset] = py[(wy + 1) + offsety];
			/*v0*/
			framebuffer[(w + 3) + offset] = pv[wuv + offsetuv];

			wuv++;
			wy+=2;
		}
	}
}

/*
 * convert yvu 420 planar (yv12) to yuv 422
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing yuv420 planar data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void yvu420_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint8_t *py;
	uint8_t *pv;
	uint8_t *pu;

	int linesize = width * 2;
	int uvlinesize = width / 2;

	py=tmpbuffer;
	pv=py+(width*height);
	pu=pv+((width*height)/4);

	int h=0;
	int huv=0;

	for(h=0;h<height;h+=2)
	{
		int wy=0;
		int wuv=0;
		int offset = h * linesize;
		int offset1 = (h + 1) * linesize;
		int offsety = h * width;
		int offsety1 = (h + 1) * width;
		int offsetuv = huv * uvlinesize;
		int w = 0;

		for(w=0;w<linesize;w+=4)
		{
			/*y00*/
			framebuffer[w + offset] = py[wy + offsety];
			/*u0*/
			framebuffer[(w + 1) + offset] = pu[wuv + offsetuv];
			/*y01*/
			framebuffer[(w + 2) + offset] = py[(wy + 1) + offsety];
			/*v0*/
			framebuffer[(w + 3) + offset] = pv[wuv + offsetuv];

			/*y10*/
			framebuffer[w + offset1] = py[wy + offsety1];
			/*u0*/
			framebuffer[(w + 1) + offset1] = pu[wuv + offsetuv];
			/*y11*/
			framebuffer[(w + 2) + offset1] = py[(wy + 1) + offsety1];
			/*v0*/
			framebuffer[(w + 3) + offset1] = pv[wuv + offsetuv];

			wuv++;
			wy+=2;
		}
		huv++;
	}
}

/*
 * convert yuv 420 planar (uv interleaved) (nv12) to yuv 422
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing yuv420 (nv12) planar data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void nv12_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint8_t *py;
	uint8_t *puv;

	int linesize = width * 2;

	py=tmpbuffer;
	puv=py+(width*height);

	int h=0;
	int huv=0;

	for(h=0;h<height;h+=2)
	{
		int wy=0;
		int wuv=0;
		int offset = h * linesize;
		int offset1 = (h+1) * linesize;
		int offsety = h * width;
		int offsety1 = (h+1) * width;
		int offsetuv = huv * width;
		int w = 0;

		for(w=0;w<linesize;w+=4)
		{
			/*y00*/
			framebuffer[w + offset] = py[wy + offsety];
			/*u0*/
			framebuffer[(w + 1) + offset] = puv[wuv + offsetuv];
			/*y01*/
			framebuffer[(w + 2) + offset] = py[(wy + 1) + offsety];
			/*v0*/
			framebuffer[(w + 3) + offset] = puv[(wuv + 1) + offsetuv];

			/*y10*/
			framebuffer[w + offset1] = py[wy + offsety1];
			/*u0*/
			framebuffer[(w + 1) + offset1] = puv[wuv + offsetuv];
			/*y11*/
			framebuffer[(w + 2) + offset1] = py[(wy + 1) + offsety1];
			/*v0*/
			framebuffer[(w + 3) + offset1] = puv[(wuv + 1) + offsetuv];

			wuv+=2;
			wy+=2;
		}
		huv++;
	}
}

/*
 * convert yuv 420 planar (vu interleaved) (nv21) to yuv 422
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing yuv420 (nv21) planar data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void nv21_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint8_t *py;
	uint8_t *puv;

	int linesize = width * 2;

	py=tmpbuffer;
	puv=py+(width*height);

	int h=0;
	int huv=0;

	for(h=0;h<height;h+=2)
	{
		int wy=0;
		int wuv=0;
		int offset = h * linesize;
		int offset1 = (h+1) * linesize;
		int offsety = h * width;
		int offsety1 = (h+1) * width;
		int offsetuv = huv * width;
		int w = 0;

		for(w=0;w<linesize;w+=4)
		{
			/*y00*/
			framebuffer[w + offset] = py[wy + offsety];
			/*u0*/
			framebuffer[(w + 1) + offset] = puv[(wuv + 1) + offsetuv];
			/*y01*/
			framebuffer[(w + 2) + offset] = py[(wy + 1) + offsety];
			/*v0*/
			framebuffer[(w + 3) + offset] = puv[wuv + offsetuv];

			/*y10*/
			framebuffer[w + offset1] = py[wy + offsety1];
			/*u0*/
			framebuffer[(w + 1) + offset1] = puv[(wuv + 1) + offsetuv];
			/*y11*/
			framebuffer[(w + 2) + offset1] = py[(wy + 1) + offsety1];
			/*v0*/
			framebuffer[(w + 3) + offset1] = puv[wuv + offsetuv];

			wuv+=2;
			wy+=2;
		}
		huv++;
	}
}

/*
 * convert yuv 422 planar (uv interleaved) (nv16) to yuv 422
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing yuv422 (nv16) planar data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void nv16_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint8_t *py;
	uint8_t *puv;

	int linesize = width * 2;

	py=tmpbuffer;
	puv=py+(width*height);

	int h=0;
	int huv=0;

	for(h=0;h<height;h++)
	{
		int wy=0;
		int wuv=0;
		int offset = h * linesize;
		int offsety = h * width;
		int offsetuv = huv * width;
		int w = 0;

		for(w=0;w<linesize;w+=4)
		{
			/*y00*/
			framebuffer[w + offset] = py[wy + offsety];
			/*u0*/
			framebuffer[(w + 1) + offset] = puv[wuv + offsetuv];
			/*y01*/
			framebuffer[(w + 2) + offset] = py[(wy + 1) + offsety];
			/*v0*/
			framebuffer[(w + 3) + offset] = puv[(wuv + 1) + offsetuv];

			wuv+=2;
			wy+=2;
		}
		huv++;
	}
}

/*
 * convert yuv 422 planar (vu interleaved) (nv61) to yuv 422
 * args:
 *    framebuffer: pointer to frame buffer (yuyv)
 *    tmpbuffer: pointer to temp buffer containing yuv422 (nv61) planar data frame
 *    width: picture width
 *    height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void nv61_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	uint8_t *py;
	uint8_t *puv;

	int linesize = width * 2;

	py=tmpbuffer;
	puv=py+(width*height);

	int h=0;
	int huv=0;

	for(h=0;h<height;h++)
	{
		int wy=0;
		int wuv=0;
		int offset = h * linesize;
		int offsety = h * width;
		int offsetuv = huv * width;
		int w = 0;
		for(w=0;w<linesize;w+=4)
		{
			/*y00*/
			framebuffer[w + offset] = py[wy + offsety];
			/*u0*/
			framebuffer[(w + 1) + offset] = puv[(wuv + 1) + offsetuv];
			/*y01*/
			framebuffer[(w + 2) + offset] = py[(wy + 1) + offsety];
			/*v0*/
			framebuffer[(w + 3) + offset] = puv[wuv + offsetuv];

			wuv+=2;
			wy+=2;
		}
		huv++;
	}
}

/*
 * convert yuv 411 packed (y41p) to yuv 422
 * args:
 *    framebuffer: pointer to frame buffer (yuyv)
 *    tmpbuffer: pointer to temp buffer containing y41p data frame
 *    width: picture width
 *    height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void y41p_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	int h=0;
	int linesize = width * 3 /2;

	for(h=0;h<height;h++)
	{
		int offset = linesize * h;
		int w = 0;
		for(w=0;w<linesize;w+=12)
		{
			*framebuffer++=tmpbuffer[w+1 + offset]; //Y0
			*framebuffer++=tmpbuffer[w   + offset]; //U0
			*framebuffer++=tmpbuffer[w+3 + offset]; //Y1
			*framebuffer++=tmpbuffer[w+2 + offset]; //V0
			*framebuffer++=tmpbuffer[w+5 + offset]; //Y2
			*framebuffer++=tmpbuffer[w   + offset]; //U0
			*framebuffer++=tmpbuffer[w+7 + offset]; //Y3
			*framebuffer++=tmpbuffer[w+2 + offset]; //V0
			*framebuffer++=tmpbuffer[w+8 + offset]; //Y4
			*framebuffer++=tmpbuffer[w+4 + offset]; //U4
			*framebuffer++=tmpbuffer[w+9 + offset]; //Y5
			*framebuffer++=tmpbuffer[w+6 + offset]; //V4
			*framebuffer++=tmpbuffer[w+10+ offset]; //Y6
			*framebuffer++=tmpbuffer[w+4 + offset]; //U4
			*framebuffer++=tmpbuffer[w+11+ offset]; //Y7
			*framebuffer++=tmpbuffer[w+6 + offset]; //V4
		}
	}
}

/*
 * convert yuv mono (grey) to yuv 422
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing grey (y only) data frame
 *   width: picture width
 *    height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void grey_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	int h=0;

	for(h=0;h<height;h++)
	{
		int offset = width * h;
		int w = 0;
		for(w=0;w<width;w++)
		{
            *framebuffer++=tmpbuffer[w + offset]; //Y
            *framebuffer++=0x80;                  //U or V
		}
	}
}

/*
 * convert SPCA501 (s501) to yuv 422
 *   s501  |Y0..width..Y0|U..width/2..U|Y1..width..Y1|V..width/2..V|
 *   signed values (-128;+127) must be converted to unsigned (0; 255)
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing s501 data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void s501_to_yuyv(uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	int h = 0;
	uint8_t *Y0 = tmpbuffer; /*fisrt line*/

	for (h = 0; h < height/2; h++ )
	{
		uint8_t *line2 = framebuffer + width * 2;   /* next line          */
		uint8_t *U = Y0 + width;
		uint8_t *Y1 = U + width / 2;
		uint8_t *V = Y1 + width;
		int w = 0;

		for (w = width / 2; --w >= 0; )
		{
			*framebuffer++ = 0x80 + *Y0++;
			*framebuffer++ = 0x80 + *U;
			*framebuffer++ = 0x80 + *Y0++;
			*framebuffer++ = 0x80 + *V;

			*line2++ = 0x80 + *Y1++;
			*line2++ = 0x80 + *U++;
			*line2++ = 0x80 + *Y1++;
			*line2++ = 0x80 + *V++;
		}
		Y0 += width * 2;                  /* next block of lines */
		framebuffer = line2;
	}
}

/*
 * convert SPCA505 (s505) to yuv 422
 *   s505  |Y0..width..Y0|Y1..width..Y1|U..width/2..U|V..width/2..V|
 *   signed values (-128;+127) must be converted to unsigned (0; 255)
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing s501 data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void s505_to_yuyv(uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	int h = 0;
	uint8_t *Y0 = tmpbuffer; /*fisrt line*/

	for (h = 0; h < height/2; h++ )
	{
		uint8_t *line2 = framebuffer + width * 2;   /* next line          */
		uint8_t *Y1 = Y0 + width;
		uint8_t *U  = Y1 + width;
		uint8_t *V  = U + width/2;
		int w = 0;

		for (w = width / 2; --w >= 0; )
		{
			*framebuffer++ = 0x80 + *Y0++;
			*framebuffer++ = 0x80 + *U;
			*framebuffer++ = 0x80 + *Y0++;
			*framebuffer++ = 0x80 + *V;

			*line2++ = 0x80 + *Y1++;
			*line2++ = 0x80 + *U++;
			*line2++ = 0x80 + *Y1++;
			*line2++ = 0x80 + *V++;
		}
		Y0 += width * 2;                  /* next block of lines */
		framebuffer = line2;
	}
}

/*
 * convert SPCA508 (s508) to yuv 422
 *   s508  |Y0..width..Y0|U..width/2..U|V..width/2..V|Y1..width..Y1|
 *   signed values (-128;+127) must be converted to unsigned (0; 255)
 * args:
 *   framebuffer: pointer to frame buffer (yuyv)
 *   tmpbuffer: pointer to temp buffer containing s501 data frame
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *    none
 *
 * returns: none
 */
void s508_to_yuyv(uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height)
{
	int h = 0;
	uint8_t *Y0 = tmpbuffer; /*fisrt line*/

	for (h = 0; h < height/2; h++ )
	{
		uint8_t *line2 = framebuffer + width * 2;   /* next line          */
		uint8_t *U = Y0 + width;
		uint8_t *V = U + width/2;
		uint8_t *Y1= V + width/2;
		int w = 0;

		for (w = width / 2; --w >= 0; )
		{
			*framebuffer++ = 0x80 + *Y0++;
			*framebuffer++ = 0x80 + *U;
			*framebuffer++ = 0x80 + *Y0++;
			*framebuffer++ = 0x80 + *V;

			*line2++ = 0x80 + *Y1++;
			*line2++ = 0x80 + *U++;
			*line2++ = 0x80 + *Y1++;
			*line2++ = 0x80 + *V++;
		}
		Y0 += width * 2;                  /* next block of lines */
		framebuffer = line2;
	}
}

/*
 * convert rgb24 to yuyv
 * args:
 *   prgb: pointer to input buffer containing rgb data
 *   pyuv: pointer to output buffer containing converted yuyv data
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *   none
 *
 * returns: none
 */
void rgb2yuyv(uint8_t *prgb, uint8_t *pyuv, int width, int height)
{

	int i=0;
	for(i=0;i<(width*height*3);i=i+6)
	{
		/* y */
		*pyuv++ =CLIP(0.299 * (prgb[i] - 128) + 0.587 * (prgb[i+1] - 128) + 0.114 * (prgb[i+2] - 128) + 128);
		/* u */
		*pyuv++ =CLIP(((- 0.147 * (prgb[i] - 128) - 0.289 * (prgb[i+1] - 128) + 0.436 * (prgb[i+2] - 128) + 128) +
			(- 0.147 * (prgb[i+3] - 128) - 0.289 * (prgb[i+4] - 128) + 0.436 * (prgb[i+5] - 128) + 128))/2);
		/* y1 */
		*pyuv++ =CLIP(0.299 * (prgb[i+3] - 128) + 0.587 * (prgb[i+4] - 128) + 0.114 * (prgb[i+5] - 128) + 128);
		/* v*/
		*pyuv++ =CLIP(((0.615 * (prgb[i] - 128) - 0.515 * (prgb[i+1] - 128) - 0.100 * (prgb[i+2] - 128) + 128) +
			(0.615 * (prgb[i+3] - 128) - 0.515 * (prgb[i+4] - 128) - 0.100 * (prgb[i+5] - 128) + 128))/2);
	}
}

/*
 * convert bgr24 to yuyv
 * args:
 *   pbgr: pointer to input buffer containing bgr data
 *   pyuv: pointer to output buffer containing converted yuyv data
 *   width: picture width
 *   height: picture height
 *
 * asserts:
 *   none
 *
 * returns: none
 */
void bgr2yuyv(uint8_t *pbgr, uint8_t *pyuv, int width, int height)
{

	int i=0;
	for(i=0;i<(width*height*3);i=i+6)
	{
		/* y */
		*pyuv++ =CLIP(0.299 * (pbgr[i+2] - 128) + 0.587 * (pbgr[i+1] - 128) + 0.114 * (pbgr[i] - 128) + 128);
		/* u */
		*pyuv++ =CLIP(((- 0.147 * (pbgr[i+2] - 128) - 0.289 * (pbgr[i+1] - 128) + 0.436 * (pbgr[i] - 128) + 128) +
			(- 0.147 * (pbgr[i+5] - 128) - 0.289 * (pbgr[i+4] - 128) + 0.436 * (pbgr[i+3] - 128) + 128))/2);
		/* y1 */
		*pyuv++ =CLIP(0.299 * (pbgr[i+5] - 128) + 0.587 * (pbgr[i+4] - 128) + 0.114 * (pbgr[i+3] - 128) + 128);
		/* v*/
		*pyuv++ =CLIP(((0.615 * (pbgr[i+2] - 128) - 0.515 * (pbgr[i+1] - 128) - 0.100 * (pbgr[i] - 128) + 128) +
			(0.615 * (pbgr[i+5] - 128) - 0.515 * (pbgr[i+4] - 128) - 0.100 * (pbgr[i+3] - 128) + 128))/2);
	}
}

/*
 * used for internal jpeg decoding  420 planar to 422
 * args:
 *   out: pointer to data output of idct (macroblocks yyyy u v)
 *   pic: pointer to picture buffer (yuyv)
 *   width: picture width
 *
 * asserts:
 *   none
 *
 * returns: none
 */
void yuv420pto422(int *out, uint8_t *pic, int width)
{
	int j = 0;
	int outy1 = 0;
	int outy2 = 8;
	//yyyyuv
	uint8_t *pic0 = pic;
	uint8_t *pic1 = pic + width;
	int *outy = out;
	int *outu = out + 64 * 4;
	int *outv = out + 64 * 5;

	for (j = 0; j < 8; j++)
	{
		int k = 0;
		for (k = 0; k < 8; k++)
		{
			if( k == 4)
			{
				outy1 += 56;
				outy2 += 56;
			}
			*pic0++ = CLIP(outy[outy1]);   //y1 line 1
			*pic0++ = CLIP(128 + *outu);   //u  line 1-2
			*pic0++ = CLIP(outy[outy1+1]); //y2 line 1
			*pic0++ = CLIP(128 + *outv);   //v  line 1-2
			*pic1++ = CLIP(outy[outy2]);   //y1 line 2
			*pic1++ = CLIP(128 + *outu);   //u  line 1-2
			*pic1++ = CLIP(outy[outy2+1]); //y2 line 2
			*pic1++ = CLIP(128 + *outv);   //v  line 1-2
			outy1 +=2; outy2 += 2; outu++; outv++;
		}
		if(j==3)
		{
			outy = out + 128;
		}
		else
		{
			outy += 16;
		}
		outy1 = 0;
		outy2 = 8;
		pic0 += 2 * (width -16);
		pic1 += 2 * (width -16);
	}
}

/*
 * used for internal jpeg decoding 422 planar to 422
 * args:
 *   out: pointer to data output of idct (macroblocks yyyy u v)
 *   pic: pointer to picture buffer (yuyv)
 *   width: picture width
 *
 * asserts:
 *   none
 *
 * returns: none
 */
void yuv422pto422(int *out, uint8_t *pic, int width)
{
	int j = 0;
	int outy1 = 0;
	int outy2 = 8;
	int outu1 = 0;
	int outv1 = 0;
	//yyyyuv
	uint8_t *pic0 = pic;
	uint8_t *pic1 = pic + width;
	int *outy = out;
	int *outu = out + 64 * 4;
	int *outv = out + 64 * 5;

	for (j = 0; j < 4; j++)
	{
		int k = 0;
		for (k = 0; k < 8; k++)
		{
			if( k == 4)
			{
				outy1 += 56;
				outy2 += 56;
			}
			*pic0++ = CLIP(outy[outy1]);        //y1 line 1
			*pic0++ = CLIP(128 + outu[outu1]);  //u  line 1
			*pic0++ = CLIP(outy[outy1+1]);      //y2 line 1
			*pic0++ = CLIP(128 + outv[outv1]);  //v  line 1
			*pic1++ = CLIP(outy[outy2]);        //y1 line 2
			*pic1++ = CLIP(128 + outu[outu1+8]);//u  line 2
			*pic1++ = CLIP(outy[outy2+1]);      //y2 line 2
			*pic1++ = CLIP(128 + outv[outv1+8]);//v  line 2
			outv1 += 1; outu1 += 1;
			outy1 +=2; outy2 +=2;
		}
		outy += 16;outu +=8; outv +=8;
		outv1 = 0; outu1=0;
		outy1 = 0;
		outy2 = 8;
		pic0 += 2 * (width -16);
		pic1 += 2 * (width -16);
	}
}

/*
 * used for internal jpeg decoding 444 planar to 422
 * args:
 *   out: pointer to data output of idct (macroblocks yyyy u v)
 *   pic: pointer to picture buffer (yuyv)
 *   width: picture width
 *
 * asserts:
 *   none
 *
 * returns: none
 */
void yuv444pto422(int *out, uint8_t *pic, int width)
{
	int j = 0;
	int outy1 = 0;
	int outy2 = 8;
	int outu1 = 0;
	int outv1 = 0;
	//yyyyuv
	uint8_t *pic0 = pic;
	uint8_t *pic1 = pic + width;
	int *outy = out;
	int *outu = out + 64 * 4; // Ooops where did i invert ??
	int *outv = out + 64 * 5;

	for (j = 0; j < 4; j++)
	{
		int k = 0;
		for (k = 0; k < 4; k++)
		{
			*pic0++ =CLIP( outy[outy1]);        //y1 line 1
			*pic0++ =CLIP( 128 + outu[outu1]);  //u  line 1
			*pic0++ =CLIP( outy[outy1+1]);      //y2 line 1
			*pic0++ =CLIP( 128 + outv[outv1]);  //v  line 1
			*pic1++ =CLIP( outy[outy2]);        //y1 line 2
			*pic1++ =CLIP( 128 + outu[outu1+8]);//u  line 2
			*pic1++ =CLIP( outy[outy2+1]);      //y2 line 2
			*pic1++ =CLIP( 128 + outv[outv1+8]);//v  line 2
			outv1 += 2; outu1 += 2;
			outy1 +=2; outy2 +=2;
		}
		outy += 16;outu +=16; outv +=16;
		outv1 = 0; outu1=0;
		outy1 = 0;
		outy2 = 8;
		pic0 += 2 * (width -8);
		pic1 += 2 * (width -8);
	}
}

/*
 * used for internal jpeg decoding 400 planar to 422
 * args:
 *   out: pointer to data output of idct (macroblocks yyyy )
 *   pic: pointer to picture buffer (yuyv)
 *   width: picture width
 *
 * asserts:
 *   none
 *
 * returns: none
 */
void yuv400pto422(int *out, uint8_t *pic, int width)
{
	int j = 0;
	int outy1 = 0;
	int outy2 = 8;
	uint8_t *pic0 = pic;
	uint8_t *pic1 = pic + width;
	int *outy = out;

	//yyyy
	for (j = 0; j < 4; j++)
	{
		int k = 0;
		for (k = 0; k < 4; k++)
		{
			*pic0++ = CLIP(outy[outy1]);  //y1 line 1
			*pic0++ = 128 ;               //u
			*pic0++ = CLIP(outy[outy1+1]);//y2 line 1
			*pic0++ = 128 ;               //v
			*pic1++ = CLIP(outy[outy2]);  //y1 line 2
			*pic1++ = 128 ;               //u
			*pic1++ = CLIP(outy[outy2+1]);//y2 line 2
			*pic1++ = 128 ;               //v
			outy1 +=2; outy2 +=2;
		}
		outy += 16;
		outy1 = 0;
		outy2 = 8;
		pic0 += 2 * (width -8);
		pic1 += 2 * (width -8);
	}
}
