/*******************************************************************************#
#           guvcview              http://guvcview.sourceforge.net               #
#                                                                               #
#           Paulo Assis <pj.assis@gmail.com>                                    #
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

#ifndef COLORSPACES_H
#define COLORSPACES_H

#include <inttypes.h>
#include <sys/types.h>
#include <libintl.h>

/*
 * Error Codes
 */
#define E_OK                      (0)
#define E_ALLOC_ERR               (-1)
#define E_QUERYCAP_ERR            (-2)
#define E_READ_ERR                (-3)
#define E_MMAP_ERR                (-4)
#define E_QUERYBUF_ERR            (-5)
#define E_QBUF_ERR                (-6)
#define E_DQBUF_ERR               (-7)
#define E_STREAMON_ERR            (-8)
#define E_STREAMOFF_ERR           (-9)
#define E_FORMAT_ERR              (-10)
#define E_REQBUFS_ERR             (-11)
#define E_DEVICE_ERR              (-12)
#define E_SELECT_ERR              (-13)
#define E_SELECT_TIMEOUT_ERR      (-14)
#define E_FBALLOC_ERR             (-15)
#define E_NO_STREAM_ERR           (-16)
#define E_NO_DATA                 (-17)
#define E_NO_CODEC                (-18)
#define E_DECODE_ERR              (-19)
#define E_BAD_TABLES_ERR          (-20)
#define E_NO_SOI_ERR              (-21)
#define E_NOT_8BIT_ERR            (-22)
#define E_BAD_WIDTH_OR_HEIGHT_ERR (-23)
#define E_TOO_MANY_COMPPS_ERR     (-24)
#define E_ILLEGAL_HV_ERR          (-25)
#define E_QUANT_TBL_SEL_ERR       (-26)
#define E_NOT_YCBCR_ERR           (-27)
#define E_UNKNOWN_CID_ERR         (-28)
#define E_WRONG_MARKER_ERR        (-29)
#define E_NO_EOI_ERR              (-30)
#define E_FILE_IO_ERR             (-31)
#define E_UNKNOWN_ERR             (-40)

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

/*clip value between 0 and 255*/
#define CLIP(value) (uint8_t)(((value)>0xFF)?0xff:(((value)<0)?0:(value)))

#ifdef __cplusplus
extern "C" {
#endif

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
void yuyv_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void yvyu_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void uyvy_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void yuv422p_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void yyuv_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void yv12_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void nv12_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void nv21_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void nv16_to_yu12 (uint8_t *out, uint8_t *in, int width, int height);

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
void nv61_to_yu12 (uint8_t *out, uint8_t *in, int width, int height);

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
void y10b_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void y41p_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void grey_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void y16_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
 *    none
 *
 * returns: none
 */
void s501_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void s505_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void s508_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void rgb24_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void bgr24_to_yu12(uint8_t *out, uint8_t *in, int width, int height);

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
void yu12_to_rgb24 (uint8_t *out, uint8_t *in, int width, int height);

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
void yu12_to_dib24 (uint8_t *out, uint8_t *in, int width, int height);

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
void yu12_to_yuyv (uint8_t *out, uint8_t *in, int width, int height);

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
void yuyv2rgb (uint8_t *pyuv, uint8_t *prgb, int width, int height);

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
//void yuyv2bgr1 (uint8_t *pyuv, uint8_t *pbgr, int width, int height);

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
void yuyv2bgr (uint8_t *pyuv, uint8_t *pbgr, int width, int height);

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
void y10b_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void y16_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void yyuv_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void uyvy_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void yvyu_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void yvu420_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void nv12_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void nv21_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void nv16_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void nv61_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void y41p_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void grey_to_yuyv (uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void s501_to_yuyv(uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void s505_to_yuyv(uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void s508_to_yuyv(uint8_t *framebuffer, uint8_t *tmpbuffer, int width, int height);

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
void bayer_to_rgb24(uint8_t *pBay, uint8_t *pRGB24, int width, int height, int pix_order);

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
void rgb2yuyv(uint8_t *prgb, uint8_t *pyuv, int width, int height);

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
void bgr2yuyv(uint8_t *pbgr, uint8_t *pyuv, int width, int height);

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
void yuv420pto422(int *out, uint8_t *pic, int width);

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
void yuv422pto422(int *out, uint8_t *pic, int width);

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
void yuv444pto422(int *out, uint8_t *pic, int width);

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
void yuv400pto422(int *out, uint8_t *pic, int width);

#ifdef __cplusplus
} /* extern C */
#endif

#endif

