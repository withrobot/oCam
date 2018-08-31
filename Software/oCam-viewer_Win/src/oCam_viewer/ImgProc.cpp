#include "stdafx.h"
#include "ImgProc.h"
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>


#define uchar	unsigned char

#define  CV_CAST_8U(t)  (uchar)(!((t) & ~255) ? (t) : (t) > 0 ? 255 : 0)
#define fix(x,n)      (int)((x)*(1 << (n)) + 0.5)
//#define descale       CV_DESCALE

#define cscGr_32f  0.299f
#define cscGg_32f  0.587f
#define cscGb_32f  0.114f

/* BGR/RGB -> YCrCb */
#define  CV_DESCALE(x,n)     (((x) + (1 << ((n)-1))) >> (n))

#define yuvYr_32f cscGr_32f
#define yuvYg_32f cscGg_32f
#define yuvYb_32f cscGb_32f
#define yuvCr_32f 0.713f
#define yuvCb_32f 0.564f

#define yuv_shift 14
#define yuvYr  fix(yuvYr_32f,yuv_shift)
#define yuvYg  fix(yuvYg_32f,yuv_shift)
#define yuvYb  fix(yuvYb_32f,yuv_shift)
#define yuvCr  fix(yuvCr_32f,yuv_shift)
#define yuvCb  fix(yuvCb_32f,yuv_shift)

#define yuv_descale(x)  CV_DESCALE((x), yuv_shift)
#define yuv_prescale(x) ((x) << yuv_shift)

#define  yuvRCr_32f   1.403f
#define  yuvGCr_32f   (-0.714f)
#define  yuvGCb_32f   (-0.344f)
#define  yuvBCb_32f   1.773f

#define  yuvRCr   fix(yuvRCr_32f,yuv_shift)
#define  yuvGCr   (-fix(-yuvGCr_32f,yuv_shift))
#define  yuvGCb   (-fix(-yuvGCb_32f,yuv_shift))
#define  yuvBCb   fix(yuvBCb_32f,yuv_shift)


//void YUV2RGB(const unsigned char* Src, unsigned char* Dst, int size, int Y0=0, int U=1, int Y1=2, int V=3)
void YUV2RGB(const unsigned char* Src, unsigned char* Dst, int Size, int Y0, int U, int Y1, int V)
{         
	int Y, Cr, Cb;
	int b, g, r;

	Size *= 2;		// YUV¸¦ ±âÁØ
	for (int i = 0; i < Size; i += 4, Dst += 6 )
	{
		Y = yuv_prescale(Src[i+Y0]);
		Cb = Src[i+U] - 128;
		Cr = Src[i+V] - 128;

		b = yuv_descale( Y + yuvBCb*Cb );
		g = yuv_descale( Y + yuvGCr*Cr + yuvGCb*Cb );
		r = yuv_descale( Y + yuvRCr*Cr );

		Dst[0] = CV_CAST_8U(b);
		Dst[1] = CV_CAST_8U(g);
		Dst[2] = CV_CAST_8U(r);

		Y = yuv_prescale(Src[i+Y1]);
		Cb = Src[i+U] - 128;
		Cr = Src[i+V] - 128;

		b = yuv_descale( Y + yuvBCb*Cb );
		g = yuv_descale( Y + yuvGCr*Cr + yuvGCb*Cb );
		r = yuv_descale( Y + yuvRCr*Cr );

		Dst[3] = CV_CAST_8U(b);
		Dst[4] = CV_CAST_8U(g);
		Dst[5] = CV_CAST_8U(r);
	}
}

void YUV2Gray(const unsigned char* Src, unsigned char* Dst, int Size, int Y)
{
	for (int i=0; i<Size; i++)
	{
		Dst[i] = Src[i*2+Y];
	}
}

void Bayer2RGB( char* Bayer, char *Dst, int Width, int Height, int Code )
{
    int blue = Code == BayerBG2BGR || Code == BayerGB2BGR ? -1 : 1;
    int start_with_green = Code == BayerGB2BGR || Code == BayerGR2BGR;

	int bayer_step = Width;
	int dst_step = Width*3;

    memset( Dst, 0, Width*3*sizeof(Dst[0]) );
    memset( Dst + (Height - 1)*dst_step, 0, Width*3*sizeof(Dst[0]) );
    Dst += dst_step + 3 + 1;
    Height -= 2;
    Width -= 2;

    for( ; Height-- > 0; Bayer += bayer_step, Dst += dst_step )
    {
        int t0, t1;
        unsigned char* bayer = (unsigned char *)Bayer;
        unsigned char* dst = (unsigned char *)Dst;
        unsigned char* bayer_end = bayer + Width;

        dst[-4] = dst[-3] = dst[-2] = dst[Width*3-1] =
            dst[Width*3] = dst[Width*3+1] = 0;

        if( Width <= 0 )
            continue;

        if( start_with_green )
        {
            t0 = (bayer[1] + bayer[bayer_step*2+1] + 1) >> 1;
            t1 = (bayer[bayer_step] + bayer[bayer_step+2] + 1) >> 1;
            dst[-blue] = (unsigned char)t0;
            dst[0] = bayer[bayer_step+1];
            dst[blue] = (unsigned char)t1;
            bayer++;
            dst += 3;
        }

        if( blue > 0 )
        {
            for( ; bayer <= bayer_end - 2; bayer += 2, dst += 6 )
            {
                t0 = (bayer[0] + bayer[2] + bayer[bayer_step*2] +
                      bayer[bayer_step*2+2] + 2) >> 2;
                t1 = (bayer[1] + bayer[bayer_step] +
                      bayer[bayer_step+2] + bayer[bayer_step*2+1]+2) >> 2;
                dst[-1] = (unsigned char)t0;
                dst[0] = (unsigned char)t1;
                dst[1] = bayer[bayer_step+1];

                t0 = (bayer[2] + bayer[bayer_step*2+2] + 1) >> 1;
                t1 = (bayer[bayer_step+1] + bayer[bayer_step+3] + 1) >> 1;
                dst[2] = (unsigned char)t0;
                dst[3] = bayer[bayer_step+2];
                dst[4] = (unsigned char)t1;
            }
        }
        else
        {
            for( ; bayer <= bayer_end - 2; bayer += 2, dst += 6 )
            {
                t0 = (bayer[0] + bayer[2] + bayer[bayer_step*2] +
                      bayer[bayer_step*2+2] + 2) >> 2;
                t1 = (bayer[1] + bayer[bayer_step] +
                      bayer[bayer_step+2] + bayer[bayer_step*2+1]+2) >> 2;
                dst[1] = (unsigned char)t0;
                dst[0] = (unsigned char)t1;
                dst[-1] = bayer[bayer_step+1];

                t0 = (bayer[2] + bayer[bayer_step*2+2] + 1) >> 1;
                t1 = (bayer[bayer_step+1] + bayer[bayer_step+3] + 1) >> 1;
                dst[4] = (unsigned char)t0;
                dst[3] = bayer[bayer_step+2];
                dst[2] = (unsigned char)t1;
            }
        }

        if( bayer < bayer_end )
        {
            t0 = (bayer[0] + bayer[2] + bayer[bayer_step*2] +
                  bayer[bayer_step*2+2] + 2) >> 2;
            t1 = (bayer[1] + bayer[bayer_step] +
                  bayer[bayer_step+2] + bayer[bayer_step*2+1]+2) >> 2;
            dst[-blue] = (unsigned char)t0;
            dst[0] = (unsigned char)t1;
            dst[blue] = bayer[bayer_step+1];
            bayer++;
            dst += 3;
        }

        blue = -blue;
        start_with_green = !start_with_green;
    }
}