#ifndef IMG_PROC_H_
#define IMG_PROC_H_

#define  BayerBG2BGR 1
#define  BayerGB2BGR 2
#define  BayerRG2BGR 3
#define  BayerGR2BGR 4

#define  BayerBG2RGB BayerRG2BGR
#define  BayerGB2RGB BayerGR2BGR
#define  BayerRG2RGB BayerBG2BGR
#define  BayerGR2RGB BayerGB2BGR

void YUV2RGB(const unsigned char* Src, unsigned char* Dst, int Size, int Y0=1, int U=0, int Y1=3, int V=2);
void YUV2Gray(const unsigned char* Src, unsigned char* Dst, int Size, int Y=1);
void Bayer2RGB( char* Bayer, char *Dst, int Width, int Height, int Code );

#endif /*IMG_PROC_H_*/