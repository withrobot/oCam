extern "C" {
#include <string.h>
#include <inttypes.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <math.h>
#include <libavutil/avassert.h>
#include <libavutil/channel_layout.h>
#include <libavutil/mathematics.h>
#include <libavutil/timestamp.h>
}
#include <direct.h>

void YUYV2YUV420P_Converter(const unsigned char* Src, int Width, int Height, int Y0, int U, int Y1, int V);
void Gray2YUV420p_Converter(const unsigned char* Src, int Width, int Height, int Y0, int U, int Y1, int V);
void RGB2YUV420P_Converter(const unsigned char* Src, int Width, int Height);
bool Record_info(int Width, int Height, int Fps);
int Record_End();
void Encode(AVCodecContext* AVCodecContext, AVFrame* frame, AVPacket* pkt, FILE* outfile);
