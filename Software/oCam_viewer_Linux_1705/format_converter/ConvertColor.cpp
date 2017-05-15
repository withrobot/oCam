/*
 * ConvertColor.cpp
 *
 *  Created on: Sep 23, 2016
 *      Author: gnohead
 */

#include "ConvertColor.h"

#include "withrobot_debug_print.h"

#include <cstring>
#include <cmath>

/*
 *  legacy
 *
 *  ```
 *  참고 코드는 OpenCV 라이브러리의 코드. 다음 참고링크[1]의 "dc1394_bayer_Bilinear(...)" 와 구조와 내용이 같음
 *  즉, Bilinear De-mosicing 방법론을 바탕으로 작성
 *
 *  참고링크[1]: http://libdc1394-22.sourcearchive.com/documentation/2.0.2/bayer_8c-source.html
 *  ```
 */
void Bayer2BGR(unsigned char* bayer, unsigned char *bgr, int width, int height, int code, double gainBlue, double gainGreen, double gainRed)
{
    static const int DIV_2_ROUNDING_OFF     = 1;
    static const int DIV_4_ROUNDING_OFF     = 2;

    static const int BGR_PIXEL_WIDTH        = 3;
    static const int BAYER_PATTERN_WIDTH    = 2;
    static const int BAYER_PATTERN_HEIGHT   = 2;

    //DBG_PRINTF("Bayer2RGB :: Width:%d, Height:%d\n", width, height);

    // white balance test 를 위함 (각 픽셀에 특정 scale을 곱한다)
    double gain[] = {gainBlue, gainGreen, gainRed};

    /*
     * bayer 이미지의 배열 형태를 결정하는 곳, 정방형 2x2 중에 (1,1), (1,2)위치가 BG 인지 GB 인지 판단
     */
    int blue        = ((code == BayerBG2BGR) || (code == BayerGB2BGR)) ? -1 : 1;
    int at_green    = ((code == BayerGB2BGR) || (code == BayerGR2BGR));

    /*
     * 변환 결과 들어갈 bgr 공간 초기화
     */
    memset(bgr, 0, width*height*BGR_PIXEL_WIDTH*sizeof(bgr[0]));        // 전체 초기화

    /*
     *  gnoehad: bgr 이미지의 시작점을 설정
     *
     *  두번째 줄 두번째 bgr 픽셀의 Green 주소 부터 시작
     */
    bgr += (width + 1)*BGR_PIXEL_WIDTH + 1;

    /*
     * 이미지 상,하,좌,우 1픽셀씩 빠짐
     */
    int conv_img_height = height - BAYER_PATTERN_HEIGHT;
    int conv_img_width  = width  - BAYER_PATTERN_WIDTH;

    /*
     *  이미지의 한 줄씩 계산한다.
     */
    while(conv_img_height--)
    {
        unsigned char* bayer_end = bayer + conv_img_width;

        while(bayer - bayer_end < 0)
        {
            /*
             * 코드 구조는 bilinear demosicing
             *
             *  정수형 나눗셈 계산을 위해 2로 나누는 건 1을 더하고, 4로 나누는건 2를 더함 (정수형 나눗셈의 반올림 효과)
             *
             *  bayer 포인터가 green 에 있을 때와 아닐때의 계산방법이 다르다. 나머지 색은 같다.
             */
#if 1
            int idxBlue = 0;
            int idxRed = 2;

            if (blue < 0) {
                idxBlue = 2;
                idxRed = 0;
            }

            if( at_green ) {
                // Green 픽셀에 대한 BGR 계산
                bgr[-blue]  = (static_cast<unsigned int>(gain[idxBlue] * (bayer[1]     + bayer[width * 2 + 1])) + DIV_2_ROUNDING_OFF) / 2;   // blue or red
                bgr[0]      =  static_cast<unsigned int>(gain[1]       * bayer[width + 1]);                                                  // Green
                bgr[blue]   = (static_cast<unsigned int>(gain[idxRed]  * (bayer[width] + bayer[width + 2]))     + DIV_2_ROUNDING_OFF) / 2;   // red or blue
            }
            else {
                // Red 또는 Blue 픽셀에 대한 BGR 계산
                bgr[-blue]  = (static_cast<unsigned int>(gain[idxBlue] * (bayer[0] + bayer[2]     + bayer[width * 2] + bayer[width * 2 + 2])) + DIV_4_ROUNDING_OFF) / 4; // blue or red
                bgr[0]      = (static_cast<unsigned int>(gain[1]       * (bayer[1] + bayer[width] + bayer[width + 2] + bayer[width * 2 + 1])) + DIV_4_ROUNDING_OFF) / 4; // Green
                bgr[blue]   =  static_cast<unsigned int>(gain[idxRed]  * bayer[width + 1]);                                                                              // red or blue
            }

#else
            if( at_green ) {
                // Green 픽셀에 대한 BGR 계산
                bgr[-blue]  = (bayer[1]     + bayer[width * 2 + 1] + DIV_2_ROUNDING_OFF) / 2;   // blue or red
                bgr[0]      = bayer[width + 1];                                                 // Green
                bgr[blue]   = (bayer[width] + bayer[width + 2]     + DIV_2_ROUNDING_OFF) / 2;   // red or blue
            }
            else {
                // Red 또는 Blue 픽셀에 대한 BGR 계산
                bgr[-blue]  = (bayer[0] + bayer[2]     + bayer[width * 2] + bayer[width * 2 + 2] + DIV_4_ROUNDING_OFF) / 4; // blue or red
                bgr[0]      = (bayer[1] + bayer[width] + bayer[width + 2] + bayer[width * 2 + 1] + DIV_4_ROUNDING_OFF) / 4; // Green
                bgr[blue]   = bayer[width + 1];                                                                             // red or blue
            }
#endif

            // switch green status
            at_green = !at_green;

            // Next Pixel
            bayer++;
            bgr += BGR_PIXEL_WIDTH;
        }

        // switch green status
        at_green = !at_green;

        // switch blue status
        blue = -blue;

        /*
         * Next Line
         */
        bayer += BAYER_PATTERN_WIDTH;
        bgr   += BAYER_PATTERN_WIDTH*BGR_PIXEL_WIDTH;
    }
}
