/*
 * ConvertColor.h
 *
 *  Created on: Sep 23, 2016
 *      Author: gnohead
 */

#ifndef CONVERTCOLOR_H_
#define CONVERTCOLOR_H_

#define  BayerBG2BGR 1
#define  BayerGB2BGR 2
#define  BayerRG2BGR 3
#define  BayerGR2BGR 4

#define  BayerBG2RGB BayerRG2BGR
#define  BayerGB2RGB BayerGR2BGR
#define  BayerRG2RGB BayerBG2BGR
#define  BayerGR2RGB BayerGB2BGR

void Bayer2BGR(unsigned char* bayer, unsigned char *bgr, int width, int height, int code, double gainRed=1.0, double gainGreen=1.0, double gainBlue=1.0);

#endif /* CONVERTCOLOR_H_ */
