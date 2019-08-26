#pragma once

/*
* calculate norm of RGB image
*/
#include <cmath>
void calNormOfImage(double* normList, unsigned char* rgbImg, const int width, const int height)
{
	int dataLength = width * height;

	double normRed = 0;
	double normGreen = 0;
	double normBlue = 0;

	double maxY = 0.0;

	for (int i = 0; i < dataLength; i += 3) {
		double valueY = 0.257*rgbImg[i] + 0.574*rgbImg[i + 1] + 0.098*rgbImg[i + 2] + 16;

		if (valueY > 200) {
			maxY = (maxY < valueY) ? valueY : maxY;
			continue;
		}

		normRed += static_cast<double>(rgbImg[i]) * static_cast<double>(rgbImg[i]);
		normGreen += static_cast<double>(rgbImg[i + 1]) * static_cast<double>(rgbImg[i + 1]);
		normBlue += static_cast<double>(rgbImg[i + 2]) * static_cast<double>(rgbImg[i + 2]);
	}

	normList[2] = std::sqrt(normRed);
	normList[1] = std::sqrt(normGreen);
	normList[0] = std::sqrt(normBlue);
}