// wImage.cpp: implementation of the wImage class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "wImage.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

wImage::wImage()
{
	Width = 0;
	Height = 0;
	Type = 0;
	ppBuf = NULL;
}

wImage::wImage(int inWidth, int inHeight, int inType)
{
	Width = 0;
	Height = 0;
	Type = 0;
	ppBuf = NULL;

	Alloc(inWidth, inHeight, inType);
}

wImage::~wImage()
{
	Free();
}

void wImage::operator=(wImage &rhs)
{
	Alloc(rhs.GetWidth(), rhs.GetHeight(), rhs.GetType());

	U8 *src = (U8 *)rhs.GetPtr1D();
	U8 *dst = (U8 *)GetPtr1D();

	memcpy(dst, src, GetSize());
}

int  wImage::operator==(wImage &rhs)
{
	if (Width==rhs.GetWidth() && Height==rhs.GetHeight() && Type==rhs.GetType())
		return 1;
	else
		return 0;
}

int  wImage::operator!=(wImage &rhs)
{
	if (Width==rhs.GetWidth() && Height==rhs.GetHeight() && Type==rhs.GetType())
		return 0;
	else
		return 1;
}

void wImage::Alloc(int inWidth, int inHeight, int inType)
{
	if (inWidth==Width && inHeight==Height && inType==Type)
		return;

	if (ppBuf)
		Free();

	Width = inWidth;
	Height = inHeight;
	Type = inType;
	int depth = GetDepth();

	U8 *ptr;
	ptr = new U8[Width*Height*depth];
	ppBuf = new void* [Height];

	for (int i=0; i<Height; i++)
		ppBuf[i] = (ptr+Width*i*depth);
}

void wImage::Alloc(wImage& Image, int Type)
{
	int width, height, type;

	width = Image.GetWidth();
	height = Image.GetHeight();

	if (Type)
		type = Type;
	else
		type = Image.GetType();

	Alloc(width, height, type);
}

void wImage::Free()
{
	if (!ppBuf)
		return;

	delete[] (U8 *)*ppBuf;
	delete[] ppBuf;

	Width = 0;
	Height = 0;
	Type = 0;
	ppBuf = NULL;
}

void *wImage::GetPtr2D() 
{
	return ppBuf;
}

void *wImage::GetPtr1D()
{
	if (ppBuf) 
		return ppBuf[0]; 
	else 
		return NULL;
}

#ifdef _WIN32

#include "wingdi.h"	//for BITMAPV4HEADER

int  wImage::Load(CString FileName)
{
	BITMAPFILEHEADER 	bmpHeader;			//bmp 화일의 헤더
	int y, len, bit, Type, width, height, clr_used;
	char** ptr, file_name[256];

	//화일을 연다.
	strcpy_s(file_name, FileName);
	FILE *fp;
	fopen_s(&fp, file_name, "rb");
	if(!fp) 
		return FALSE;

	//bmpHeader 구조체에 BMPHEADER 헤더 정보를 읽어 온다.  
	if (fread(&bmpHeader, sizeof(bmpHeader), 1, fp)==0)
		goto END;

	//화일이 bmp 일경우에는 화일 처음에 "BM"이라고 설정해있음
	if (bmpHeader.bfType != ((WORD)('M'<<8) | 'B')) 
		goto END;

	DWORD size;
	if (fread(&size, sizeof(size), 1, fp)==0)
		goto END;

	fseek(fp, -(int)sizeof(size), SEEK_CUR);
	if (size==sizeof(BITMAPINFOHEADER))
	{
		BITMAPINFOHEADER 	bmpInfoHeader;		//bmp 정보헤더
		fread(&bmpInfoHeader, sizeof(bmpInfoHeader), 1, fp);

		bit	   = bmpInfoHeader.biBitCount;
		width  = bmpInfoHeader.biWidth;
		height = bmpInfoHeader.biHeight;
		clr_used = bmpInfoHeader.biClrUsed;
	}
	else if (size==sizeof(BITMAPCOREHEADER))
	{
		BITMAPCOREHEADER 	bmpCoreHeader;		//bmp 정보헤더
		fread(&bmpCoreHeader, sizeof(bmpCoreHeader), 1, fp);

		bit	   = bmpCoreHeader.bcBitCount;
		width  = bmpCoreHeader.bcWidth;
		height = bmpCoreHeader.bcHeight;
		clr_used = 0;
	}
	else if (size==sizeof(BITMAPV4HEADER))
	{
		BITMAPV4HEADER		bmpV4Header;
	//	BITMAPV5HEADER		bmpV5Header;
		fread(&bmpV4Header, sizeof(bmpV4Header), 1, fp);

		bit	   = bmpV4Header.bV4BitCount;
		width  = bmpV4Header.bV4Width;
		height = bmpV4Header.bV4Height;
		clr_used = bmpV4Header.bV4ClrUsed;
	}


	if (bit==8)
		Type = MV_Y8;
	else if (bit==16)
		Type = MV_RGB15;
	else if (bit==24)
		Type = MV_RGB24;
	else if (bit==32)
		Type = MV_RGB32;
	else
		goto END;

	Alloc(width, height, Type);

	if (clr_used == 256) 
	{
		RGBQUAD bmiColors[256];
		fread(bmiColors, sizeof(RGBQUAD), 256, fp);
	}

	//read data
	ptr = (char **)GetPtr2D();
	len = Width*GetDepth();

	for (y=height-1; y>=0; y--) 
	{
		if (fread(ptr[y], len, 1, fp)==0)
			goto END;
	}

	fclose(fp);

	return TRUE;

END:
	fclose(fp);

	return FALSE;	
}

int  wImage::Save(CString FileName)
{
	BITMAPFILEHEADER 	bmpfh;			//bmp 화일의 헤더
	BITMAPINFOHEADER 	bmpInfoHeader;		//bmp 정보헤더
	int y, len, depth=GetDepth();
	char** ptr, file_name[256];

	bmpInfoHeader.biSize		= sizeof(BITMAPINFOHEADER);
	bmpInfoHeader.biWidth		= Width;
	bmpInfoHeader.biHeight		= Height;
	bmpInfoHeader.biPlanes		= 1;
	bmpInfoHeader.biBitCount	= 8*depth;
	bmpInfoHeader.biCompression = BI_RGB;
	bmpInfoHeader.biSizeImage	= Width*Height*depth;
	bmpInfoHeader.biXPelsPerMeter = 0;
	bmpInfoHeader.biYPelsPerMeter = 0;

	if (depth == 1) 
	{
		bmpInfoHeader.biClrUsed 		= 256;
		bmpInfoHeader.biClrImportant	= 256;
	}
	else 
	{
		bmpInfoHeader.biClrUsed 		= 0;
		bmpInfoHeader.biClrImportant	= 0;
	}

	bmpfh.bfType = ((WORD)('M'<<8) | 'B');
	if (depth == 1)
		bmpfh.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD)*256;
	else
		bmpfh.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
	bmpfh.bfSize = bmpfh.bfOffBits + bmpInfoHeader.biSizeImage;
	bmpfh.bfReserved1 = 0;
	bmpfh.bfReserved2 = 0;

	//화일을 연다.
	strcpy_s(file_name, FileName);
	FILE *fp;
	fopen_s(&fp, file_name,"wb");
	if(!fp) 
		return FALSE;

	//bmp file header를 쓴다. 
	fwrite(&bmpfh, sizeof(bmpfh), 1, fp);

	//bmpInfoHeader 구조체에 BMPINFOHEADER 헤더 정보를 쓴다. 
	fwrite(&bmpInfoHeader, sizeof(bmpInfoHeader), 1, fp);

	//8bit이면 팔레트 정보를 ...
	if (bmpInfoHeader.biBitCount==8) 
	{
		RGBQUAD bmiColors[256];
		for (int i=0; i<256; i++) 
		{
			bmiColors[i].rgbRed = i;
			bmiColors[i].rgbGreen = i;
			bmiColors[i].rgbBlue = i;
			bmiColors[i].rgbReserved = 0;
		}
		fwrite(bmiColors, sizeof(RGBQUAD), 256, fp);
	}

	//write data
	ptr = (char **)GetPtr2D();
	len = depth*Width;

	for (y=Height-1; y>=0; y--) 
	{
		fwrite(ptr[y], len, 1, fp);
	}

	fclose(fp);
	return TRUE;

//END:
//	fclose(fp);

	return FALSE;
}

#endif

