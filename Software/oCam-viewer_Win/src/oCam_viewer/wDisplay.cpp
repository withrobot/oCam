// wDisplay.cpp : implementation file
//

#include "stdafx.h"
#include "wDisplay.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// wDisplay
int wDisplay::Count;

wDisplay::wDisplay()
{
	CString class_name = AfxRegisterWndClass(CS_VREDRAW | CS_HREDRAW, 
							::LoadCursor(NULL, IDC_ARROW),
							(HBRUSH)::GetStockObject(WHITE_BRUSH), 
							::LoadIcon(NULL, IDI_INFORMATION));

//	CWnd* pWnd = AfxGetMainWnd();
	CWnd* pWnd = GetDesktopWindow();
//	WS_POPUPWINDOW |WS_VISIBLE|WS_CAPTION|WS_VISIBLE|

	char temp[128];
	sprintf_s(temp, "Display%d", Count++);

	CreateEx(0 ,   
			class_name,   
			temp, 
			WS_OVERLAPPED |WS_SYSMENU|WS_CAPTION|WS_VISIBLE|
//			WS_OVERLAPPED |WS_SYSMENU|WS_CAPTION|
			WS_MINIMIZEBOX|WS_THICKFRAME, 
			CW_USEDEFAULT,CW_USEDEFAULT,640,480, 
			pWnd->m_hWnd, 
			NULL,NULL);

	CMenu* pSysMenu = this->GetSystemMenu(FALSE); 
	pSysMenu->EnableMenuItem(SC_CLOSE, MF_BYCOMMAND | MF_DISABLED);

	// Init MemDC
	CDC *pDC = GetDC();
	m_MemDC.CreateCompatibleDC(pDC);
	ReleaseDC(pDC);

	// Bmp Information
	static BYTE ptr[sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD)*256];

	m_pBmpInfo = (LPBITMAPINFO)ptr;
	LPBITMAPINFOHEADER  pBmpInfoHdr = (LPBITMAPINFOHEADER)m_pBmpInfo;

	pBmpInfoHdr->biSize			= sizeof(BITMAPINFOHEADER);
	pBmpInfoHdr->biWidth			= 640;
	pBmpInfoHdr->biHeight			= 1;
	pBmpInfoHdr->biPlanes			= 1;
	pBmpInfoHdr->biBitCount			= 8*2;
	pBmpInfoHdr->biCompression		= BI_RGB;
	pBmpInfoHdr->biSizeImage		= 640*2;
	pBmpInfoHdr->biXPelsPerMeter	= 0;
	pBmpInfoHdr->biYPelsPerMeter	= 0;
	pBmpInfoHdr->biClrUsed 		= 256;
	pBmpInfoHdr->biClrImportant	= 256;

	for (int i=0; i<256; i++) 
	{
		m_pBmpInfo->bmiColors[i].rgbRed = i;
		m_pBmpInfo->bmiColors[i].rgbGreen = i;
		m_pBmpInfo->bmiColors[i].rgbBlue = i;
	}

	m_Width  = 0;
	m_Height = 0;
}

wDisplay::~wDisplay()
{
	DestroyWindow();
}


BEGIN_MESSAGE_MAP(wDisplay, CWnd)
	//{{AFX_MSG_MAP(wDisplay)
	ON_WM_PAINT()
	ON_WM_SIZE()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// wDisplay message handlers

/*
	pDisp->FactorX = 0;
	int j, size_x, size_y

	if (pDisp->FactorX >= 0)
		size_x = pBuf->SizeX*(pDisp->FactorX+1);
	else
		size_x = pBuf->SizeX/(abs(pDisp->FactorX)+1);

	if (pDisp->FactorY >= 0)
		size_y = pBuf->SizeX*(pDisp->FactorX+1);
	else
		size_y = 1;

	for (i=0,j=0; i<pBuf->SizeY; i++, j++) {
		::StretchDIBits(hDC,                    // hDC
			   0+pDisp->OffsetX,				// DestX
			   i+pDisp->OffsetY,				// DestY
			   size_x,			// nDestWidth
			   1,				// nDestHeight
			   0,				// SrcX
			   0,   			// SrcY
			   pBuf->SizeX,		// nSrcWidth
			   1,				// nSrcHeight
			   pBuf->LinAddr[i],// lpBits
			   pDisp->bmpInfo,	// lpBitsInfo
			   DIB_RGB_COLORS,	// wUsage
			   SRCCOPY);		// Rop
	}
*/
void wDisplay::Display(wImage& Img)
{
	int i, depth, type, width, height;
	width  = Img.GetWidth();
	height = Img.GetHeight();
	depth  = Img.GetDepth();
	type   = Img.GetType();

	if (type==MV_RGB16)
	{
		__int16 *img = (__int16 *)Img.GetPtr1D();
		__int16 data, r,g,b;
		for (int i=0; i<Img.GetWidth()*Img.GetHeight(); i++)
		{
			data = img[i];
			r = (data>>11)&0x1f;
			g = (data>>6)&0x1f;
			b = (data>>0)&0x1f;
			img[i] = (r<<10)|(g<<5)|b;
		}
	}
	else if (type==MV_YUV422)
	{
		height *= 2;
		depth  /= 2;
	}

	if ( (width!=0 && height!=0) &&
		 (width!=m_Width || height!=m_Height) )
	{
		m_Width  = width;
		m_Height = height;

		// 타이틀바의 크기를 구한다. 
		// 위젯에 따라 타이틀바의 크기가 다르기 때문에 고정된 값을 사용할 수 없다. 
		CRect rect;
		GetClientRect(&rect);

		CRect rectWin;
		GetWindowRect(&rectWin);
		int offset_x = rectWin.Width()-rect.Width();
		int offset_y = rectWin.Height()-rect.Height();
		
		SetWindowPos(&wndTop, 0, 0, width+offset_x, height+offset_y, SWP_NOMOVE|SWP_NOZORDER);
	}

	char *data = (char *)Img.GetPtr1D();

	LPBITMAPINFOHEADER  pBmpInfoHdr = (LPBITMAPINFOHEADER)m_pBmpInfo;
	pBmpInfoHdr->biWidth		= width;
	pBmpInfoHdr->biBitCount		= 8*depth;
	pBmpInfoHdr->biSizeImage	= width*depth;

	for (i=0; i<height; i++) 
	{
		::SetDIBitsToDevice(m_MemDC.m_hDC,
			   0,				// DestX
			   i,				// DestY
			   width,			// nDestWidth
			   1,					// nDestHeight
			   0,					// SrcX
			   0,   				// SrcY
			   0,					// nStartScan
			   (WORD)1,				// nNumScans
			   data+i*width*depth,	// lpBits
			   m_pBmpInfo,			// lpBitsInfo
			   DIB_RGB_COLORS);		// wUsage
	}

	CDC *pDC = GetDC();
	pDC->BitBlt(0, 0, width, height, &m_MemDC, 0, 0, SRCCOPY);
	ReleaseDC(pDC);
}

void wDisplay::DrawRect(RECT Rect, COLORREF Color, int Num)
{/*
	CPen Pen;
	CPen *pOldPen;

	Pen.CreatePen(PS_SOLID, 1, Color);
	pOldPen = (CPen *) m_MemDC.SelectObject(&Pen);
	m_MemDC.MoveTo(Rect.left, Rect.top);
	m_MemDC.LineTo(Rect.right, Rect.top);
	m_MemDC.LineTo(Rect.right, Rect.bottom);
	m_MemDC.LineTo(Rect.left, Rect.bottom);
	m_MemDC.LineTo(Rect.left, Rect.top);
	m_MemDC.SelectObject(pOldPen);
	Pen.DeleteObject();

	if (Num>=0)
	{
		char num[8];
		m_MemDC.TextOut(Rect.left, Rect.top, itoa(Num, num, 10));
	}


	int x,y;

	for (y=Rect.top+1; y<Rect.bottom; y++)
	{
		for (x=Rect.left+1; x<Rect.right; x++)
		{
			if (m_MemDC.GetPixel(x,y))
			{
				m_MemDC.SetPixel(x,y,RGB(0,255,0));
				break;
			}
		}
		for (x=Rect.right-1; x>Rect.left; x--)
		{
			if (m_MemDC.GetPixel(x,y))
			{
				m_MemDC.SetPixel(x,y,RGB(0,255,0));
				break;
			}
		}
	}

	for (x=Rect.left+1; x<Rect.right; x++)
	{
		for (y=Rect.top+1; y<Rect.bottom; y++)
		{
			if (m_MemDC.GetPixel(x,y))
			{
				m_MemDC.SetPixel(x,y,RGB(0,255,0));
				break;
			}
		}
		for (y=Rect.bottom-1; y>Rect.top; y--)
		{
			if (m_MemDC.GetPixel(x,y))
			{
				m_MemDC.SetPixel(x,y,RGB(0,255,0));
				break;
			}
		}
	}*/
}

void wDisplay::OnPaint() 
{
	CPaintDC dc(this); // device context for painting
	
	// TODO: Add your message handler code here
	CRect rect;
	GetClientRect(&rect);
	dc.BitBlt(0, 0, rect.Width(), rect.Height(), &m_MemDC, 0, 0, SRCCOPY);
}

void wDisplay::OnSize(UINT nType, int cx, int cy) 
{
	CWnd::OnSize(nType, cx, cy);
	
	// TODO: Add your message handler code here
	CDC *pDC = GetDC();
	CBitmap Bitmap;
	Bitmap.CreateCompatibleBitmap(pDC, cx, cy);
	m_MemDC.SelectObject(Bitmap);
	ReleaseDC(pDC);
}

void wDisplay::SavePos(CWinApp* pApp, LPCTSTR Section)
{
	RECT		rc;

	GetWindowRect(&rc );
	pApp->WriteProfileInt(Section, "top",	rc.top);
	pApp->WriteProfileInt(Section, "left",	rc.left);
	pApp->WriteProfileInt(Section, "bottom",rc.bottom);
	pApp->WriteProfileInt(Section, "right",	rc.right);
}

void wDisplay::LoadPos(CWinApp* pApp, LPCTSTR Section)
{
	RECT        rc;

	GetWindowRect( &rc );
	rc.top		= pApp->GetProfileInt(Section, "top",	rc.top);
	rc.left		= pApp->GetProfileInt(Section, "left",	rc.left);
	rc.bottom	= pApp->GetProfileInt(Section, "bottom",rc.bottom);
	rc.right	= pApp->GetProfileInt(Section, "right",	rc.right);

	MoveWindow (rc.left, rc.top, rc.right-rc.left, rc.bottom-rc.top, TRUE);
}

void wDisplay::SavePos(char *File, LPCTSTR Section)
{
	RECT rc;
	char pos[256];
	
	GetWindowRect(&rc );
	sprintf_s(pos, "(%d,%d,%d,%d)", rc.left, rc.top, rc.right, rc.bottom);	
	WritePrivateProfileString(Section, "Pos", pos, File);
}

void wDisplay::LoadPos(char *File, LPCTSTR Section)
{
	RECT rc;
	char pos[256];
	int n;

	GetPrivateProfileString	(Section, "Pos", "", pos, sizeof(pos), File);	
	n = sscanf_s(pos, "(%d,%d,%d,%d)", &rc.left, &rc.top, &rc.right, &rc.bottom);
	
	if (n==4)
		MoveWindow (rc.left, rc.top, rc.right-rc.left, rc.bottom-rc.top, TRUE);
}
