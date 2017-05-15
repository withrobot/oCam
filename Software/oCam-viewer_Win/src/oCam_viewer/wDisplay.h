///////////////////////////////////////////////////////////////////////////////////////
//
// wDisplay.h : header file
//
// 20070910 
//	Display에서 바로 화면에 출력하도록 변경하였다. (기존에는 OnPaint에서 출력하도록 하였다. )
//
// 앞으로 구현할 내용. 
// SetTitle(CString str) or (char *) 
//		: 타이틀바의 내용을 바꾼다
// void EnableRect(int Num, COLORREF Color = RGB(255,0,0))
//		: Num번째 Rect 입력을 받아들인다. 
// void GetRect(int Num, CRect *Rect);
//		: 사각형을 가져온다. 
// void SetID(int ID)
//		: ID를 설정한다. 윈도우 메세지가 올때 어떤 윈도우인지를 확인하기 위하여 사용 
// void SetWinHandle(HWND hWnd)
//		: 윈도우 핸들을 설정해 놓으면 LMOUSEBUTTON_DOWN, MOVE, UP 메세지를 보낸다. 
// 확대 축소가 가능하도록 ...
// 사각형은 wDisplay에서 그리고 윈도우 메세지로 알려준다. 
// 최종 위치를 기억했다가 다음번 출력에 최종 위치에 출력된다. 
//
// 20100715
//	출력화면은 최대 모니터 크기까지 밖에 커지지 않는다. 
//	그래서 모니터보다 큰 출력 화면은 계속 화면을 크게하려고 하는 바람에 
//	깜박임이 발생한다. 이문제를 수정함. 
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_WDISPLAY_H__5A853788_798B_4259_9073_179A9F572FD4__INCLUDED_)
#define AFX_WDISPLAY_H__5A853788_798B_4259_9073_179A9F572FD4__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "wImage.h"

class wDisplay : public CWnd
{
// Construction
public:
	wDisplay();

// Attributes
public:
	static int		Count;
	CDC				m_MemDC;
	LPBITMAPINFO	m_pBmpInfo;
	int				m_Width, m_Height;

// Operations
public:
	void Display(wImage& Img);
	void DrawRect(RECT Rect, COLORREF Color, int Num=-1);

	void SavePos(CWinApp* pApp, LPCTSTR Section="Display");
	void LoadPos(CWinApp* pApp, LPCTSTR Section="Display");
	void SavePos(char *File, LPCTSTR Section="Display");
	void LoadPos(char *File, LPCTSTR Section="Display");
	
// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(wDisplay)
	public:
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~wDisplay();

	// Generated message map functions
protected:
	//{{AFX_MSG(wDisplay)
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_WDISPLAY_H__5A853788_798B_4259_9073_179A9F572FD4__INCLUDED_)
