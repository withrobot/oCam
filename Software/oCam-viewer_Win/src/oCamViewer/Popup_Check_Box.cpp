// Popup_Check_Box.cpp: 구현 파일
//

#include "pch.h"
#include "Popup_Check_Box.h"
#include "afxdialogex.h"
#include "resource.h"

// Popup_Check_Box 대화 상자

IMPLEMENT_DYNAMIC(Popup_Check_Box, CDialogEx)

Popup_Check_Box::Popup_Check_Box(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_POPUP_CHECK_BOX, pParent)
{
	
}

Popup_Check_Box::~Popup_Check_Box()
{
}

void Popup_Check_Box::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(Popup_Check_Box, CDialogEx)
END_MESSAGE_MAP()


// Popup_Check_Box 메시지 처리기
