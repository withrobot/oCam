// Popup_Window_Box.cpp: 구현 파일
//

#include "pch.h"
#include "Popup_Window_Box.h"
#include "afxdialogex.h"
#include "OCamViewer.h"

// Popup_Window_Box 대화 상자

IMPLEMENT_DYNAMIC(Popup_Window_Box, CDialogEx)

Popup_Window_Box::Popup_Window_Box(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_POPUP_WINDOW_BOX, pParent)
{
	//m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

Popup_Window_Box::~Popup_Window_Box()
{
}

void Popup_Window_Box::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(Popup_Window_Box, CDialogEx)
	ON_WM_CTLCOLOR()
	ON_WM_SETCURSOR()
	ON_STN_CLICKED(IDC_FW_Version_KR, &Popup_Window_Box::OnStnClickedFwVersionKR)
	ON_STN_CLICKED(IDC_FW_Version_EN, &Popup_Window_Box::OnStnClickedFwVersionEN)
END_MESSAGE_MAP()


// Popup_Window_Box 메시지 처리기

//fungofljm 210128 팝업창 처리
BOOL Popup_Window_Box::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  여기에 추가 초기화 작업을 추가합니다.
	int extendedStyle = GetWindowLong(m_hWnd, GWL_EXSTYLE);
	SetWindowLong(m_hWnd, GWL_EXSTYLE, extendedStyle | WS_EX_DLGMODALFRAME);

	LOGFONT logFont;
	GetFont()->GetLogFont(&logFont);
	logFont.lfUnderline = TRUE;
	m_font.CreateFontIndirect(&logFont);


	CRect rect;
	GetDlgItem(IDC_FW_Version_KR)->SetFont(&m_font);
	GetDlgItem(IDC_FW_Version_KR)->GetWindowRect(&rect);
	ScreenToClient(&rect);
	InvalidateRect(rect);

	//GetDlgItem(IDC_FW_Version_KR)->SetWindowText(_T("이 링크"));

	GetDlgItem(IDC_FW_Version_EN)->SetFont(&m_font);
	GetDlgItem(IDC_FW_Version_EN)->GetWindowRect(&rect);
	ScreenToClient(&rect);
	InvalidateRect(rect);
	//GetDlgItem(IDC_FW_Version_EN)->SetWindowText(_T("here."));
	return TRUE;  // return TRUE unless you set the focus to a control
				  // 예외: OCX 속성 페이지는 FALSE를 반환해야 합니다.
}

HBRUSH Popup_Window_Box::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = CDialogEx::OnCtlColor(pDC, pWnd, nCtlColor);

	// TODO:  여기서 DC의 특성을 변경합니다.
	if (pWnd->GetDlgCtrlID() == IDC_FW_Version_KR)
	{
		if (m_clicked == FALSE)
			pDC->SetTextColor(RGB(0, 0, 255));
		else
			pDC->SetTextColor(RGB(97, 28, 161));

		pDC->SetBkMode(TRANSPARENT);
		return (HBRUSH)GetStockObject(NULL_BRUSH);
	}

	if (pWnd->GetDlgCtrlID() == IDC_FW_Version_EN)
	{
		if (m_clicked == FALSE)
			pDC->SetTextColor(RGB(0, 0, 255));
		else
			pDC->SetTextColor(RGB(97, 28, 161));

		pDC->SetBkMode(TRANSPARENT);
		return (HBRUSH)GetStockObject(NULL_BRUSH);
	}

	// TODO:  기본값이 적당하지 않으면 다른 브러시를 반환합니다.
	return hbr;
}

BOOL Popup_Window_Box::OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	CPoint pt;
	CRect rect;

	GetCursorPos(&pt);
	GetDlgItem(IDC_FW_Version_KR)->GetWindowRect(rect);
	if (rect.PtInRect(pt))
	{
		SetCursor(AfxGetApp()->LoadStandardCursor(MAKEINTRESOURCE(IDC_HAND)));
		return TRUE;
	}

	GetDlgItem(IDC_FW_Version_EN)->GetWindowRect(rect);
	if (rect.PtInRect(pt))
	{
		SetCursor(AfxGetApp()->LoadStandardCursor(MAKEINTRESOURCE(IDC_HAND)));
		return TRUE;
	}
	return CDialogEx::OnSetCursor(pWnd, nHitTest, message);
	{
		return TRUE;

	}
}

void Popup_Window_Box::OnStnClickedFwVersionKR()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_clicked = TRUE;
	GetDlgItem(IDC_FW_Version_KR)->Invalidate();
	ShellExecute(NULL, "open", "http://withrobot.com/data/?uid=691&mod=document&pageid=1", NULL, NULL, SW_SHOWNORMAL);
}


void Popup_Window_Box::OnStnClickedFwVersionEN()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_clicked = TRUE;
	GetDlgItem(IDC_FW_Version_EN)->Invalidate();
	ShellExecute(NULL, "open", "http://withrobot.com/en/technical-data/?uid=87&mod=document&pageid=1", NULL, NULL, SW_SHOWNORMAL);
}
