#pragma once


// Popup_Window_Box 대화 상자

class Popup_Window_Box : public CDialogEx
{
	DECLARE_DYNAMIC(Popup_Window_Box)

public:
	Popup_Window_Box(CWnd* pParent = nullptr);   // 표준 생성자입니다.
	virtual ~Popup_Window_Box();

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_POPUP_WINDOW_BOX };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()
public:
	//HICON m_hIcon;
	BOOL m_clicked = false;
	CFont m_font;

	virtual BOOL OnInitDialog();
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg BOOL OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message);
	afx_msg void OnStnClickedFwVersionKR();
	afx_msg void OnStnClickedFwVersionEN();
};
