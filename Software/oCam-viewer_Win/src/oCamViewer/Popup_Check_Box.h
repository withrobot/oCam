#pragma once


// Popup_Check_Box 대화 상자

class Popup_Check_Box : public CDialogEx
{
	DECLARE_DYNAMIC(Popup_Check_Box)

public:
	Popup_Check_Box(CWnd* pParent = nullptr);   // 표준 생성자입니다.
	virtual ~Popup_Check_Box();

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_POPUP_CHECK_BOX };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()
};
