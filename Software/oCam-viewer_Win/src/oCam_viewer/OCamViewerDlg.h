
// OCamViewerDlg.h : 헤더 파일
//

#pragma once

#include "wDisplay.h"
#include "wImage.h"
#include "afxwin.h"

#include "libCamCap.h"

// COCamViewerDlg 대화 상자
class COCamViewerDlg : public CDialogEx
{
// 생성입니다.
public:
	COCamViewerDlg(CWnd* pParent = NULL);	// 표준 생성자입니다.
	
    int			m_CamID;
	int			m_Width;
	int			m_Height;
    double		m_Fps;

	CComboBox	m_cbResolution;

    wImage		m_Image;
    wDisplay	m_Display;

    CAMPTR		ptrCam;

    void		CallbackProc(void* data);
    void		DisplayFPS();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_OCAMVIEWER_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.


// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonPlay();
	afx_msg void OnBnClickedButtonStop();
	afx_msg void OnCbnSelchangeComboResolution();
	afx_msg void OnBnClickedButtonSaveImage();
	afx_msg void OnDestroy();
};
