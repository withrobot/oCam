
// OCamViewerDlg.h : header file
//

#pragma once

#include "wImage.h"
#include "wDisplay.h"
#include "DlgCamCtrl.h"
#include "libCamCap.h"

// COCamViewerDlg dialog
class COCamViewerDlg : public CDialogEx
{
// Construction
public:
	COCamViewerDlg(CWnd* pParent = NULL);	// standard constructor
	
    void		UpdateFPS();
    void		CopyImage(void *Data);

// Dialog Data
	enum { IDD = IDD_OCAMVIEWER_DIALOG };

	CDlgCamCtrl	m_DlgCamCtrl;

    CAMPTR		m_pCam;
	CString		m_CamModel;
	CString		m_CamSN;
	CString		m_UsbType;
	CString		m_FW;

	int			m_CamSel;
    int			m_CamID;
	int			m_Width;
	int			m_Height;
	int			m_Count;
    double		m_FPS;
    double		m_CurrFPS;
	DWORD		m_StartTime;

	CComboBox	m_cbCam;
	CComboBox	m_cbResolution;

    wImage		m_Image;
    wImage		m_ImageSrc;
    wDisplay	m_Display;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnBnClickedButtonCamCtrl();
	afx_msg void OnBnClickedButtonPlay();
	afx_msg void OnBnClickedButtonStop();
	afx_msg void OnCbnSelchangeComboCam();
	afx_msg void OnCbnSelchangeComboResolution();
	afx_msg void OnBnClickedButtonSaveImage();
	afx_msg void OnDestroy();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg LRESULT CallbackProc(WPARAM wParam, LPARAM lParam);
};
