
// OCamViewerDlg.h : header file
//

#pragma once

#define MESSAGE_INCREASE_COUNT WM_USER

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
	void		CopyImage(void* Data);

	// Dialog Data
	enum { IDD = IDD_OCAMVIEWER_DIALOG };

	CDlgCamCtrl	m_DlgCamCtrl;

	CAMPTR		m_pCam;
	CString		m_CamModel;
	CString		m_CamSN;
	CString		m_UsbType;
	CString		m_FW;

	int			m_Blue_Gain;
	int			m_Red_Gain;
	int			m_CamSel;
	int			m_CamID;
	int			m_Width;
	int			m_Height;
	int			m_Count;
	int			m_WinVersion;
	double		m_FPS;
	double		m_CurrFPS;
	DWORD		m_StartTime;

	CComboBox	m_cbCam;
	CComboBox	m_cbResolution;

	wImage		m_Image;
	wImage		m_ImageSrc;
	wImage		m_IrImage;
	wImage		m_streoImage;
	wDisplay	m_Display;

	bool			m_WinVersion_flag = false;
	bool			m_connect_flag = false;
	bool			m_Fwcheck_flag = false;
	bool			m_record_flag = false;
	unsigned short	m_bcdDevice;
	unsigned short	m_vid;

	afx_msg void OnTimer(UINT_PTR nIDEvent);
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
	afx_msg LRESULT CallbackProc(WPARAM wParam, LPARAM lParam);
	virtual LRESULT DefWindowProc(UINT message, WPARAM wParam, LPARAM lParam);
	afx_msg void OnBnClickedButtonRecording();
	afx_msg void YUV_alloc(int WH_len);
	afx_msg void OnBnClickedButtonExit();
	afx_msg void Popup_Check_box();
	afx_msg void Popup_Window_box();
};