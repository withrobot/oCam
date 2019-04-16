#pragma once
#include "OCamViewer.h"
#include "libCamCap.h"
#include "afxwin.h"

// CDlgCamCtrl dialog

class CDlgCamCtrl : public CDialog
{
	DECLARE_DYNAMIC(CDlgCamCtrl)

	int width;
	int height;

public:
	CDlgCamCtrl(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDlgCamCtrl();
	
	void UpdateCamCtrl(CAMPTR pCam, int imgWidth, int imgHeight);
	void control_command(unsigned int value);
	void oCam2WRS_WDR_Ctrl(unsigned int value);

// Dialog Data
    CAMPTR		ptrCam;

	CSliderCtrl m_scBrightness;
	CSliderCtrl m_scContrast;
	CSliderCtrl m_scHue;
	CSliderCtrl m_scSaturation;
	CSliderCtrl m_scExposure;
	CSliderCtrl m_scGain;
	CSliderCtrl m_scWbBlue;
	CSliderCtrl m_scWbRed;

	enum { IDD = IDD_DLGCAMCTRL };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnBnClickedButtonCamCcDefault();
	afx_msg void OnBnClickedButtonCamCcReset();
	afx_msg void OnBnClickedButtonCamColorCorrection();
	afx_msg void OnBnClickedCheck1();
	CButton check_WDR_ctrl;
	afx_msg void OnBnClickedCheck2();
	CButton check_IR_ctrl;
};
