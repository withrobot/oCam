#pragma once
#include "OCamViewer.h"
#include "libCamCap.h"

// CDlgCamCtrl dialog

class CDlgCamCtrl : public CDialog
{
	DECLARE_DYNAMIC(CDlgCamCtrl)

public:
	CDlgCamCtrl(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDlgCamCtrl();
	void	UpdateCamCtrl(CAMPTR pCam);

// Dialog Data
    CAMPTR		ptrCam;

	CSliderCtrl m_scBrightness;
	CSliderCtrl m_scContrast;
	CSliderCtrl m_scHue;
	CSliderCtrl m_scSaturation;
	CSliderCtrl m_scExposure;

	enum { IDD = IDD_DLGCAMCTRL };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
};
