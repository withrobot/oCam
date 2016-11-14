// DlgCamCtrl.cpp : implementation file
//

#include "stdafx.h"
#include "DlgCamCtrl.h"
#include "afxdialogex.h"
#include <math.h>


// CDlgCamCtrl dialog

IMPLEMENT_DYNAMIC(CDlgCamCtrl, CDialog)

CDlgCamCtrl::CDlgCamCtrl(CWnd* pParent /*=NULL*/)
	: CDialog(CDlgCamCtrl::IDD, pParent)
{
	ptrCam = NULL;

    Create(CDlgCamCtrl::IDD, pParent);	// create dialog
}

CDlgCamCtrl::~CDlgCamCtrl()
{
}

void CDlgCamCtrl::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_SLIDER_BRIGHTNESS, m_scBrightness);
	DDX_Control(pDX, IDC_SLIDER_CONTRAST, m_scContrast);
	DDX_Control(pDX, IDC_SLIDER_HUE, m_scHue);
	DDX_Control(pDX, IDC_SLIDER_SATURATION, m_scSaturation);
	DDX_Control(pDX, IDC_SLIDER_EXPOSURE, m_scExposure);
}


BEGIN_MESSAGE_MAP(CDlgCamCtrl, CDialog)
	ON_WM_HSCROLL()
END_MESSAGE_MAP()


// CDlgCamCtrl message handlers


void CDlgCamCtrl::UpdateCamCtrl(CAMPTR pCam)
{
	ptrCam = pCam;

	long min_value, max_value, pos;

	(CWnd*)GetDlgItem(IDC_SLIDER_BRIGHTNESS)->EnableWindow(FALSE);
	(CWnd*)GetDlgItem(IDC_SLIDER_CONTRAST)->EnableWindow(FALSE);
	(CWnd*)GetDlgItem(IDC_SLIDER_HUE)->EnableWindow(FALSE);
	(CWnd*)GetDlgItem(IDC_SLIDER_SATURATION)->EnableWindow(FALSE);
	(CWnd*)GetDlgItem(IDC_SLIDER_EXPOSURE)->EnableWindow(FALSE);

	if (CamGetCtrlRange(ptrCam, CTRL_BRIGHTNESS, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_BRIGHTNESS, &pos);
		(CWnd*)GetDlgItem(IDC_SLIDER_BRIGHTNESS)->EnableWindow(TRUE);
		m_scBrightness.SetRange(min_value, max_value, TRUE);
		m_scBrightness.SetPos(pos);
		SetDlgItemInt(IDC_STATIC_BRIGHTNESS, pos);
	}

	if (CamGetCtrlRange(ptrCam, CTRL_CONTRAST, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_CONTRAST, &pos);
		(CWnd*)GetDlgItem(IDC_SLIDER_CONTRAST)->EnableWindow(TRUE);
		m_scContrast.SetRange(min_value, max_value, TRUE);
		m_scContrast.SetPos(pos);
		SetDlgItemInt(IDC_STATIC_CONTRAST, pos);
	}

	if (CamGetCtrlRange(ptrCam, CTRL_HUE, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_HUE, &pos);
		(CWnd*)GetDlgItem(IDC_SLIDER_HUE)->EnableWindow(TRUE);
		m_scHue.SetRange(min_value, max_value, TRUE);
		m_scHue.SetPos(pos);
		SetDlgItemInt(IDC_STATIC_HUE, pos);
	}

	if (CamGetCtrlRange(ptrCam, CTRL_SATURATION, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_SATURATION, &pos);
		(CWnd*)GetDlgItem(IDC_SLIDER_SATURATION)->EnableWindow(TRUE);
		m_scSaturation.SetRange(min_value, max_value, TRUE);
		m_scSaturation.SetPos(pos);
		SetDlgItemInt(IDC_STATIC_SATURATION, pos);
	}

	if (CamGetCtrlRange(ptrCam, CTRL_EXPOSURE, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_EXPOSURE, &pos);
		(CWnd*)GetDlgItem(IDC_SLIDER_EXPOSURE)->EnableWindow(TRUE);
		m_scExposure.SetRange(min_value, max_value, TRUE);
		m_scExposure.SetPos(pos);

		CString temp;
		temp.Format("%d=%.1fms", pos, pow(2.0,pos)*1000); 
		SetDlgItemText(IDC_STATIC_EXPOSURE, temp);
	}
}


void CDlgCamCtrl::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	// TODO: Add your message handler code here and/or call default
	int pos;

	if ((CSliderCtrl *)pScrollBar == &m_scBrightness)
	{
		pos = m_scBrightness.GetPos();
		CamSetCtrl(ptrCam, CTRL_BRIGHTNESS, pos);
		SetDlgItemInt(IDC_STATIC_BRIGHTNESS, pos);
	}

	if ((CSliderCtrl *)pScrollBar == &m_scContrast)
	{
		pos = m_scContrast.GetPos();
		CamSetCtrl(ptrCam, CTRL_CONTRAST, pos);
		SetDlgItemInt(IDC_STATIC_CONTRAST, pos);
	}

	if ((CSliderCtrl *)pScrollBar == &m_scHue)
	{
		pos = m_scHue.GetPos();
		CamSetCtrl(ptrCam, CTRL_HUE, pos);
		SetDlgItemInt(IDC_STATIC_HUE, pos);
	}

	if ((CSliderCtrl *)pScrollBar == &m_scSaturation)
	{
		pos = m_scSaturation.GetPos();
		CamSetCtrl(ptrCam, CTRL_SATURATION, pos);
		SetDlgItemInt(IDC_STATIC_SATURATION, pos);
	}

	if ((CSliderCtrl *)pScrollBar == &m_scExposure)
	{
		pos = m_scExposure.GetPos();
		CamSetCtrl(ptrCam, CTRL_EXPOSURE, pos);

		CString temp;
		temp.Format("%d=%.1fms", pos, pow(2.0,pos)*1000); 
		SetDlgItemText(IDC_STATIC_EXPOSURE, temp);
	}

	CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
}
