// DlgCamCtrl.cpp : implementation file
//

#include "stdafx.h"
#include "DlgCamCtrl.h"
#include "afxdialogex.h"

#include "wImage.h"
#include "ImgProc.h"
#include "colorCorrection.hpp"

#include <math.h>
#include <cmath>

#ifdef _DEBUG
#define DBG_PRINTF(...) wprintf(__VA_ARGS__);
#else
#define DBG_PRINTF(...)
#endif

extern int g_2WRS_Flag;
extern int g_4IRO_Flag;
extern char g_IR_check;

//konan91 1CGN, 18CRN 구분 flag 추가 20190315
extern int g_1CGN_Flag;
extern int g_18CRN_Flag;
extern int g_1CGNS_Flag;

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
	DDX_Control(pDX, IDC_SLIDER_GAIN, m_scGain);
	DDX_Control(pDX, IDC_SLIDER_WB_BLUE, m_scWbBlue);
	DDX_Control(pDX, IDC_SLIDER_WB_RED, m_scWbRed);
	DDX_Control(pDX, IDC_CHECK1, check_WDR_ctrl);
	DDX_Control(pDX, IDC_CHECK2, check_IR_ctrl);
}


BEGIN_MESSAGE_MAP(CDlgCamCtrl, CDialog)
	ON_WM_HSCROLL()
	ON_BN_CLICKED(IDC_BUTTON_CAM_CC_DEFAULT, &CDlgCamCtrl::OnBnClickedButtonCamCcDefault)
	ON_BN_CLICKED(IDC_BUTTON_CAM_CC_RESET, &CDlgCamCtrl::OnBnClickedButtonCamCcReset)
	ON_BN_CLICKED(IDC_BUTTON_CAM_COLOR_CORRECTION, &CDlgCamCtrl::OnBnClickedButtonCamColorCorrection)
	ON_BN_CLICKED(IDC_CHECK1, &CDlgCamCtrl::OnBnClickedCheck1)
	ON_BN_CLICKED(IDC_CHECK2, &CDlgCamCtrl::OnBnClickedCheck2)
END_MESSAGE_MAP()


// CDlgCamCtrl message handlers
void CDlgCamCtrl::UpdateCamCtrl(CAMPTR pCam, int imgWidth, int imgHeight)
{
	ptrCam = pCam;
	width = imgWidth;
	height = imgHeight;

	long min_value = 0;
	long max_value = 0;
	long pos = 0;
	uint8_t Blue_Gain = 0;
	uint8_t Red_Gain = 0;

	GetDlgItem(IDC_SLIDER_BRIGHTNESS)->EnableWindow(FALSE);
	GetDlgItem(IDC_SLIDER_CONTRAST)->EnableWindow(FALSE);
	GetDlgItem(IDC_SLIDER_HUE)->EnableWindow(FALSE);
	GetDlgItem(IDC_SLIDER_SATURATION)->EnableWindow(FALSE);
	GetDlgItem(IDC_SLIDER_EXPOSURE)->EnableWindow(FALSE);
	GetDlgItem(IDC_SLIDER_GAIN)->EnableWindow(FALSE);
	GetDlgItem(IDC_SLIDER_WB_BLUE)->EnableWindow(FALSE);
	GetDlgItem(IDC_SLIDER_WB_RED)->EnableWindow(FALSE);
	GetDlgItem(IDC_BUTTON_CAM_CC_DEFAULT)->EnableWindow(FALSE);
	GetDlgItem(IDC_BUTTON_CAM_CC_RESET)->EnableWindow(FALSE);
	GetDlgItem(IDC_BUTTON_CAM_COLOR_CORRECTION)->EnableWindow(FALSE);
	GetDlgItem(IDC_CHECK1)->EnableWindow(FALSE);
	GetDlgItem(IDC_CHECK2)->EnableWindow(FALSE);

	if (g_2WRS_Flag == 1)
	{
		CamGetCtrl(ptrCam, CTRL_BACKLIGHTCOMPENSATION, &pos);
		GetDlgItem(IDC_CHECK1)->EnableWindow(TRUE);
		check_WDR_ctrl.SetCheck(pos);
	}
	if (g_4IRO_Flag == 1) {

		GetDlgItem(IDC_CHECK2)->EnableWindow(TRUE);
		check_IR_ctrl.SetCheck(0);
	}

	if (CamGetCtrlRange(ptrCam, CTRL_BRIGHTNESS, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_BRIGHTNESS, &pos);
		GetDlgItem(IDC_SLIDER_BRIGHTNESS)->EnableWindow(TRUE);
		m_scBrightness.SetRange(min_value, max_value, TRUE);
		m_scBrightness.SetPos(pos);
		SetDlgItemInt(IDC_STATIC_BRIGHTNESS, pos);
	}

	if (CamGetCtrlRange(ptrCam, CTRL_CONTRAST, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_CONTRAST, &pos);
		GetDlgItem(IDC_SLIDER_CONTRAST)->EnableWindow(TRUE);
		m_scContrast.SetRange(min_value, max_value, TRUE);
		m_scContrast.SetPos(pos);
		SetDlgItemInt(IDC_STATIC_CONTRAST, pos);
	}

	if (CamGetCtrlRange(ptrCam, CTRL_HUE, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_HUE, &pos);
		GetDlgItem(IDC_SLIDER_HUE)->EnableWindow(TRUE);
		m_scHue.SetRange(min_value, max_value, TRUE);
		m_scHue.SetPos(pos);
		SetDlgItemInt(IDC_STATIC_HUE, pos);
	}

	if (CamGetCtrlRange(ptrCam, CTRL_SATURATION, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_SATURATION, &pos);
		GetDlgItem(IDC_SLIDER_SATURATION)->EnableWindow(TRUE);
		m_scSaturation.SetRange(min_value, max_value, TRUE);
		m_scSaturation.SetPos(pos);
		SetDlgItemInt(IDC_STATIC_SATURATION, pos);
	}

	if (CamGetCtrlRange(ptrCam, CTRL_EXPOSURE, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_EXPOSURE, &pos);
		GetDlgItem(IDC_SLIDER_EXPOSURE)->EnableWindow(TRUE);
		m_scExposure.SetRange(min_value, max_value, TRUE);
		m_scExposure.SetPos(pos);

		CString temp;
		temp.Format("%d=%.1fms", pos, pow(2.0, pos) * 1000);
		SetDlgItemText(IDC_STATIC_EXPOSURE, temp);
	}

	if (CamGetCtrlRange(ptrCam, CTRL_GAIN, &min_value, &max_value))
	{
		CamGetCtrl(ptrCam, CTRL_GAIN, &pos);
		GetDlgItem(IDC_SLIDER_GAIN)->EnableWindow(TRUE);
		m_scGain.SetRange(min_value, max_value, TRUE);
		m_scGain.SetPos(pos);
		SetDlgItemInt(IDC_STATIC_GAIN, pos);
	}

	//fungofljm WhiteBalance 수정 및 Windows os 버전 확인 추가 20201116
	if (m_WinVersion_Ctrl >= Windows_Version) {
		if (g_1CGNS_Flag == 1 || g_18CRN_Flag == 1 || g_1CGN_Flag == 1)
		{
			//Blue_Gain을 카메라에서 초기값 수신
			Blue_Gain = CamGetWhiteBalanceInfo(m_CamSel_Ctrl, INFO_BLUE_GAIN);
			GetDlgItem(IDC_SLIDER_WB_BLUE)->EnableWindow(TRUE);
			m_scWbBlue.SetRange(min_value, max_value, TRUE);
			m_scWbBlue.SetPos(Blue_Gain);
			SetDlgItemInt(IDC_STATIC_WB_BLUE, Blue_Gain);

			//Red_Gain을 카메라에서 초기값 수신
			Red_Gain = CamGetWhiteBalanceInfo(m_CamSel_Ctrl, INFO_RED_GAIN);
			GetDlgItem(IDC_SLIDER_WB_RED)->EnableWindow(TRUE);
			m_scWbRed.SetRange(min_value, max_value, TRUE);
			m_scWbRed.SetPos(Red_Gain);
			SetDlgItemInt(IDC_STATIC_WB_RED, Red_Gain);

			GetDlgItem(IDC_BUTTON_CAM_CC_DEFAULT)->EnableWindow(TRUE);
			GetDlgItem(IDC_BUTTON_CAM_CC_RESET)->EnableWindow(TRUE);
		}
	}

	else if (m_WinVersion_Ctrl < Windows_Version) {
		if (CamGetCtrlRange(ptrCam, CTRL_WHITEBALANCE_COMPONENT_BLUE, &min_value, &max_value))
		{
			CamGetCtrl(ptrCam, CTRL_WHITEBALANCE_COMPONENT_BLUE, &pos);
			GetDlgItem(IDC_SLIDER_WB_BLUE)->EnableWindow(TRUE);
			m_scWbBlue.SetRange(min_value, max_value, TRUE);
			m_scWbBlue.SetPos(pos);
			SetDlgItemInt(IDC_STATIC_WB_BLUE, pos);
		}

		if (CamGetCtrlRange(ptrCam, CTRL_WHITEBALANCE_COMPONENT_RED, &min_value, &max_value))
		{
			CamGetCtrl(ptrCam, CTRL_WHITEBALANCE_COMPONENT_RED, &pos);
			GetDlgItem(IDC_SLIDER_WB_RED)->EnableWindow(TRUE);
			m_scWbRed.SetRange(min_value, max_value, TRUE);
			m_scWbRed.SetPos(pos);
			SetDlgItemInt(IDC_STATIC_WB_RED, pos);

			/* 여기에서 1CGN 임을 판단하여 컬러 보정 요소 버튼 활성화 */
			if (g_1CGN_Flag == 1 || g_18CRN_Flag == 1) {
				GetDlgItem(IDC_BUTTON_CAM_CC_DEFAULT)->EnableWindow(TRUE);
				GetDlgItem(IDC_BUTTON_CAM_CC_RESET)->EnableWindow(TRUE);
			}
			//oCamS-1CGN의 경우 해당 값으로 처음에 color를 고정시킴
			else if (g_1CGNS_Flag == 1) {
				GetDlgItem(IDC_BUTTON_CAM_CC_DEFAULT)->EnableWindow(TRUE);
				GetDlgItem(IDC_BUTTON_CAM_CC_RESET)->EnableWindow(TRUE);
				CamSetCtrl(ptrCam, CTRL_GAIN, 100);
				CamSetCtrl(ptrCam, CTRL_WHITEBALANCE_COMPONENT_BLUE, 180);
				CamSetCtrl(ptrCam, CTRL_WHITEBALANCE_COMPONENT_RED, 145);
				SetDlgItemInt(IDC_STATIC_GAIN, 100);
				SetDlgItemInt(IDC_STATIC_WB_BLUE, 180);
				SetDlgItemInt(IDC_STATIC_WB_RED, 145);
			}
		}
	}
}


void CDlgCamCtrl::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	// TODO: Add your message handler code here and/or call default
	int pos;

	if ((CSliderCtrl*)pScrollBar == &m_scBrightness)
	{
		pos = m_scBrightness.GetPos();
		CamSetCtrl(ptrCam, CTRL_BRIGHTNESS, pos);
		SetDlgItemInt(IDC_STATIC_BRIGHTNESS, pos);
	}

	if ((CSliderCtrl*)pScrollBar == &m_scContrast)
	{
		pos = m_scContrast.GetPos();
		CamSetCtrl(ptrCam, CTRL_CONTRAST, pos);
		SetDlgItemInt(IDC_STATIC_CONTRAST, pos);
	}

	if ((CSliderCtrl*)pScrollBar == &m_scHue)
	{
		pos = m_scHue.GetPos();
		CamSetCtrl(ptrCam, CTRL_HUE, pos);
		SetDlgItemInt(IDC_STATIC_HUE, pos);
	}

	if ((CSliderCtrl*)pScrollBar == &m_scSaturation)
	{
		pos = m_scSaturation.GetPos();
		CamSetCtrl(ptrCam, CTRL_SATURATION, pos);
		SetDlgItemInt(IDC_STATIC_SATURATION, pos);
	}

	if ((CSliderCtrl*)pScrollBar == &m_scExposure)
	{
		pos = m_scExposure.GetPos();
		CamSetCtrl(ptrCam, CTRL_EXPOSURE, pos);
		CString temp;
		temp.Format("%d=%.1fms", pos, pow(2.0, pos) * 1000);
		SetDlgItemText(IDC_STATIC_EXPOSURE, temp);
	}

	if ((CSliderCtrl*)pScrollBar == &m_scGain)
	{
		pos = m_scGain.GetPos();
		CamSetCtrl(ptrCam, CTRL_GAIN, pos);
		SetDlgItemInt(IDC_STATIC_GAIN, pos);
	}

	//fungofljm 20201028 WhiteBalace 수정(Blue, Red)
	if ((CSliderCtrl*)pScrollBar == &m_scWbBlue)
	{
		pos = m_scWbBlue.GetPos();
		SetDlgItemInt(IDC_STATIC_WB_BLUE, pos);
		
		if (m_WinVersion_Ctrl >= Windows_Version) {
			//CamSetCtrl(ptrCam, CTRL_WHITEBALANCE_COMPONENT_BLUE, pos);
			CamSetWhiteBalanceInfo(m_CamSel_Ctrl, CTRL_BLUE_GAIN + pos);
		}
		else if (m_WinVersion_Ctrl < Windows_Version) {
			CamSetCtrl(ptrCam, CTRL_WHITEBALANCE_COMPONENT_BLUE, pos);
		}
	}

	if ((CSliderCtrl*)pScrollBar == &m_scWbRed)
	{
		pos = m_scWbRed.GetPos();
		SetDlgItemInt(IDC_STATIC_WB_RED, pos);
		
		if (m_WinVersion_Ctrl >= Windows_Version) {
			//CamSetCtrl(ptrCam, CTRL_WHITEBALANCE_COMPONENT_RED, pos);
			CamSetWhiteBalanceInfo(m_CamSel_Ctrl, CTRL_RED_GAIN + pos);
		}
		else if (m_WinVersion_Ctrl < Windows_Version) {
			CamSetCtrl(ptrCam, CTRL_WHITEBALANCE_COMPONENT_RED, pos);
		}
	}
	/* White Balance Red/Blue setting */


	CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
}

/*
*  Color correction for 1CGN(Bayer RGB pattern)
*/
//#define FLOAT_ORDER     2   // 소수점 두 째 자리까지만 고려한다.
#define FLOAT_SCALER    100 // 소수점을 UVC 인터페이스에 설정할 수 없으므로, 소수점 두째자리까지 정수로 변환하기 위한 값//
#define SAVE_SCALE      0x73617665  // scale 저장명령(ASCII:save)
#define ERASE_SCALE     0x78787878  // scale 지우기명령
#define LOAD_SCALE      0x89898989  // scale 불러오기명령
#define ERASE_FIRMWARE  0x30303030  // 펌웨어 지우기명령

static const int DEFAULT_EXPOSURE = -6;
static const int DEFAULT_GAIN = 128;
static const int DEFAULT_WB_COMP = 100;

void CDlgCamCtrl::OnBnClickedButtonCamCcDefault()
{
	/* White Balance Red/Blue setting & slider update*/
	m_scExposure.SetPos(DEFAULT_EXPOSURE);
	CamSetCtrl(ptrCam, CTRL_EXPOSURE, DEFAULT_EXPOSURE);
	CString temp;
	temp.Format("%d=%.1fms", DEFAULT_EXPOSURE, pow(2.0, DEFAULT_EXPOSURE) * 1000);
	SetDlgItemText(IDC_STATIC_EXPOSURE, temp);

	m_scGain.SetPos(DEFAULT_GAIN);
	CamSetCtrl(ptrCam, CTRL_GAIN, DEFAULT_GAIN);
	SetDlgItemInt(IDC_STATIC_GAIN, DEFAULT_GAIN);

	m_scWbBlue.SetPos(DEFAULT_WB_COMP);
	CamSetWhiteBalanceInfo(m_CamSel_Ctrl, CTRL_BLUE_GAIN + DEFAULT_WB_COMP);
	SetDlgItemInt(IDC_STATIC_WB_BLUE, DEFAULT_WB_COMP);

	m_scWbRed.SetPos(DEFAULT_WB_COMP);
	CamSetWhiteBalanceInfo(m_CamSel_Ctrl, CTRL_RED_GAIN + DEFAULT_WB_COMP);
	SetDlgItemInt(IDC_STATIC_WB_RED, DEFAULT_WB_COMP);
}


void CDlgCamCtrl::OnBnClickedButtonCamCcReset()
{

	DBG_PRINTF(L"reset_color_correction called!");

	OnBnClickedButtonCamCcDefault();
	/*
	* erase scale trigger
	*/
	control_command(ERASE_SCALE);

	/*
	* load scale trigger
	*/
	control_command(LOAD_SCALE);

	// 껏다키는방법
	CamStop(ptrCam);
	CamStart(ptrCam);
	CamSetCtrl(ptrCam, CTRL_GAIN, DEFAULT_GAIN);

	// save trigger 이후, Gain 값이 변했으므로 껐다가 켰을때 64로 시작 할 수 있도록
	
	GetDlgItem(IDC_BUTTON_CAM_COLOR_CORRECTION)->EnableWindow(TRUE);
}

//#ifndef _DEBUG // by SDKIM 20180220
//int round(double d)
//{
//	return static_cast<int>(d + 0.5);
//}
//#endif

void CDlgCamCtrl::OnBnClickedButtonCamColorCorrection()
{
	GetDlgItem(IDC_BUTTON_CAM_COLOR_CORRECTION)->EnableWindow(FALSE);

	DBG_PRINTF(L"calculate_color_correction called!");

	//wImage stereodst(width, height, MV_Y8);

	double normList[3];

	/*
	 *  calculate the white balance
	 */
	 //konan91 1CGN, 18CRN color correction 구분 20190325 
	 //fungofljm 1CGNS color correction 활성화 및 1CGN, 18CRN 과 color correction 구분 20201102
	if (g_1CGN_Flag == 1 || g_18CRN_Flag == 1) {
		wImage src(width, height, MV_Y8);
		wImage stereo_dst(width, height, MV_Y8);
		wImage dst(width, height, MV_RGB24);

		CamGetImage(ptrCam, (BYTE*)src.GetPtr1D());

		if (g_1CGN_Flag == 1) {
			Bayer2RGB((char*)src.GetPtr1D(), (char*)dst.GetPtr1D(), width, height, BayerGR2RGB);
		}

		else if (g_18CRN_Flag == 1) {
			Bayer2RGB((char*)src.GetPtr1D(), (char*)dst.GetPtr1D(), width, height, BayerGB2RGB);
		}

		calNormOfImage(normList, (unsigned char*)dst.GetPtr1D(), width, height);
	}

	else if (g_1CGNS_Flag == 1) {
		wImage src(width * 2, height, MV_Y8);
		wImage stereo_dst(width * 2, height, MV_Y8);
		wImage dst(width * 2, height, MV_RGB24);

		CamGetImage(ptrCam, (BYTE*)src.GetPtr1D());

		SplitImage((char*)src.GetPtr1D(), (char*)stereo_dst.GetPtr1D(), width, height);
		Bayer2RGB((char*)stereo_dst.GetPtr1D(), (char*)dst.GetPtr1D(), width * 2, height, BayerGR2RGB);

		calNormOfImage(normList, (unsigned char*)dst.GetPtr1D(), width, height);
	}

	double scaleRed = normList[1] / normList[0];
	double scaleBlue = normList[1] / normList[2];

	int settingValueRed = static_cast<int>(round(scaleRed * FLOAT_SCALER));
	int settingValueBlue = static_cast<int>(round(scaleBlue * FLOAT_SCALER));

	DBG_PRINTF(L"scaleRed: %f(%d), scaleBlue: %f(%d)\n", scaleRed, settingValueRed, scaleBlue, settingValueBlue);

	/* White Balance Red/Blue setting */
	m_scWbBlue.SetPos(settingValueBlue);
	CamSetCtrl(ptrCam, CTRL_WHITEBALANCE_COMPONENT_BLUE, settingValueBlue);
	if (m_WinVersion_Ctrl >= Windows_Version)
		CamSetWhiteBalanceInfo(m_CamSel_Ctrl, CTRL_BLUE_GAIN + settingValueBlue);
	SetDlgItemInt(IDC_STATIC_WB_BLUE, settingValueBlue);

	m_scWbRed.SetPos(settingValueRed);
	CamSetCtrl(ptrCam, CTRL_WHITEBALANCE_COMPONENT_RED, settingValueRed);
	if (m_WinVersion_Ctrl >= Windows_Version)
		CamSetWhiteBalanceInfo(m_CamSel_Ctrl, CTRL_RED_GAIN + settingValueRed);
	SetDlgItemInt(IDC_STATIC_WB_RED, settingValueRed);

	///*
	//* save trigger
	//*/
	control_command(SAVE_SCALE);
	//// 껏다키는방법
	CamStop(ptrCam);
	CamStart(ptrCam);

	// save trigger 이후, Gain 값이 변했으므로 껐다가 켰을때 64로 시작 할 수 있도록
	m_scGain.SetPos(DEFAULT_GAIN);
	CamSetCtrl(ptrCam, CTRL_GAIN, DEFAULT_GAIN);
	SetDlgItemInt(IDC_STATIC_GAIN, DEFAULT_GAIN);
}

void CDlgCamCtrl::control_command(unsigned int value)
{
	unsigned char cmd[8];

	cmd[0] = (value >> 28) & 0xf;
	cmd[1] = (value >> 24) & 0xf;
	cmd[2] = (value >> 20) & 0xf;
	cmd[3] = (value >> 16) & 0xf;
	cmd[4] = (value >> 12) & 0xf;
	cmd[5] = (value >> 8) & 0xf;
	cmd[6] = (value >> 4) & 0xf;
	cmd[7] = (value >> 0) & 0xf;

	DBG_PRINTF(L"ControlCommand=0x%X%X%X%X%X%X%X%X\n", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7]);

	CamSetCtrl(ptrCam, CTRL_GAIN, cmd[0]);
	CamSetCtrl(ptrCam, CTRL_GAIN, cmd[1]);
	CamSetCtrl(ptrCam, CTRL_GAIN, cmd[2]);
	CamSetCtrl(ptrCam, CTRL_GAIN, cmd[3]);
	CamSetCtrl(ptrCam, CTRL_GAIN, cmd[4]);
	CamSetCtrl(ptrCam, CTRL_GAIN, cmd[5]);
	CamSetCtrl(ptrCam, CTRL_GAIN, cmd[6]);
	CamSetCtrl(ptrCam, CTRL_GAIN, cmd[7]);
}

void CDlgCamCtrl::oCam2WRS_WDR_Ctrl(unsigned int value)
{
	unsigned char cmd[8];

	cmd[0] = (value >> 28) & 0xf;
	cmd[1] = (value >> 24) & 0xf;
	cmd[2] = (value >> 20) & 0xf;
	cmd[3] = (value >> 16) & 0xf;
	cmd[4] = (value >> 12) & 0xf;
	cmd[5] = (value >> 8) & 0xf;
	cmd[6] = (value >> 4) & 0xf;
	cmd[7] = (value >> 0) & 0xf;

	DBG_PRINTF(L"WRS_WDR_Ctrl=0x%X%X%X%X%X%X%X%X\n", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7]);

	CamSetCtrl(ptrCam, CTRL_BRIGHTNESS, cmd[0]);
	CamSetCtrl(ptrCam, CTRL_BRIGHTNESS, cmd[1]);
	CamSetCtrl(ptrCam, CTRL_BRIGHTNESS, cmd[2]);
	CamSetCtrl(ptrCam, CTRL_BRIGHTNESS, cmd[3]);
	CamSetCtrl(ptrCam, CTRL_BRIGHTNESS, cmd[4]);
	CamSetCtrl(ptrCam, CTRL_BRIGHTNESS, cmd[5]);
	CamSetCtrl(ptrCam, CTRL_BRIGHTNESS, cmd[6]);
	CamSetCtrl(ptrCam, CTRL_BRIGHTNESS, cmd[7]);
}

void CDlgCamCtrl::OnBnClickedCheck1()
{
	if (check_WDR_ctrl.GetCheck()) {
		CamSetCtrl(ptrCam, CTRL_BACKLIGHTCOMPENSATION, 1);
	}
	else {
		CamSetCtrl(ptrCam, CTRL_BACKLIGHTCOMPENSATION, 0);
	}
}


void CDlgCamCtrl::OnBnClickedCheck2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (check_IR_ctrl.GetCheck()) {
		g_IR_check = 1;
	}
	else {
		g_IR_check = 0;
	}

}
