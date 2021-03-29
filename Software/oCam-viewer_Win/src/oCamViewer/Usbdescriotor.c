#include "Usbdescriotor.h"

VOID InitializeListHead(_Out_ PLIST_ENTRY ListHead)
{
	ListHead->Flink = ListHead->Blink = ListHead;
}

_Success_(return != NULL)
_Post_writable_byte_size_(dwBytes)
HGLOBAL MyAlloc(_In_ PCHAR File, ULONG Line, DWORD dwBytes);

LIST_ENTRY AllocListHead =
{
	&AllocListHead,
	&AllocListHead
};

unsigned short m_bcdDevice;
unsigned short m_vid;
/*****************************************************************************

USBView_OnInitDialog()

*****************************************************************************/

void USB_OnInit(unsigned short* bcdDevice, unsigned short* Vid)
{
	m_bcdDevice = 0;
	m_vid = 0;

	EnumerateHostControllers();

	*Vid = m_vid;
	*bcdDevice = m_bcdDevice;
}

/*****************************************************************************

EnumerateHostControllers()

*****************************************************************************/

void EnumerateHostControllers()
{
	HANDLE                           hHCDev = NULL;
	HDEVINFO                         deviceInfo = NULL;
	SP_DEVINFO_DATA                  deviceInfoData;
	SP_DEVICE_INTERFACE_DATA         deviceInterfaceData;
	PSP_DEVICE_INTERFACE_DETAIL_DATA deviceDetailData = NULL;
	ULONG                            index = 0;
	ULONG                            requiredLength = 0;
	BOOL                             success;

	deviceInfo = SetupDiGetClassDevs((LPGUID)&GUID_CLASS_USB_HOST_CONTROLLER, NULL, NULL, (DIGCF_PRESENT | DIGCF_DEVICEINTERFACE));
	deviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

	for (index = 0; SetupDiEnumDeviceInfo(deviceInfo, index, &deviceInfoData); index++)
	{
		deviceInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
		success = SetupDiEnumDeviceInterfaces(deviceInfo, 0, (LPGUID)&GUID_CLASS_USB_HOST_CONTROLLER, index, &deviceInterfaceData);

		if (!success)
		{
			OOPS();
			break;
		}

		success = SetupDiGetDeviceInterfaceDetail(deviceInfo, &deviceInterfaceData, NULL, 0, &requiredLength, NULL);

		if (!success && GetLastError() != ERROR_INSUFFICIENT_BUFFER)
		{
			OOPS();
			break;
		}

		deviceDetailData = ALLOC(requiredLength);
		if (deviceDetailData == NULL)
		{
			OOPS();
			break;
		}

		deviceDetailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
		success = SetupDiGetDeviceInterfaceDetail(deviceInfo, &deviceInterfaceData, deviceDetailData, requiredLength, &requiredLength, NULL);

		if (!success)
		{
			OOPS();
			break;
		}

		hHCDev = CreateFile(deviceDetailData->DevicePath, GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

		if (hHCDev != INVALID_HANDLE_VALUE)
		{
			EnumerateHostController(hHCDev);
			CloseHandle(hHCDev);
		}
		FREE(deviceDetailData);
	}

	SetupDiDestroyDeviceInfoList(deviceInfo);
	return;
}

/*****************************************************************************

GetDeviceProperty()

*****************************************************************************/

_Success_(return == TRUE)
BOOL GetDeviceProperty(
	_In_    HDEVINFO         DeviceInfoSet,
	_In_    PSP_DEVINFO_DATA DeviceInfoData,
	_In_    DWORD            Property,
	_Outptr_  LPTSTR * ppBuffer)
{
	BOOL bResult;
	DWORD requiredLength = 0;
	DWORD lastError;

	if (ppBuffer == NULL)
	{
		return FALSE;
	}

	*ppBuffer = NULL;
	bResult = SetupDiGetDeviceRegistryProperty(DeviceInfoSet, DeviceInfoData, Property, NULL, NULL, 0, &requiredLength);
	lastError = GetLastError();

	if ((requiredLength == 0) || (bResult != FALSE && lastError != ERROR_INSUFFICIENT_BUFFER))
	{
		return FALSE;
	}

	*ppBuffer = ALLOC(requiredLength);

	if (*ppBuffer == NULL)
	{
		return FALSE;
	}

	bResult = SetupDiGetDeviceRegistryProperty(DeviceInfoSet, DeviceInfoData, Property, NULL, (PBYTE)*ppBuffer, requiredLength, &requiredLength);
	if (bResult == FALSE)
	{
		FREE(*ppBuffer);
		*ppBuffer = NULL;
		return FALSE;
	}

	return TRUE;
}

//*****************************************************************************
//
// EnumerateHostController()
//
//*****************************************************************************

void EnumerateHostController(HANDLE hHCDev)
{
	PCHAR rootHubName = NULL;
	rootHubName = GetRootHubName(hHCDev);

	if (rootHubName != NULL)
	{
		size_t cbHubName = 0;
		HRESULT hr = S_OK;

		hr = StringCbLength(rootHubName, MAX_DRIVER_KEY_NAME, &cbHubName);
		if (SUCCEEDED(hr))
		{
			EnumerateHub(rootHubName, cbHubName);
		}
	}
	else
	{
		OOPS();
	}
	return;
}

//*****************************************************************************
//
// GetRootHubName()
//
//*****************************************************************************

PCHAR GetRootHubName(HANDLE HostController)
{
	BOOL                success = 0;
	ULONG               nBytes = 0;
	USB_ROOT_HUB_NAME   rootHubName;
	PUSB_ROOT_HUB_NAME  rootHubNameW = NULL;
	PCHAR               rootHubNameA = NULL;

	success = DeviceIoControl(HostController, IOCTL_USB_GET_ROOT_HUB_NAME, 0, 0, &rootHubName, sizeof(rootHubName), &nBytes, NULL);
	if (!success)
	{
		OOPS();
		goto GetRootHubNameError;
	}

	nBytes = rootHubName.ActualLength;
	rootHubNameW = ALLOC(nBytes);

	if (rootHubNameW == NULL)
	{
		OOPS();
		goto GetRootHubNameError;
	}

	success = DeviceIoControl(HostController, IOCTL_USB_GET_ROOT_HUB_NAME, NULL, 0, rootHubNameW, nBytes, &nBytes, NULL);
	if (!success)
	{
		OOPS();
		goto GetRootHubNameError;
	}

	rootHubNameA = WideStrToMultiStr(rootHubNameW->RootHubName, nBytes - sizeof(USB_ROOT_HUB_NAME) + sizeof(WCHAR));
	FREE(rootHubNameW);
	return rootHubNameA;

GetRootHubNameError:
	if (rootHubNameW != NULL)
	{
		FREE(rootHubNameW);
		rootHubNameW = NULL;
	}
	return NULL;

}

//*****************************************************************************
//
// WideStrToMultiStr()
//
//*****************************************************************************

PCHAR WideStrToMultiStr(_In_reads_bytes_(cbWideStr) PWCHAR WideStr, _In_ size_t cbWideStr)
{
	ULONG  nBytes = 0;
	PCHAR  MultiStr = NULL;
	PWCHAR pWideStr = NULL;

	pWideStr = (PWCHAR)ALLOC((DWORD)cbWideStr + sizeof(WCHAR));
	if (NULL == pWideStr)
	{
		return NULL;
	}
	memset(pWideStr, 0, cbWideStr + sizeof(WCHAR));
	memcpy(pWideStr, WideStr, cbWideStr);

	nBytes = WideCharToMultiByte(CP_ACP, WC_NO_BEST_FIT_CHARS, pWideStr, -1, NULL, 0, NULL, NULL);
	if (nBytes == 0)
	{
		FREE(pWideStr);
		return NULL;
	}

	MultiStr = ALLOC(nBytes);
	if (MultiStr == NULL)
	{
		FREE(pWideStr);
		return NULL;
	}

	nBytes = WideCharToMultiByte(CP_ACP, WC_NO_BEST_FIT_CHARS, pWideStr, -1, MultiStr, nBytes, NULL, NULL);
	if (nBytes == 0)
	{
		FREE(MultiStr);
		FREE(pWideStr);
		return NULL;
	}

	FREE(pWideStr);
	return MultiStr;
}

//*****************************************************************************
//
// EnumerateHub()
//
// HubName - Name of this hub.  This pointer is kept so the caller can neither
// free nor reuse this memory.
//
//*****************************************************************************

void EnumerateHub(_In_reads_(cbHubName) PCHAR HubName, _In_ size_t cbHubName)
{
	PUSB_NODE_INFORMATION    hubInfo = NULL;
	HANDLE                  hHubDevice = INVALID_HANDLE_VALUE;
	PVOID                   info = NULL;
	PCHAR                   deviceName = NULL;
	ULONG                   nBytes = 0;
	BOOL                    success = 0;
	HRESULT                 hr = S_OK;
	size_t                  cchHeader = 0;
	size_t                  cchFullHubName = 0;

	info = ALLOC(sizeof(USBEXTERNALHUBINFO));
	if (info == NULL)
	{
		OOPS();
		goto EnumerateHubError;
	}

	hubInfo = (PUSB_NODE_INFORMATION)ALLOC(sizeof(USB_NODE_INFORMATION));
	if (hubInfo == NULL)
	{
		OOPS();
		goto EnumerateHubError;
	}

	((PUSBROOTHUBINFO)info)->HubInfo = hubInfo;
	((PUSBROOTHUBINFO)info)->HubName = HubName;

	hr = StringCbLength("\\\\.\\", MAX_DEVICE_PROP, &cchHeader);
	if (FAILED(hr))
	{
		goto EnumerateHubError;
	}

	cchFullHubName = cchHeader + cbHubName + 1;
	deviceName = (PCHAR)ALLOC((DWORD)cchFullHubName);
	if (deviceName == NULL)
	{
		OOPS();
		goto EnumerateHubError;
	}

	hr = StringCchCopyN(deviceName, cchFullHubName, "\\\\.\\", cchHeader);
	if (FAILED(hr))
	{
		goto EnumerateHubError;
	}

	hr = StringCchCatN(deviceName, cchFullHubName, HubName, cbHubName);
	if (FAILED(hr))
	{
		goto EnumerateHubError;
	}

	hHubDevice = CreateFile(deviceName, GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
	FREE(deviceName);
	if (hHubDevice == INVALID_HANDLE_VALUE)
	{
		OOPS();
		goto EnumerateHubError;
	}


	success = DeviceIoControl(hHubDevice, IOCTL_USB_GET_NODE_INFORMATION, hubInfo, sizeof(USB_NODE_INFORMATION), hubInfo, sizeof(USB_NODE_INFORMATION), &nBytes, NULL);
	if (!success)
	{
		OOPS();
		goto EnumerateHubError;
	}

	EnumerateHubPorts(hHubDevice, hubInfo->u.HubInformation.HubDescriptor.bNumberOfPorts);
	CloseHandle(hHubDevice);
	return;

EnumerateHubError:
	if (hHubDevice != INVALID_HANDLE_VALUE)
	{
		CloseHandle(hHubDevice);
		hHubDevice = INVALID_HANDLE_VALUE;
	}

	if (hubInfo)
	{
		FREE(hubInfo);
	}

	if (info)
	{
		FREE(info);
	}

	if (HubName)
	{
		FREE(HubName);
	}
}

//*****************************************************************************
//
// EnumerateHubPorts()
//
// hHubDevice - Handle of the hub device to enumerate.
//
// NumPorts - Number of ports on the hub.
//
//*****************************************************************************

void EnumerateHubPorts(HANDLE hHubDevice, ULONG NumPorts)
{
	ULONG       index = 0;
	BOOL        success = 0;
	HRESULT     hr = S_OK;

	PUSB_NODE_CONNECTION_INFORMATION_EX    connectionInfoEx;
	PUSB_PORT_CONNECTOR_PROPERTIES         pPortConnectorProps;
	USB_PORT_CONNECTOR_PROPERTIES          portConnectorProps;

	for (index = 1; index <= NumPorts; index++)
	{
		ULONG nBytesEx;
		ULONG nBytes = 0;

		connectionInfoEx = NULL;
		pPortConnectorProps = NULL;
		ZeroMemory(&portConnectorProps, sizeof(portConnectorProps));

		nBytesEx = sizeof(USB_NODE_CONNECTION_INFORMATION_EX) + (sizeof(USB_PIPE_INFO) * 30);

		connectionInfoEx = (PUSB_NODE_CONNECTION_INFORMATION_EX)ALLOC(nBytesEx);
		if (connectionInfoEx == NULL)
		{
			OOPS();
			break;
		}

		portConnectorProps.ConnectionIndex = index;
		success = DeviceIoControl(hHubDevice, IOCTL_USB_GET_PORT_CONNECTOR_PROPERTIES, &portConnectorProps, sizeof(USB_PORT_CONNECTOR_PROPERTIES), &portConnectorProps, sizeof(USB_PORT_CONNECTOR_PROPERTIES), &nBytes, NULL);
		if (success && nBytes == sizeof(USB_PORT_CONNECTOR_PROPERTIES))
		{
			pPortConnectorProps = (PUSB_PORT_CONNECTOR_PROPERTIES)ALLOC(portConnectorProps.ActualLength);
			if (pPortConnectorProps != NULL)
			{
				pPortConnectorProps->ConnectionIndex = index;

				success = DeviceIoControl(hHubDevice, IOCTL_USB_GET_PORT_CONNECTOR_PROPERTIES, pPortConnectorProps, portConnectorProps.ActualLength, pPortConnectorProps, portConnectorProps.ActualLength, &nBytes, NULL);
				if (!success || nBytes < portConnectorProps.ActualLength)
				{
					FREE(pPortConnectorProps);
					pPortConnectorProps = NULL;
				}
			}
		}

		connectionInfoEx->ConnectionIndex = index;

		success = DeviceIoControl(hHubDevice, IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX, connectionInfoEx, nBytesEx, connectionInfoEx, nBytesEx, &nBytesEx, NULL);
		if (success && connectionInfoEx->DeviceDescriptor.idVendor == 0x04B4) {
			m_vid = connectionInfoEx->DeviceDescriptor.idVendor;
			m_bcdDevice = connectionInfoEx->DeviceDescriptor.bcdDevice;
		}

		if (connectionInfoEx->DeviceIsHub)
		{
			PCHAR extHubName;
			size_t cbHubName = 0;

			extHubName = GetExternalHubName(hHubDevice, index);
			if (extHubName != NULL)
			{
				hr = StringCbLength(extHubName, MAX_DRIVER_KEY_NAME, &cbHubName);
				if (SUCCEEDED(hr))
				{
					EnumerateHub(extHubName, cbHubName);
				}
			}
		}

		else {
			FREE(connectionInfoEx);
			FREE(pPortConnectorProps);
		}
	} // for
}

//*****************************************************************************
//
// GetExternalHubName()
//
//*****************************************************************************

PCHAR GetExternalHubName(HANDLE Hub, ULONG ConnectionIndex)
{
	BOOL                        success = 0;
	ULONG                       nBytes = 0;
	USB_NODE_CONNECTION_NAME    extHubName;
	PUSB_NODE_CONNECTION_NAME   extHubNameW = NULL;
	PCHAR                       extHubNameA = NULL;

	extHubName.ConnectionIndex = ConnectionIndex;

	success = DeviceIoControl(Hub, IOCTL_USB_GET_NODE_CONNECTION_NAME, &extHubName, sizeof(extHubName), &extHubName, sizeof(extHubName), &nBytes, NULL);
	if (!success)
	{
		OOPS();
		goto GetExternalHubNameError;
	}

	nBytes = extHubName.ActualLength;
	if (nBytes <= sizeof(extHubName))
	{
		OOPS();
		goto GetExternalHubNameError;
	}

	extHubNameW = ALLOC(nBytes);
	if (extHubNameW == NULL)
	{
		OOPS();
		goto GetExternalHubNameError;
	}

	extHubNameW->ConnectionIndex = ConnectionIndex;

	success = DeviceIoControl(Hub, IOCTL_USB_GET_NODE_CONNECTION_NAME, extHubNameW, nBytes, extHubNameW, nBytes, &nBytes, NULL);
	if (!success)
	{
		OOPS();
		goto GetExternalHubNameError;
	}

	extHubNameA = WideStrToMultiStr(extHubNameW->NodeName, nBytes - sizeof(USB_NODE_CONNECTION_NAME) + sizeof(WCHAR));

	FREE(extHubNameW);
	return extHubNameA;

GetExternalHubNameError:
	if (extHubNameW != NULL)
	{
		FREE(extHubNameW);
		extHubNameW = NULL;
	}

	return NULL;
}

//*****************************************************************************
//
// FreeDeviceInfoNode()
//
//*****************************************************************************

void FreeDeviceInfoNode(_In_ PDEVICE_INFO_NODE * ppNode)
{
	if (ppNode == NULL) return;
	if (*ppNode == NULL) return;
	if ((*ppNode)->DeviceDetailData != NULL) FREE((*ppNode)->DeviceDetailData);
	if ((*ppNode)->DeviceDescName != NULL) FREE((*ppNode)->DeviceDescName);
	if ((*ppNode)->DeviceDriverName != NULL) FREE((*ppNode)->DeviceDriverName);

	FREE(*ppNode);
	*ppNode = NULL;
}

/*****************************************************************************

 MyAlloc()

*****************************************************************************/
_Success_(return != NULL)
_Post_writable_byte_size_(dwBytes)
HGLOBAL MyAlloc(_In_ PCHAR File, ULONG Line, DWORD dwBytes)
{
	PALLOCHEADER header;
	DWORD dwRequest = dwBytes;

	if (0 == dwBytes)
	{
		return NULL;
	}
	dwBytes += sizeof(ALLOCHEADER);

	if (dwBytes > dwRequest)
	{
		header = (PALLOCHEADER)GlobalAlloc(GPTR, dwBytes);
		if (header != NULL)
		{
			InsertTailList(&AllocListHead, &header->ListEntry);
			header->File = File;
			header->Line = Line;
			return (HGLOBAL)(header + 1);
		}
	}
	return NULL;
}

/*****************************************************************************

 MyFree()

*****************************************************************************/

HGLOBAL MyFree(HGLOBAL hMem)
{
	PALLOCHEADER header;

	if (hMem)
	{
		header = (PALLOCHEADER)hMem;
		header--;
		RemoveEntryList(&header->ListEntry);
		return GlobalFree((HGLOBAL)header);
	}

	return GlobalFree(hMem);
}

/*****************************************************************************

Oops()

*****************************************************************************/

void Oops(_In_ PCHAR File, ULONG Line) {
	char szBuf[1024];
	LPTSTR lpMsgBuf;
	DWORD dwGLE = GetLastError();

	memset(szBuf, 0, sizeof(szBuf));

	if (FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, dwGLE, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPTSTR)&lpMsgBuf, 0, NULL))
	{
		StringCchPrintf(szBuf, sizeof(szBuf), "File: %s, Line %d\r\nGetLastError 0x%x %u %s\n", File, Line, dwGLE, dwGLE, lpMsgBuf);
	}
	else
	{
		StringCchPrintf(szBuf, sizeof(szBuf), "File: %s, Line %d\r\nGetLastError 0x%x %u\r\n", File, Line, dwGLE, dwGLE);
	}
	OutputDebugString(szBuf);
	LocalFree(lpMsgBuf);
	return;
}

/*****************************************************************************

Windows_info()
//Windows os 버전 판단 20201116
*****************************************************************************/

int Windows_info() {
	FILE* fp;
	char buf[26] = { 0, };
	int Value = 0;
	int cnt = 0;

	cnt = 0;
	fp = _popen("wmic os get BuildNumber", "rt");
	while (fgets(buf, 12, fp))
	{
		if (cnt == 2) {
			Value = atoi(buf);
		}
		cnt++;
	}

	_pclose(fp);
	return Value;
}
