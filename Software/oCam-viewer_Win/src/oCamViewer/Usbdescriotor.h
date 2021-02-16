//*****************************************************************************
// I N C L U D E S
//*****************************************************************************

#include <initguid.h>
#include <windows.h>
#include <setupapi.h>
#include <usbioctl.h>
#include <strsafe.h>
#include <usb.h>

//*****************************************************************************
// D E F I N E S
//*****************************************************************************

#define USB_IAD_DESCRIPTOR_TYPE	0x0B
#define NUM_STRING_DESC_TO_GET 32
#define MAX_DEVICE_PROP 200
#define MAX_DRIVER_KEY_NAME 256


#define ALLOC(dwBytes) MyAlloc(__FILE__, __LINE__, (dwBytes))
#define FREE(hMem)  MyFree((hMem))
#define OOPS() Oops(__FILE__, __LINE__)

#define IsListEmpty(ListHead) \
    ((ListHead)->Flink == (ListHead))

#define RemoveHeadList(ListHead) \
    (ListHead)->Flink;\
    {RemoveEntryList((ListHead)->Flink)}

#define RemoveEntryList(Entry) {\
		PLIST_ENTRY _EX_Blink; \
		PLIST_ENTRY _EX_Flink; \
		_EX_Flink = (Entry)->Flink; \
		_EX_Blink = (Entry)->Blink; \
		_EX_Blink->Flink = _EX_Flink; \
		_EX_Flink->Blink = _EX_Blink; \
}

#define InsertTailList(ListHead,Entry) {\
    PLIST_ENTRY _EX_Blink;\
    PLIST_ENTRY _EX_ListHead;\
    _EX_ListHead = (ListHead);\
    _EX_Blink = _EX_ListHead->Blink;\
    (Entry)->Flink = _EX_ListHead;\
    (Entry)->Blink = _EX_Blink;\
    _EX_Blink->Flink = (Entry);\
    _EX_ListHead->Blink = (Entry);\
 }

/*****************************************************************************
 T Y P E D E F S
*****************************************************************************/

typedef enum _USBDEVICEINFOTYPE
{
	HostControllerInfo,
	RootHubInfo,
	ExternalHubInfo,
	DeviceInfo
} USBDEVICEINFOTYPE, * PUSBDEVICEINFOTYPE;

typedef struct _STRING_DESCRIPTOR_NODE
{
	struct _STRING_DESCRIPTOR_NODE* Next;
	UCHAR                           DescriptorIndex;
	USHORT                          LanguageID;
	USB_STRING_DESCRIPTOR           StringDescriptor[1];
} STRING_DESCRIPTOR_NODE, * PSTRING_DESCRIPTOR_NODE;

typedef struct _DEVICE_INFO_NODE {
	HDEVINFO                         DeviceInfo;
	LIST_ENTRY                       ListEntry;
	SP_DEVINFO_DATA                  DeviceInfoData;
	SP_DEVICE_INTERFACE_DATA         DeviceInterfaceData;
	PSP_DEVICE_INTERFACE_DETAIL_DATA DeviceDetailData;
	PSTR                             DeviceDescName;
	ULONG                            DeviceDescNameLength;
	PSTR                             DeviceDriverName;
	ULONG                            DeviceDriverNameLength;
	DEVICE_POWER_STATE               LatestDevicePowerState;
} DEVICE_INFO_NODE, * PDEVICE_INFO_NODE;

typedef struct _USB_DEVICE_PNP_STRINGS
{
	PCHAR DeviceId;
	PCHAR DeviceDesc;
	PCHAR HwId;
	PCHAR Service;
	PCHAR DeviceClass;
	PCHAR PowerState;
} USB_DEVICE_PNP_STRINGS, * PUSB_DEVICE_PNP_STRINGS;

typedef struct _USBROOTHUBINFO
{
	USBDEVICEINFOTYPE                   DeviceInfoType;
	PUSB_NODE_INFORMATION               HubInfo;
	PUSB_HUB_INFORMATION_EX             HubInfoEx;
	PCHAR                               HubName;
	PUSB_PORT_CONNECTOR_PROPERTIES      PortConnectorProps;
	PUSB_DEVICE_PNP_STRINGS             UsbDeviceProperties;
	PDEVICE_INFO_NODE                   DeviceInfoNode;
	PUSB_HUB_CAPABILITIES_EX            HubCapabilityEx;


} USBROOTHUBINFO, * PUSBROOTHUBINFO;

typedef struct _USBEXTERNALHUBINFO
{
	USBDEVICEINFOTYPE                      DeviceInfoType;
	PUSB_NODE_INFORMATION                  HubInfo;
	PUSB_HUB_INFORMATION_EX                HubInfoEx;
	PCHAR                                  HubName;
	PUSB_NODE_CONNECTION_INFORMATION_EX    ConnectionInfo;
	PUSB_PORT_CONNECTOR_PROPERTIES         PortConnectorProps;
	PUSB_DESCRIPTOR_REQUEST                ConfigDesc;
	PUSB_DESCRIPTOR_REQUEST                BosDesc;
	PSTRING_DESCRIPTOR_NODE                StringDescs;
	PUSB_NODE_CONNECTION_INFORMATION_EX_V2 ConnectionInfoV2; // NULL if root HUB
	PUSB_DEVICE_PNP_STRINGS                UsbDeviceProperties;
	PDEVICE_INFO_NODE                      DeviceInfoNode;
	PUSB_HUB_CAPABILITIES_EX               HubCapabilityEx;
} USBEXTERNALHUBINFO, * PUSBEXTERNALHUBINFO;

typedef struct _ALLOCHEADER
{
	LIST_ENTRY  ListEntry;
	PCHAR       File;
	ULONG       Line;
} ALLOCHEADER, * PALLOCHEADER;

typedef struct _DEVICE_GUID_LIST {
	HDEVINFO   DeviceInfo;
	LIST_ENTRY ListHead;
} DEVICE_GUID_LIST, * PDEVICE_GUID_LIST;

//*****************************************************************************
// L O C A L    F U N C T I O N    P R O T O T Y P E S
//*****************************************************************************

void USB_OnInit(unsigned short* bcdDevice, unsigned short* Vid);
void EnumerateHostControllers();
void EnumerateHostController(HANDLE hHCDev);

_Success_(return == TRUE)
BOOL GetDeviceProperty(_In_ HDEVINFO DeviceInfoSet, _In_ PSP_DEVINFO_DATA DeviceInfoData, _In_ DWORD Property, _Outptr_  LPTSTR * ppBuffer);

PCHAR WideStrToMultiStr(_In_reads_bytes_(cbWideStr) PWCHAR WideStr, _In_ size_t cbWideStr);
PCHAR GetRootHubName(HANDLE HostController);

void EnumerateHub(_In_reads_(cbHubName) PCHAR HubName, _In_ size_t cbHubName);
void EnumerateHubPorts(HANDLE hHubDevice, ULONG NumPorts);

PCHAR GetExternalHubName(HANDLE Hub, ULONG ConnectionIndex);

void FreeDeviceInfoNode(_In_ PDEVICE_INFO_NODE * ppNode);

HGLOBAL MyFree(HGLOBAL hMem);
void Oops(_In_ PCHAR File, ULONG Line);
int Windows_info();
