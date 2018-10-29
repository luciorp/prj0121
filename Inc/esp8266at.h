#ifndef	_ESP8266AT_H
#define	_ESP8266AT_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "main.h"
#include "stm32f1xx_hal.h" // trocar o include conforme a familia stm32f0xx_hal.h para o M0 e stm32f3xx_hal.h para o M4

extern UART_HandleTypeDef huart3; // Usar o mesmo nome da struct utilizada na config do SPI utilizado

#define 	_WIFI_RESET_PIN 			ESP_RST_Pin // Usar o nome do pino de reset configurado.
#define 	_WIFI_RESET_PORT 			ESP_RST_GPIO_Port  // Usar o nome da porta de reset configurado.

#define		_WIFI_USART					huart3
#define		_WIFI_RX_SIZE				512
#define		_WIFI_RX_FOR_DATA_SIZE		1024
#define		_WIFI_TX_SIZE				512
#define		_WIFI_TASK_SIZE				512


#define		_WIFI_WAIT_TIME_LOW			1000
#define		_WIFI_WAIT_TIME_MED			10000
#define		_WIFI_WAIT_TIME_HIGH		25000
#define		_WIFI_WAIT_TIME_VERYHIGH	60000

typedef	enum
{
	WifiMode_Error                          =     0,
	WifiMode_Station                        =     1,
	WifiMode_SoftAp                         =     2,
	WifiMode_StationAndSoftAp               =     3,	
}WifiMode_t;

typedef enum
{
	WifiEncryptionType_Open                 =     0,
	WifiEncryptionType_WPA_PSK              =     2,
	WifiEncryptionType_WPA2_PSK             =     3,
	WifiEncryptionType_WPA_WPA2_PSK         =     4,
}WifiEncryptionType_t;

typedef enum
{
	WifiConnectionStatus_Error              =     0,
	WifiConnectionStatus_GotIp              =     2,
	WifiConnectionStatus_Connected          =     3,
	WifiConnectionStatus_Disconnected       =     4,
	WifiConnectionStatus_ConnectionFail     =     5,
}WifiConnectionStatus_t;

typedef struct
{
	WifiConnectionStatus_t      status;
	uint8_t                     LinkId;
	char                        Type[4];
	char                        RemoteIp[17];
	uint16_t                    RemotePort;
	uint16_t                    LocalPort;
	bool                        RunAsServer;
}WifiConnection_t;

typedef struct
{
	//----------------Usart	Paremeter
	uint8_t                       usartBuff;
	uint8_t                       RxBuffer[_WIFI_RX_SIZE];
	uint8_t                       TxBuffer[_WIFI_TX_SIZE];
	uint16_t                      RxIndex;
	uint8_t                       RxBufferForData[_WIFI_RX_FOR_DATA_SIZE];
	uint8_t                       RxBufferForDataTmp[8];
	uint8_t                       RxIndexForDataTmp;
	uint16_t                      RxIndexForData;
	uint16_t                      RxDataLen;
	uint8_t                       RxDataConnectionNumber;
	uint32_t                      RxDataLastTime;
	bool                          RxIsData;
	bool                          GotNewData;
	//----------------General	Parameter			
	WifiMode_t                    Mode;
	char                          MyIP[16];	
	char                          MyGateWay[16];
	//----------------Station	Parameter
	bool                          StationDhcp;
	char                          StationIp[16];	
	//----------------SoftAp Parameter
	bool                          SoftApDhcp;
	char                          SoftApConnectedDevicesIp[6][16];	
	char                          SoftApConnectedDevicesMac[6][18];	
	//----------------TcpIp Parameter
	bool                          TcpIpMultiConnection;
	uint16_t                      TcpIpPingAnswer;
	WifiConnection_t              TcpIpConnections[5];
	//----------------
}Wifi_t;


extern Wifi_t	Wifi;

extern WifiConnection_t Connection;



void	Wifi_RxCallBack(void);

void	Wifi_HWRestart(void);

void	Wifi_Init(void);

bool	Wifi_Restart(void);
bool	Wifi_DeepSleep(uint16_t DelayMs);
bool	Wifi_FactoryReset(void);
bool	Wifi_Update(void);
bool	Wifi_SetRfPower(uint8_t Power_0_to_82);

bool	Wifi_SetMode(WifiMode_t	WifiMode_);
bool	Wifi_GetMode(void);
bool	Wifi_GetMyIp(void);

bool	Wifi_Station_ConnectToAp(char *SSID,char *Pass,char *MAC);
bool	Wifi_Station_Disconnect(void);
bool	Wifi_Station_DhcpEnable(bool Enable);
bool	Wifi_Station_DhcpIsEnable(void);
bool	Wifi_Station_SetIp(char *IP,char *GateWay,char *NetMask);

bool  Wifi_SoftAp_GetConnectedDevices(void);
bool  Wifi_SoftAp_Create(char *SSID,char *password,uint8_t channel,WifiEncryptionType_t WifiEncryptionType,uint8_t MaxConnections_1_to_4,bool HiddenSSID);

bool  Wifi_TcpIp_GetConnectionStatus(void);
bool  Wifi_TcpIp_Ping(char *PingTo);
bool  Wifi_TcpIp_SetMultiConnection(bool EnableMultiConnections);
bool  Wifi_TcpIp_GetMultiConnection(void);
bool  Wifi_TcpIp_StartTcpConnection(uint8_t LinkId,char *RemoteIp,uint16_t RemotePort,uint16_t TimeOut_S);
bool  Wifi_TcpIp_StartUdpConnection(uint8_t LinkId,char *RemoteIp,uint16_t RemotePort,uint16_t LocalPort);
bool  Wifi_TcpIp_Close(uint8_t LinkId);
bool  Wifi_TcpIp_SetEnableTcpServer(uint16_t PortNumber);
bool  Wifi_TcpIp_SetDisableTcpServer(uint16_t PortNumber);
bool  Wifi_TcpIp_SendDataUdp(uint8_t LinkId,uint16_t dataLen,uint8_t *data);
bool  Wifi_TcpIp_SendDataTcp(uint8_t LinkId,uint16_t dataLen,uint8_t *data);

#endif


