#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include "epoll_timerfd_utilities.h"
#include <applibs/uart.h>
#include <applibs/gpio.h>
#include <applibs/log.h>
#include <applibs/wificonfig.h>
#include "mt3620_rdb.h"

// File descriptors - initialized to invalid value
static int uartFd = -1;
static int gpioButtonFd = -1;
static int gpioButtonTimerFd = -1;
static int gpioWiFiLedFd = -1;
static int epollFd = -1;

// State variables
static GPIO_Value_Type buttonState = GPIO_Value_High;

// Termination state
static volatile sig_atomic_t terminationRequired = false;

// Added for BlueTooth
#define RECEIVE_BUFFER_SIZE 1024
#define WIFI_SSID_PW_DELIMITER ":"
#define LINE_DELIMITER "HUN"

typedef void(*uart_line_received_handler_t)(char *pszLine, size_t nBytesRead);

///<summary>static receive buffer for UART</summary>
static char receiveBuffer[RECEIVE_BUFFER_SIZE];

///<summary>Number of bytes in ring buffer</summary>
static size_t nBytesInBuffer = 0;

/// <summary>
///		Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
	// Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
	terminationRequired = true;
}

/// <summary>
///		Show error logs when call wifi functions
/// </summary>
/// <param name="functionName">
///		Name of function calling wifi function
/// <param name="wifiFuctionName">
///		Name of wifi function called.
/// <param name="networkName">
///		Name of network attempted to connect
/// <param name="result">
///		Result of Wifi function
///	</param>
void WiFiErrorCheckLog(char *functionName, char *wifiFuctionName, char *networkName, int result)
{
	switch (errno)
	{
	case EACCES:
	{
		Log_Debug("[INFO - %s] There is no permission for WifiConfig in App Manifest. Check app_manifest.json\n", functionName);
		break;
	}
	case EAGAIN:
	{
		Log_Debug("[INFO - %s] WiFi device isn't ready yet\n", functionName);
		break;
	}
	case EEXIST:
	{
		Log_Debug("[INFO - %s] The \"%s\" WiFi network is already stored on the device.\n", functionName, networkName);
		break;
	}
	case EFAULT:
	{
		Log_Debug("[INFO - %s] The WiFi network name is NULL\n");
		break;
	}
	case ERANGE:
	{
		Log_Debug("[INFO - %s] The WiFi network name Length is 0 or greater than %d\n", functionName, WIFICONFIG_SSID_MAX_LENGTH);
		break;
	}
	case ENOTCONN:
	{
		Log_Debug("[INFO - %s] Not currently connected to a WiFi network.\n", functionName);
		break;
	}
	case EINVAL:
	{
		Log_Debug("[INFO - %s] The stored Network or its struct version is invaild\n", functionName);
		break;
	}
	default:
	{
		if (networkName == NULL)
		{
			Log_Debug("[ERROR - %s] %s failed result %d. Errno: %s (%d).\n",
				functionName, wifiFuctionName, result, strerror(errno), errno);
		}
		else
		{
			Log_Debug(
				"[ERROR - %s ] %s failed to store WiFi network \"%s\" with result %d. Errno: %s (%d).\n",
				functionName, wifiFuctionName, networkName, result, strerror(errno), errno);
		}
	}
	}
}

/// <summary>
///		Delete all stored Wifi network.
///	</param>
/// </summary>
static void DeleteAllWifiNetwork(void)
{
	int result = 0;
	result = WifiConfig_ForgetAllNetworks();
	if (result < 0)
	{
		WiFiErrorCheckLog("DeleteAllWifiNetwork", "WifiConfig_ForgetAllNetworks\n", NULL, result);
	}
	else
	{
		Log_Debug("[INFO - DeleteAllWifiNetwork] Successfully delete all stored WiFi network\n");
	}
}

/// <summary>
///		Add open Wifi network.
/// </summary>
/// <param name="networkName">
///		The open WiFi network name (SSID).
///	</param>
/// <return>
///		0 for success, or -1 for failure,
/// </return>
static int AddOpenWifi(char *networkName)
{
	int result = -1;
	result = WifiConfig_StoreOpenNetwork((uint8_t *)networkName, strlen(networkName));
	//Errors for adding open WiFi network.
	if (result < 0)
	{
		WiFiErrorCheckLog("AddOpenWifi", "WifiConfig_StoreOpenNetwork", networkName, result);
	}
	//Success for adding open WiFi network.
	else
	{
		Log_Debug("[INFO - AddOpenWifi] Successfully stored WiFi network: \"%s\".\n", networkName);
	}
	return result;
}
/// <summary>
///		Add Wpa2 Wifi network.
/// </summary>
/// <param name="networkName">
///		The Wpa2 WiFi network name (SSID).
///	</param>
/// <param name="networkPassword">
///		The Wpa2 WiFi network password.
///	</param>
/// <return>
///		0 for success, or -1 for failure,
/// </return>
static int AddWpa2Wifi(char *networkName, char *networkPassword)
{
	int result = -1;
	result = WifiConfig_StoreWpa2Network((uint8_t *)networkName, strlen(networkName), networkPassword, strlen(networkPassword));
	//Errors for adding open WiFi network.
	if (result < 0)
	{
		WiFiErrorCheckLog("AddOpenWifi", "WifiConfig_StoreOpenNetwork", networkName, result);
	}
	//Success for adding open WiFi network.
	else
	{
		Log_Debug("[INFO - AddOpenWifi] Successfully stored WiFi network: \"%s\".\n", networkName);
	}
	return result;
}

/// <summary>
///			Show stored WiFi networks in your azure sphere.
/// </summary>
static void PrintStoredWiFiNetworks(void)
{
	int result = WifiConfig_GetStoredNetworkCount();
	//Error for getting stored network count.
	if (result < 0)
	{
		WiFiErrorCheckLog("PrintStoredWiFiNetworks", "WifiConfig_GetStoredNetworkCount", NULL, result);
	}
	//Success for getting stored network count.
	else if (result == 0)
	{
		Log_Debug("[INFO - PrintStoredWiFiNetworks] There are no stored WiFi networks.\n");
	}
	else
	{
		Log_Debug("[INFO - PrintStoredWiFiNetworks] There are %d stored WiFi networks:\n", result);
		size_t networkCount = (size_t)result;
		WifiConfig_StoredNetwork *networks =
			(WifiConfig_StoredNetwork *)malloc(sizeof(WifiConfig_StoredNetwork) * networkCount);
		int result = WifiConfig_GetStoredNetworks(networks, networkCount);
		//Error for getting stored networks.
		if (result < 0)
		{
			WiFiErrorCheckLog("PrintStoredWiFiNetworks", "WifiConfig_GetStoredNetworks", NULL, result);
		}
		//Success for getting stored networks.
		else
		{
			networkCount = (size_t)result;
			for (size_t i = 0; i < networkCount; ++i)
			{
				Log_Debug("[INFO - PrintStoredWiFiNetworks] %3d) SSID \"%.*s\"\n", i, networks[i].ssidLength, networks[i].ssid);
			}
		}
		free(networks);
	}
}
/// <summary>
///     Show current connected Wifi network info.
/// </summary>
static void PrintCurrentlyConnectedWiFiNetwork(void)
{
	WifiConfig_ConnectedNetwork network;
	int result = WifiConfig_GetCurrentNetwork(&network);
	//Error for getting current WiFi network.
	if (result < 0)
	{
		WiFiErrorCheckLog("PrintCurrentlyConnectedWiFiNetwork", "WifiConfig_GetCurrentNetwork", NULL, result);
	}
	//Success for getting current WiFi network.
	else
	{
		Log_Debug("[INFO - PrintCurrentlyConnectedWiFiNetwork] Currently connected WiFi network: ");
		Log_Debug("[INFO - PrintCurrentlyConnectedWiFiNetwork] SSID \"%.*s\", BSSID %02x:%02x:%02x:%02x:%02x:%02x, Frequency %dMHz\n",
			network.ssidLength, network.ssid, network.bssid[0], network.bssid[1],
			network.bssid[2], network.bssid[3], network.bssid[4], network.bssid[5],
			network.frequencyMHz);
		GPIO_SetValue(gpioWiFiLedFd, GPIO_Value_Low);
	}
}

/// <summary>
///     Check WiFi status.
/// </summary>
///	<return>
///		True for WiFi Connected, False for Not Connected.
static bool IsWiFiConnected(void)
{
	WifiConfig_ConnectedNetwork network;
	int result = WifiConfig_GetCurrentNetwork(&network);
	if (result < 0)
	{
		WiFiErrorCheckLog("IsWiFiConnected", "WifiConfig_GetCurrentNetwork", NULL, result);
		return false;
	}
	else
	{
		Log_Debug("[INFO - IsWiFiConnected] Currently WiFi is connected");
		Log_Debug("[INFO - IsWiFiConnected] SSID \"%.*s\", BSSID %02x:%02x:%02x:%02x:%02x:%02x, Frequency %dMHz\n",
			network.ssidLength, network.ssid, network.bssid[0], network.bssid[1],
			network.bssid[2], network.bssid[3], network.bssid[4], network.bssid[5],
			network.frequencyMHz);
		return true;
	}
}

/// <summary>
///     Scan Wifi network and Show its info.
/// </summary>
static void PrintScanFoundNetworks(void)
{
	int result = WifiConfig_TriggerScanAndGetScannedNetworkCount();
	if (result < 0)
	{
		WiFiErrorCheckLog("PrintScanFoundNetworks", "WifiConfig_TriggerScanAndGetScannedNetworkCount", NULL, result);
	}
	else if (result == 0)
	{
		Log_Debug("[INFO - PrintScanFoundNetworks] Scan found no WiFi network.\n");
	}
	else
	{
		size_t networkCount = (size_t)result;
		Log_Debug("[INFO - PrintScanFoundNetworks] Scan found %d WiFi networks:\n", result);
		WifiConfig_ScannedNetwork *networks =
			(WifiConfig_ScannedNetwork *)malloc(sizeof(WifiConfig_ScannedNetwork) * networkCount);
		result = WifiConfig_GetScannedNetworks(networks, networkCount);
		if (result < 0)
		{
			WiFiErrorCheckLog("PrintScanFoundNetworks", "WifiConfig_GetScannedNetworks", NULL, result);
		}
		else
		{
			// Log SSID, signal strength and frequency of the found WiFi networks
			networkCount = (size_t)result;
			for (size_t i = 0; i < networkCount; ++i)
			{
				Log_Debug("[INFO - PrintScanFoundNetworks] %3d) SSID \"%.*s\", Signal Level %d, Frequency %dMHz\n", i,
					networks[i].ssidLength, networks[i].ssid, networks[i].signalRssi,
					networks[i].frequencyMHz);
			}
		}
		free(networks);
	}
}

/// <summary>
///     Helper function to send a fixed message via the given UART.
/// </summary>
/// <param name="uartFd">The open file descriptor of the UART to write to</param>
/// <param name="dataToSend">The data to send over the UART</param>
static void SendUartMessage(int uartFd, const char *dataToSend)
{
	size_t totalBytesSent = 0;
	size_t totalBytesToSend = strlen(dataToSend);
	int sendIterations = 0;
	while (totalBytesSent < totalBytesToSend)
	{
		sendIterations++;

		// Send as much of the remaining data as possible
		size_t bytesLeftToSend = totalBytesToSend - totalBytesSent;
		const char *remainingMessageToSend = dataToSend + totalBytesSent;
		ssize_t bytesSent = write(uartFd, remainingMessageToSend, bytesLeftToSend);
		Log_Debug("[INFO - SendUartMessage] write ble to device from azure sphere %d bytes\n", bytesSent);
		if (bytesSent < 0)
		{
			Log_Debug("[ERROR - SendUartMessage] Could not write to UART: %s (%d).\n", strerror(errno), errno);
			terminationRequired = true;
			return;
		}

		totalBytesSent += (size_t)bytesSent;
	}

	Log_Debug("[INFO - SendUartMessage] Sent %zu bytes over UART in %d calls.\n", totalBytesSent, sendIterations);
}

/// <summary>
///     le button timer event: if the button is pressed, send data over the UART.
/// </summary>
static void ButtonTimerEventHandler(event_data_t *eventData)
{
	if (ConsumeTimerFdEvent(gpioButtonTimerFd) != 0)
	{
		terminationRequired = true;
		return;
	}

	// Check for a button press
	GPIO_Value_Type newButtonState;
	int result = GPIO_GetValue(gpioButtonFd, &newButtonState);
	if (result != 0)
	{
		Log_Debug("[ERROR - ButtonTimerEventHandler] Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	// If the button has just been pressed, send data over the UART
	// The button has GPIO_Value_Low when pressed and GPIO_Value_High when released
	if (newButtonState != buttonState)
	{
		if (newButtonState == GPIO_Value_Low)
		{
			const char *messageToSend = "Hello world!|";
			SendUartMessage(uartFd, messageToSend);
		}
		buttonState = newButtonState;
	}
}
/// <summary>
///     Handle UART event: if there is incoming data, print it, and blink the LED.
/// </summary>
static void UartEventHandler(void)
{
	//Azure sphere read 12bytes at once. Need to concatenate the message
	//if not, you will see message divided per 12 bytes. Check UART Sample.
	char *messageLine = NULL;
	char *messageSegment = &receiveBuffer[nBytesInBuffer];

	// Poll the UART and store the byte(s) behind already received bytes
	ssize_t nBytesRead = read(uartFd, (void *)messageSegment, RECEIVE_BUFFER_SIZE - nBytesInBuffer);

	if (nBytesRead < 0)
	{
		Log_Debug("[ERROR - UartEventHandler] Could not read UART: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (nBytesRead > 0)
	{
		nBytesInBuffer += (size_t)nBytesRead;
		char *messageLineEnd = NULL;

		// if you use on one char, you can make more faster by replacing strstr to memchr
		while ((messageLineEnd = (char *)strstr(receiveBuffer, LINE_DELIMITER)) != NULL)
		{
			for (int i = 0; i < strlen(LINE_DELIMITER); i++)
			{
				// replace LINE_DELIMITER with string terminator and advance to 'next' line
				*messageLineEnd++ = '\0';
			}
			size_t nLineLength = (size_t)(messageLineEnd - receiveBuffer);
			if (nLineLength > 1)
			{
				// create string for received line
				messageLine = (char *)malloc(nLineLength);
				memcpy((void *)messageLine, (const void *)receiveBuffer, nLineLength); // faster than strncpy
			}

			// move remaining bytes (if any) to begin of buffer
			nBytesInBuffer -= nLineLength;
			if (nBytesInBuffer > 0)
			{
				memcpy((void *)receiveBuffer, (const void *)messageLineEnd, nBytesInBuffer);
			}

			// handle received line and release memory
			if (messageLine != NULL)
			{
				Log_Debug("[INFO - UartEventHandler] Read Data : %s\n", messageLine);
				// now we split the pszLine and set the WiFi Settings
				// while JSON is the correct method using something like Parsons, I this is a simple demo, so we will split at :
				char *divider = NULL;
				while ((divider = (char *)memchr(messageLine, WIFI_SSID_PW_DELIMITER[0], strlen(messageLine))) != NULL)
				{
					char *tokens;
					char *networkName = NULL;
					char *networkPassword = NULL;
					char *sendMessage;
					int wifiAddResult = -1, count = 0;
					bool wifiConnectSuccess;
					tokens = strtok(messageLine, WIFI_SSID_PW_DELIMITER);
					networkName = tokens;
					tokens = strtok(NULL, WIFI_SSID_PW_DELIMITER);
					networkPassword = tokens;
					Log_Debug("networkName : %s - networkPassword : %s\n", networkName, networkPassword);

					DeleteAllWifiNetwork();
					if (networkPassword == NULL)
						wifiAddResult = AddOpenWifi(networkName);
					else
						wifiAddResult = AddWpa2Wifi(networkName, networkPassword);
					Log_Debug("WiFi add result %d\n", wifiAddResult);
					while (1)
					{ //Check WiFi can connect while 10 sec.
						wifiConnectSuccess = IsWiFiConnected();
						Log_Debug("wifi connect result : %d  count : %d\n", wifiConnectSuccess, count);
						sleep(1);
						if (wifiConnectSuccess || count++ == 10)
							break;
					}
					if (wifiAddResult == 0 && wifiConnectSuccess)
					{
						sendMessage = "Success";
						GPIO_SetValue(gpioWiFiLedFd, GPIO_Value_Low);
					}
					else
					{
						sendMessage = "Fail";
						GPIO_SetValue(gpioWiFiLedFd, GPIO_Value_High);
					}
					SendUartMessage(uartFd, sendMessage);
				}
				free(messageLine);
			}
		}
	}
}
// event handler data structures. Only the event handler field needs to be populated.
static event_data_t buttonEventData = { .eventHandler = &ButtonTimerEventHandler };
static event_data_t uartEventData = { .eventHandler = &UartEventHandler };

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = TerminationHandler;
	sigaction(SIGTERM, &action, NULL);

	epollFd = CreateEpollFd();
	if (epollFd < 0)
	{
		return -1;
	}

	// Create a UART_Config object, open the UART and set up UART event handler
	UART_Config uartConfig;
	UART_InitConfig(&uartConfig);
	uartConfig.baudRate = 9600;
	uartConfig.flowControl = UART_FlowControl_None;
	uartFd = UART_Open(MT3620_RDB_HEADER2_ISU0_UART, &uartConfig);
	if (uartFd < 0)
	{
		Log_Debug("[ERROR - InitPeripheralsAndHandlers] Could not open UART: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	if (RegisterEventHandlerToEpoll(epollFd, uartFd, &uartEventData, EPOLLIN) != 0)
	{
		return -1;
	}
	// Open button GPIO as input, and set up a timer to poll it
	Log_Debug("Opening MT3620_RDB_BUTTON_A as input.\n");
	gpioButtonFd = GPIO_OpenAsInput(MT3620_RDB_BUTTON_A);
	if (gpioButtonFd < 0)
	{
		Log_Debug("[ERROR - InitPeripheralsAndHandlers] Could not open button GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	struct timespec buttonPressCheckPeriod = { 0, 1000000 };
	gpioButtonTimerFd =
		CreateTimerFdAndAddToEpoll(epollFd, &buttonPressCheckPeriod, &buttonEventData, EPOLLIN);
	if (gpioButtonTimerFd < 0)
	{
		return -1;
	}

	// Open LED GPIO and set as output with value GPIO_Value_High (off)
	Log_Debug("Opening MT3620_RDB_LED1_BLUE for checking WiFi Connection.\n");
	gpioWiFiLedFd = GPIO_OpenAsOutput(MT3620_RDB_LED1_BLUE, GPIO_OutputMode_PushPull, GPIO_Value_High);
	if (gpioWiFiLedFd < 0)
	{
		Log_Debug("[ERROR - InitPeripheralsAndHandlers] Could not open LED GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
	// Leave the LED off
	if (gpioWiFiLedFd >= 0)
	{
		GPIO_SetValue(gpioWiFiLedFd, GPIO_Value_High);
	}

	Log_Debug("Closing file descriptors.\n");
	CloseFdAndPrintError(gpioWiFiLedFd, "gpioWiFiLedFd");
	CloseFdAndPrintError(gpioButtonTimerFd, "ButtonTimer");
	CloseFdAndPrintError(gpioButtonFd, "GpioButton");
	CloseFdAndPrintError(uartFd, "Uart");
	CloseFdAndPrintError(epollFd, "Epoll");
}
/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{
	Log_Debug("UART application starting\n");
	if (InitPeripheralsAndHandlers() != 0)
	{
		terminationRequired = true;
	}
	PrintStoredWiFiNetworks();
	PrintCurrentlyConnectedWiFiNetwork();
	PrintScanFoundNetworks();
	// Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
	Log_Debug("Waiting for UART messages");

	while (!terminationRequired)
	{
		if (WaitForEventAndCallHandler(epollFd) != 0)
		{
			terminationRequired = true;
		}
	}
	ClosePeripheralsAndHandlers();
	Log_Debug("Application exiting\n");
	return 0;
}
