/**
 * @file ESP32.hpp
 * @brief CPP public header for esp32 low level driver
 * @date Created on: Mar 21, 2023
 * @author:  Matteo Vittorio Ricciutelli
 */

#ifndef INC_ESP32_HPP_
#define INC_ESP32_HPP_

#include <MCAL_iGpio.hpp> ///< imports gpio abstraction layer to manipulate individual pins

#include <MCAL_iUart.hpp> ///< imports uart abstraction layer to use the uart peripheral
#include <UTILS_Time.h>         ///< imports time formats and functions

#define ESP32C3_RX_BUFFER_SIZE 2048
#define ESP32C3_TX_BUFFER_SIZE 2048
#define ESP32C3_MAX_WS_SOCKET 3
#define SECURED_ESP32 0
#define DEFAULT_WEBSOCKET_INTERNAL_BUFFER_SIZE 1024

typedef struct tm Time_t;
namespace DRV
{

/**
 * @brief esp32 low level AT-commands driver class
 * it's purpose is to format and send parametric at commands via uart to an esp32
 * and manage the response reception via isr callbacks
 * @note to suit this object in the main application, the developer shall write the implementation for
 *  @ESP32_Callback function which is used to relay outcome of the given commands
 */
class ESP32C3
{
public:
    ///@brief esp32 module interaction events
    enum Event:uint8_t
    {
        RespReceived,       ///< expected response to previous query fully received
        RespErr,            ///< received a command ERROR
        HwErr,              ///< reported error by the module
        UnsolicitedMsgBegin,///< unsolicited message from the module receive begin
        UnsolicitedMsgCont,///< unsolicited message from the module receive continuation
        UnsolicitedMsgEnd,  ///< unsolicited message receive complete
        ExpectedIPD         ///expected IPD (usually unsolicited)
    };
    ///@brief esp32 supported wifi modes
    enum WiFiMode:uint8_t
    {
        Disable =           0,
        Station =           1,
        SoftAP =            2,
        SoftAp_Station =    3,
    };
    ///@brief esp32 supported wifi encryption
    enum WifiEncryption:uint8_t
    {
        Open            = 0,
        WPA_PSK         = 2,
        WAP2_PSK        = 3,
        WPA_WPA2_PSK    = 4
    };
    ///@brief range of channels for wifi offered by the esp32
    enum WifiChannelIds:uint8_t
    {
        ch1 = 1,
        ch2,
        ch3,
        ch4,
        ch5,
        ch6,
        ch7
    };
    ///@brief server commands table
    enum ServerMode
    {
        Delete_Server = 0,
        Create_Server = 1
    };
    ///@brief enumeration of supported connection types
    enum ConnectionType
    {
        TCP,
        UDP,
        SSL,
        ConnectionType_Max
    };
    ///@brief link id parameter enumeration for certain at commands
    enum LinkId:uint8_t
    {
        LinkId0 = 0,
        LinkId1 = 1,
        LinkId2 =2,
        LinkId3 = 3,
        LinkIdMax = 4,
        LinkIdCloseAll = 5,
        LinkIdInvalid = 0xDD,
        LinkId_MuxIsZero = 0xFF
    };

    ///@brief esp32 send command modes supported by the send command
    enum SendMode
    {
        SendMode_SendAndReturnToNormalMode = 0,
        SendMode_SendAndRemainInSendMode = 1,
    };
    ///@brief esp32 unsolicited commuinications
    enum UnsolicitedMessage:uint8_t
    {
        SocketConnection,
        SocketClose,
        WifiClientConnection,
        WifiClientDisconnection,
        WifiIpClient,
        WifiConnection,
        WifiDisconnection,
        IpAssignment,
        IPD,
        WebSocketData,
        WebsocketConnect,
        WebsocketDisconnect,
        WebsocketClosed,
        //----------//
        Unknown
    };
    ///@brief define websocket exchanged data type @https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/websocket_at_commands.html#cmd-wscfg
    enum WebSockedtOpCode_t:uint8_t
    {
        ContinuationFrame = 0,
        TextFrame = 1,
        BinaryFrame = 2,
        ConnectionCloseFrame = 8,
        PingFrame = 9,
        PongFrame = 10,
    };
    ///@brief authentication mode for secure secokets, see https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/AT_Command_Set/TCP-IP_AT_Commands.html#id93
    enum AuthMode_t:uint8_t
    {
        NoAuthentication = 0,
        ClientCertificate = 1,
        ServerCertificate = 2,
        ClientAndServerCertificate = 3,
    };
    ///@brief possible operation type used by AT+SYSFLASH command
    enum FlashOperation_t:uint8_t
    {
        EraseSector = 0,
        WriteDataIntoUserPartition = 1,
        ReadDataFromUserPartition = 2
    };
    ///@brief from https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/AT_Command_Set/Wi-Fi_AT_Commands.html#at-cwstate-query-the-wi-fi-state-and-wi-fi-information
    enum ConnectionState_t:uint8_t
    {
        NotStarted = 0,
        Connected_NoIp = 1,
        Connected_IpObtained = 2,
        Reconnecting = 3,
        Disconnected = 4,
        ConnectionStateMax,///< invalid value used to check for validity
    };
    ///@brief esp32 response characteristics
    typedef struct
    {
        const uint8_t * string;
        uint16_t stringLen;
    }ResponseInfo_t;

    ESP32C3( MCAL::iUart* _huart, MCAL::iGpio* _enPin);
    void UartCallback();
    void UartErrCallback();
    bool Send_AT();
    bool Send_AT_SYSSTORE(bool enable);
    bool Send_AT_CIPSNTPTIME();
    bool Parse_AT_CIPSNTPTIME(Time_t* timeStruct);
    bool Send_AT_CWQAP();
    bool Parse_AT(uint8_t* resp);
    bool Send_AT_CWMODE(WiFiMode mode);
    bool Parse_AT_CWMODE();
    bool Send_AT_CWSAP(uint8_t* ssid, uint8_t* pwd, WifiChannelIds channel_id, WifiEncryption encryption, uint8_t max_conn, bool hidden);
    bool Parse_AT_CWSAP();
    bool Send_AT_CWJAP(uint8_t* ssid, uint8_t* pwd);
    bool Parse_AT_CWJAP();
    bool Send_AT_CIPMUX(bool enable_multiple_connections);
    bool Parse_AT_CIPMUX();
    bool Send_AT_CIPSERVER(ServerMode mode, uint16_t port);/*the ip address of the server must be queried with CIFSR or must be set via AT+CIPAP*/
    bool Parse_AT_CIPSERVER();
    bool Send_AT_CIPSERVERMAXCONN(uint8_t num);
    bool Send_AT_CIFSR();
    bool Parse_AT_CIFSR();
    bool Send_AT_CIPRECVTYPE(LinkId link_id, bool bufferedMode);
    bool Send_AT_CIPRECVDATA(LinkId link_id, uint32_t maxLen);
    bool Parse_AT_CIPRECVDATA(uint8_t* outputBuffer, uint32_t maxLen);
    bool Send_AT_CWSTATE();
    bool Parse_AT_CWSTATE(ConnectionState_t* state);
    bool Send_AT_CIPSTART(LinkId link_id, ConnectionType conn_type, uint8_t* ip, uint16_t port, uint32_t local_port = 0xFFFFFFFF);
    bool Parse_AT_CIPSTART();
    bool SendData(const uint8_t* data);
    bool SendWebSocketData(const uint8_t* data);
    bool Parse_SendData();
    bool Send_AT_CIPSEND(LinkId link_id, uint32_t len);
    bool Parse_AT_CIPSEND();
    bool Send_AT_CIPCLOSE(LinkId link_id);
    bool Parse_AT_CIPCLOSE();
    bool Send_AT_CIPRECEIVELEN();
    bool Parse_AT_CIPRECEIVELEN(LinkId link, uint32_t* len);
    bool GetClientConnectionId(uint8_t* buffer, ESP32C3::LinkId* retVal );
    //---- FileSystem ----//
    bool Send_AT_SYSFLASH(FlashOperation_t operation,
            uint8_t* partition,
            uint32_t offset,
            uint32_t length);
    //---- Websocket -----//
    bool Send_AT_WSCFG(LinkId link,
            uint32_t ping_interval,
            uint32_t ping_timeout,
            uint32_t buffer_size = DEFAULT_WEBSOCKET_INTERNAL_BUFFER_SIZE,
            AuthMode_t  auth_mode= NoAuthentication ,
            uint32_t pki_number = 0,
            uint32_t ca_number =0 );
    bool Send_AT_WSHEAD(uint32_t length);
    bool Send_WsHeader(const uint8_t* data);
    bool Parse_AT_WSHEAD();
    bool Parse_AT_WSHEAD_GET(uint32_t* index, uint8_t* header_ptr);
    bool Send_AT_WSOPEN(LinkId link, const uint8_t *uri, const uint8_t* sub_protocol , const uint8_t* auth );
    bool Parse_AT_WSOPEN(LinkId* link);
    bool Send_AT_WSSEND(LinkId link, uint32_t length, WebSockedtOpCode_t opcode);
    bool Send_AT_WSCLOSE(LinkId link);
    bool GetWsClientConnectionId(uint8_t* buffer, ESP32C3::LinkId* retVal );
    bool GetWsClientDisconnectionId(uint8_t* buffer, ESP32C3::LinkId* retVal );
    bool GetWsClientClosedId(uint8_t* buffer, ESP32C3::LinkId* retVal );
    uint8_t* GetWsData(uint8_t* data, uint8_t** wsStart, uint32_t* dataLength,uint32_t dataMaxLength, ESP32C3::LinkId *id);
    //----------------------//
    UnsolicitedMessage GetUnsolicitedMessage(uint8_t* buffer, uint32_t* len);
    uint8_t* GetIPD(uint8_t* data, uint8_t** ipdStart, uint32_t* dataLength, uint32_t dataMaxLength, ESP32C3::LinkId *id);

    bool WaitData();
    void SetUrcReadiness(bool setting);
    void Enable(bool en);
    void StartReset();
    void StopReset();
    void FlushRx();
    bool EscapeStringParameter(const uint8_t* input, uint8_t* output, uint32_t max_output_length);
    uint32_t GetRxBufferData(uint8_t* output, uint32_t maxLen);
private:
#if 0
    enum commands_t
    {
        AT,
        AT_GMR,
    };
#endif
    ///@brief dedicated uart line for esp32 communication
    MCAL::iUart* huart;
    ///@brief module enable pin
    MCAL::iGpio* enPin;
    ///@brief pointer to the list of expected responses  that the esp32 can issue to a given command, the list varies depending on the command
    const ResponseInfo_t* expectedResponsesList = nullptr;
    ///@brief how many responses are listed in the expectedResponsesList
    uint16_t expectedResponsesListLength = 0;
    ///@brief received bytes counter
    uint16_t rx_index = 0;
    bool SendCommand(uint32_t len);
    ///@brief transmit data buffer
    volatile uint8_t tx_buffer[ESP32C3_TX_BUFFER_SIZE];
    ///@brief receive data buffer
    volatile uint8_t rx_buffer[ESP32C3_RX_BUFFER_SIZE];
    ///@brief how many bytes are to be streamed to esp32 used by @Send_AT_CIPSEND and @Send_AT_SYSFLASH and @Send_AT_WSSEND
    uint32_t send_data_length = 0;
    ///@brief  used by  @Send_AT_WSHEAD
    uint32_t ws_header_length = 0;
    ///@brief unsolicited result codes readiness (module keeps listening on uart)
    bool urc_readiness = false;
#if 0
    commands_t last_command = AT;
#endif
};

};/* namespace DRV*/
void ESP32_Callback(DRV::ESP32C3::Event event, DRV::ESP32C3* caller, uint8_t* data, uint32_t size);

#endif /* INC_ESP32_HPP_ */
