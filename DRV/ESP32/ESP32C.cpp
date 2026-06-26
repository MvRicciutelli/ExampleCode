/**
 * @file  ESP32.cpp
 * @brief CPP implementation of a low level driver for esp32 AT command interface
 * @date Created on: Mar 21, 2023
 * @author:  Matteo Vittorio Ricciutelli
 */

#include "ESP32C.hpp"        ///< main header for the class
#include "Debug_Print.hpp"  ///< debug print functionality
#include "stdio.h"
#include "string.h"

#define _STA_DISCONNECTED_LEN   17
#define _STA_CONNECTED_LEN      14
#define _DIST_IP_CLIENT_LEN     12
#define _SOCKET_NUMBER_OFFSET   1
#define ESP32_SSID_MAX_LEN      64
#define ESP32_PWD_MAX_LEN       64
#define ESP32_IP_MAX_LEN        312
#define ESP32_URI_MAX_LEN       312
#define ESP32_DEFAULT_WS_TIMEOUT_MS 15000
///@brief macro to define the offset at which the state value is found used in @Parse_AT_CWSTATE
#define CW_STATE_STATE_OFFSET 9

using namespace DRV;
///@brief standard timeout in milliseconds for a esp32 response
#define ESP32_DEFAULT_TIMEOUT_MS 100
///@brief table for connection types string used as input for certain AT commands
const char* ConnectionTypeMap[ESP32C3::ConnectionType_Max] = {"TCP", "UDP", "SSL"};
///@brief expected string output after a successful send command
#if __ESP8266__
const uint8_t expected_SENDREADY[]                 = "OK\r\n> ";
#else
const uint8_t expected_SENDREADY[]                 = "OK\r\n\r\n>";
#endif
///@brief expected string output after a successful generic command
const uint8_t expected_OK[]                        = "OK\r\n";
///@brief expected string output after a successful CIPRECVDATA Send command
const uint8_t expected_CIPRECVDATA[]                        = "+CIPRECVDATA:\r\n\r\n";
///@brief expected string output after a successful Send command
const uint8_t expected_SendOK[]                        = "SEND OK\r\n";
///@brief expected string output after a successful Send command
const uint8_t expected_WsSendOK[]                        = "SEND OK\r\n";
///@brief expected string output after a successful WSOPEN command
const uint8_t expected_WsConnected[]                   = "+WS_CONNECTED:";
///@brief websocket disconnection ie:
const uint8_t expected_WsDisconnected[]             = "+WS_DISCONNECTED:";
///@brief websocket disconnection ie:
const uint8_t expected_WsClosed[]                   = "+WS_CLOSED:";
///@brief expected string output after a bad command (error or wrong format)
const uint8_t expected_ERROR[]                     = "ERROR\r\n";
///@brief expected string esp32 provides upon receiving data from the outside
const uint8_t expected_IPD[]                       = "\r\n";
///@brief Unsolicited message when an open socxket receives data
const uint8_t expected_IPD_Header[]             =    "+IPD";
///@brief Unsolicited message when an open websockettransmits data to the esp32
const uint8_t expected_WSDATA_Header[]             =    "+WS_DATA:";
///@brief Unsolicited message when a socket is opened by a client (esp32 is a server), X is the link id
const uint8_t expected_client_connection[]         = "X,CONNECT\r\n";
///@brief Unsolicited message when a socket is closed by a client (esp32 is a server), X is the link id
const uint8_t expected_socket_close[]              = "X,CLOSED\r\n";
///@brief Unsolicited message when a device connects to the esp32c hotspot, XX are mac address numbers
const uint8_t expected_wifi_client_connection[]    = "+STA_CONNECTED:\"XX:XX:XX:XX:XX:XX\"\r\n";
///@brief Unsolicited message when a device disconnects to the esp32c hotspot, XX are mac address numbers
const uint8_t expected_wifi_client_disconnection[] = "+STA_DISCONNECTED:\"XX:XX:XX:XX:XX:XX\"\r\n";
///@brief Unsolicited message when a device connects to the esp32c hotspot, XX are mac address numbers YY is the ip address last number
const uint8_t expected_wifi_ip_client[]            = "+DIST_STA_IP:\"XX:XX:XX:XX:XX:XX\",\"192.168.0.YY\"\r\n";
///@brief Unsolicited message when the esp32 connects to a wifi
const uint8_t expected_connection_to_wifi[]        = "WIFI CONNECTED\r\n";
///@brief Unsolicited message when the esp32 connects to a wifi
const uint8_t expected_disconnection_to_wifi[]     = "WIFI DISCONNECT\r\n";
///@brief Unsolicited message when esp32c receives his ip address within a subnet
const uint8_t expected_ip_assignment[]             = "WIFI GOT IP\r\n";
///@brief expected string output after a successful CWSTATE command
const uint8_t expected_CWSTATE[]                        = "+CWSTATE:";
///@brief commonly used response for simple commands which return Ok or ERROR
static const ESP32C3::ResponseInfo_t Basic_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
/**
 * @brief ESP32C3 object constructor, it gets all required dependencies to run it does not use nrstPin
 *
 * @param _huart
 * @param _enPin
 */
ESP32C3::ESP32C3( MCAL::iUart* _huart, MCAL::iGpio* _enPin):huart(_huart),enPin(_enPin)
{}

/**
 * @brief identifies the unsolicited message format in a buffer
 *
 * @param buffer
 * @param length of the unsolicited message
 * @return @UnsolicitedMessage message type
 */
ESP32C3::UnsolicitedMessage ESP32C3::GetUnsolicitedMessage(uint8_t* buffer , uint32_t* len )
{
    ///@brief form the possible expected unsolicited messages list
    static const ESP32C3::ResponseInfo_t unsolicited_expectedResponses[] =
    {
         [ESP32C3::SocketConnection] =          {expected_client_connection,         sizeof(expected_client_connection)-1},
         [ESP32C3::SocketClose] =               {expected_socket_close,              sizeof(expected_socket_close)-1},
         [ESP32C3::WifiClientConnection] =      {expected_wifi_client_connection,    sizeof(expected_wifi_client_connection)-1},
         [ESP32C3::WifiClientDisconnection] =   {expected_wifi_client_disconnection, sizeof(expected_wifi_client_disconnection)-1},
         [ESP32C3::WifiIpClient] =              {expected_wifi_ip_client,            sizeof(expected_wifi_ip_client)-1},
         [ESP32C3::WifiConnection] =            {expected_connection_to_wifi,        sizeof(expected_connection_to_wifi)-1},
         [ESP32C3::WifiDisconnection] =         {expected_disconnection_to_wifi,     sizeof(expected_disconnection_to_wifi)-1},
         [ESP32C3::IpAssignment] =              {expected_ip_assignment,             sizeof(expected_ip_assignment)-1},
         [ESP32C3::IPD] =                       {expected_IPD_Header,                sizeof(expected_IPD_Header)-1},
         [ESP32C3::WebSocketData] =             {expected_WSDATA_Header,             sizeof(expected_WSDATA_Header)-1},
         [ESP32C3::WebsocketConnect] =          {expected_WsConnected,               sizeof(expected_WsConnected)-1},
         [ESP32C3::WebsocketDisconnect] =       {expected_WsDisconnected,            sizeof(expected_WsDisconnected)-1},
         [ESP32C3::WebsocketClosed] =           {expected_WsClosed,                  sizeof(expected_WsClosed)-1},
    };
    ///@brief how many entries in the expected unsolicited messages list
    static uint32_t unsolicited_expectedResponsesLength = (sizeof(unsolicited_expectedResponses))/(sizeof(unsolicited_expectedResponses));
    // init length at 0
    *len = 0;
    // init result as unknown
    ESP32C3::UnsolicitedMessage result = ESP32C3::UnsolicitedMessage::Unknown;
    if(buffer == nullptr){
        buffer = (uint8_t*)rx_buffer;
    }
    if(
        0 == memcmp(buffer+_SOCKET_NUMBER_OFFSET,
            unsolicited_expectedResponses[ESP32C3::SocketConnection].string+_SOCKET_NUMBER_OFFSET,
            unsolicited_expectedResponses[ESP32C3::SocketConnection].stringLen -_SOCKET_NUMBER_OFFSET))
    {
        *len = unsolicited_expectedResponses[SocketConnection].stringLen;
        result = ESP32C3::SocketConnection;
    }
    else if((
                0 == memcmp(buffer+_SOCKET_NUMBER_OFFSET,
                    unsolicited_expectedResponses[ESP32C3::SocketClose].string+_SOCKET_NUMBER_OFFSET,
                    unsolicited_expectedResponses[ESP32C3::SocketClose].stringLen -_SOCKET_NUMBER_OFFSET)))
    {
        *len = unsolicited_expectedResponses[SocketClose].stringLen;
        result = ESP32C3::SocketClose;
    }
    else if((
                0 == memcmp(buffer,
                    unsolicited_expectedResponses[ESP32C3::WifiClientConnection].string,
                    _STA_CONNECTED_LEN)))
    {
        *len = unsolicited_expectedResponses[WifiClientConnection].stringLen;
        result = ESP32C3::WifiClientConnection;
    }
    else if((
                0 == memcmp(buffer,
                    unsolicited_expectedResponses[ESP32C3::WifiClientDisconnection].string,
                    _STA_DISCONNECTED_LEN)))
    {
        *len = unsolicited_expectedResponses[WifiClientDisconnection].stringLen;
        result = ESP32C3::WifiClientDisconnection;
    }
    else if((
                0 == memcmp(buffer,
                    unsolicited_expectedResponses[ESP32C3::WifiIpClient].string,
                    _DIST_IP_CLIENT_LEN)))
    {
        *len = unsolicited_expectedResponses[WifiIpClient].stringLen;
        result = ESP32C3::WifiIpClient;
    }
    else if((
                0 == memcmp(buffer,
                    unsolicited_expectedResponses[ESP32C3::WifiConnection].string,
                    unsolicited_expectedResponses[ESP32C3::WifiConnection].stringLen)))
    {
        *len = unsolicited_expectedResponses[WifiConnection].stringLen;
        result = ESP32C3::WifiConnection;
    }
    else if((
                0 == memcmp(buffer,
                    unsolicited_expectedResponses[ESP32C3::WifiDisconnection].string,
                    unsolicited_expectedResponses[ESP32C3::WifiDisconnection].stringLen)))
    {
        *len = unsolicited_expectedResponses[WifiDisconnection].stringLen;
        result = ESP32C3::WifiDisconnection;
    }
    else if((
                0 == memcmp(buffer,
                    unsolicited_expectedResponses[ESP32C3::IpAssignment].string,
                    unsolicited_expectedResponses[ESP32C3::IpAssignment].stringLen)))
    {
        *len = unsolicited_expectedResponses[IpAssignment].stringLen;
        result = ESP32C3::IpAssignment;
    }else if(strnstr( (char*)buffer,
                      (char*)unsolicited_expectedResponses[ESP32C3::IPD].string,
                      sizeof(rx_buffer)))
    {
        result = ESP32C3::IPD;
    }else if(strnstr( (char*)buffer,
            (char*)unsolicited_expectedResponses[ESP32C3::WebSocketData].string,
            sizeof(rx_buffer)))
    {
        *len = unsolicited_expectedResponses[WebSocketData].stringLen;
        result = ESP32C3::WebSocketData;
    }
    else if(strnstr( (char*)buffer,
                (char*)unsolicited_expectedResponses[ESP32C3::WebsocketConnect].string,
                sizeof(rx_buffer)))
    {
        *len = unsolicited_expectedResponses[WebsocketConnect].stringLen;
        result = ESP32C3::WebsocketConnect;
    }
    else if(strnstr( (char*)buffer,
                    (char*)unsolicited_expectedResponses[ESP32C3::WebsocketDisconnect].string,
                    sizeof(rx_buffer)))
    {
        *len = unsolicited_expectedResponses[WebsocketDisconnect].stringLen;
        result = ESP32C3::WebsocketDisconnect;
    }
    else if(strnstr( (char*)buffer,
                    (char*)unsolicited_expectedResponses[ESP32C3::WebsocketClosed].string,
                    sizeof(rx_buffer)))
    {
        *len = unsolicited_expectedResponses[WebsocketClosed].stringLen;
        result = ESP32C3::WebsocketClosed;
    }
    return result;
}
#define WS_LINKID_MAX 3
/**
 * @brief parses the Websocket unsolicited result code for connection and converts it into a linkId
 * @param buffer data source
 * @param id output
 * @return true id found false id not found
 */
bool ESP32C3::GetWsClientConnectionId(uint8_t* buffer, ESP32C3::LinkId* retVal )
{
    uint8_t* idPtr = (uint8_t*)strnstr( (char*)buffer,
                            (char*)expected_WsConnected,
                            sizeof(rx_buffer));
    if(idPtr)
    {
        idPtr += 14; //+WS_CONNECTED:X <---X offset
        uint8_t num_id = idPtr[0]-'0';
        if(num_id < (WS_LINKID_MAX) && retVal){
            *retVal = (ESP32C3::LinkId)(num_id);
            return true;
        }else{
            TRACE_DRV_ESP32C("BadWsLinkId%d", buffer[0]);
        }
    }

    return false;
}
/**
 * @brief parses the Websocket unsolicited result code for disconnection and converts it into a linkId
 * @param buffer data source
 * @param id output
 * @return true id found false id not found
 */
bool ESP32C3::GetWsClientDisconnectionId(uint8_t* buffer, ESP32C3::LinkId* retVal )
{
    uint8_t* idPtr = (uint8_t*)strnstr( (char*)buffer,
                            (char*)expected_WsDisconnected,
                            sizeof(rx_buffer));
    if(idPtr)
    {
        idPtr += 17;//+WS_DISCONNECTED:X <---X offset
        uint8_t num_id = idPtr[0]-'0';
        if(num_id < (WS_LINKID_MAX) && retVal){
            *retVal = (ESP32C3::LinkId)(num_id);
            return true;
        }else{
            TRACE_DRV_ESP32C("BadLinkId%d", buffer[0]);

        }
    }

    return false;
}

/**
 * @brief parses the Websocket unsolicited result code for closing the connection and converts it into a linkId
 * @param buffer data source
 * @param id output
 * @return true id found false id not found
 */
bool ESP32C3::GetWsClientClosedId(uint8_t* buffer, ESP32C3::LinkId* retVal )
{
    uint8_t* idPtr = (uint8_t*)strnstr( (char*)buffer,
                            (char*)expected_WsClosed,
                            sizeof(rx_buffer));
    if(idPtr)
    {
        idPtr += 11;//+WS_CLOSED:X <---X offset
        uint8_t num_id = idPtr[0]-'0';
        if(num_id < (WS_LINKID_MAX) && retVal){
            *retVal = (ESP32C3::LinkId)(num_id);
            return true;
        }else{
            TRACE_DRV_ESP32C("BadClsLinkId%d", buffer[0]);

        }
    }

    return false;
}
/**
 * @brief parses the first byte of the input buffer and converts it into a linkId
 * @param buffer
 * @param id output
 * @return true if valid id is found, false otherwise
 */
bool ESP32C3::GetClientConnectionId(uint8_t* buffer, ESP32C3::LinkId* retVal )
{
    uint8_t num_id = buffer[0]-'0';
    if(num_id < (uint8_t)ESP32C3::LinkIdMax && retVal){
        *retVal = (ESP32C3::LinkId)(num_id);
        return true;
    }else{
        TRACE_DRV_ESP32C("BadLinkId%d", buffer[0]);

    }
    return false;
}
/**
 * @brief Analyze response buffer when an +WS_DATA urc is received from the module
 * upon receiving data from an open websocket
 * example:
 * +WS_DATA:<link_id>,<data_len>,<data>
 * @param input data to parse
 * @param output pointer where the websocket +WS_OPEN message starts
 * @param output dataLength
 * @param buffer max size
 * @param output data ptr
 * @return pointer to data beginning or nullptr
 */
uint8_t*  ESP32C3::GetWsData(uint8_t* data, uint8_t** wsStart, uint32_t* dataLength,uint32_t dataMaxLength, ESP32C3::LinkId *id)
{
    *id = LinkIdInvalid;
    uint8_t* ptr = (uint8_t*)strnstr((char*)data,(char*)expected_WSDATA_Header, dataMaxLength );
    if(ptr)//<-- +WS_DATA exists
    {
        *wsStart = ptr;
        if(ptr[9]>= '0' && ptr[9]<= '4')//<--valid link id between 0 and 4
        {
            /* get linkId and length*/
            *id = (LinkId)(ptr[9]-'0');
            *dataLength = atoi((char*) &ptr[11]);
            uint8_t* ptrDataStart = ptr+12;
            /* search for start message token "," after the data length number which is at most 4 chars long*/
            for(uint8_t i = 0; i< 5 ; i++)
            {
                if(*ptrDataStart == ',')
                {
                    /* token found */
                    break;
                }else{
                    ptrDataStart++;
                }
            }

            if(*ptrDataStart == ',')
            {
                /*token found*/
                ptr = ptrDataStart+1;
            }else{
                ptr = nullptr;
                *dataLength = 0;
            }

        }else{
            ptr = nullptr;
            *dataLength = 0;
        }
    }
    return ptr;
}
/**
 * @brief Analyze response buffer when an IPD urc is received return data ptr data length and link id
 *        Example "+IPD,2,6000:hello" LinkId is 2 and length is 6
 * @param input data to parse
 * @param output pointer where the ipd message starts
 * @param output dataLength
 * @param buffer max size
 * @param output data ptr
 * @return pointer to data beginning or nullptr
 */
uint8_t*  ESP32C3::GetIPD(uint8_t* data, uint8_t** ipdStart, uint32_t* dataLength,uint32_t dataMaxLength, ESP32C3::LinkId *id)
{
    *id = LinkId0;

    uint8_t* ptr = (uint8_t*)strnstr((char*)data,(char*)expected_IPD_Header, dataMaxLength );
    if(ptr)
    {
        *ipdStart = ptr;
        if(ptr[6] == ',' && ptr[5]>= '0' && ptr[5]<= '4') //<-- valid link id between 0 and 4
        {
            /* get linkId and length*/
            *id = (LinkId)(ptr[5]-'0');
            *dataLength = atoi((char*) &ptr[7]);
            uint8_t* ptrDataStart = ptr+8;
            /* search for start message token ":"*/
            for(uint8_t i = 0; i< 5 ; i++)
            {
                if(*ptrDataStart == ':')
                {
                    /* token found */
                    break;
                }else{
                    ptrDataStart++;
                }
            }
            if(*ptrDataStart == ':')
            {
                /*token found*/
                ptr = ptrDataStart+1;
            }else{
                ptr = nullptr;
            }

        }
        /* no link id */
        else if(ptr[6] == ':')
        {
            /* no linkId reported, only length*/
            *dataLength = atoi((char*) &ptr[5]);
            uint8_t* ptrDataStart = ptr+6;
            /* search for start message token ":"*/
            for(uint8_t i = 0; i< 5 ; i++)
            {
                if(*ptrDataStart == ':')
                {
                    /*token found*/
                    break;
                }else{
                    ptrDataStart++;
                }
            }
            if(*ptrDataStart == ':')
            {
                ptr = ptrDataStart+1;
            }else{
                /*token not found*/
                ptr = nullptr;
            }
        }else{
            ptr = nullptr;
        }
    }else{
        /* error */
        *dataLength = 0;
        ptr = nullptr;
    }
    return ptr;
}

/**
 * @brief function to call upon uart interrupt reception
 * @details it checks the received string against the list of all possible expected responses
 *  and it then decides whether to stop or to trigger a new receive process (byte-wise)
 * @note this function runs during ISR
 */
void ESP32C3::UartCallback()
{
    bool complete = false;
    /* check if we are not expecting responses to a given command (unsolicited)*/
    if(expectedResponsesList == nullptr)
    {
        /* We are receiving unsolicited bytes from the module     */
        if(rx_index == 0)
        {
            /* inform upper layer that an unsolicited message is being received from the module*/
            ESP32_Callback(Event::UnsolicitedMsgBegin, this, (uint8_t*)rx_buffer, 1);
        }
#if IPD_SELF_DETECTION
        else if(rx_index>=6)
        {

            //TRACE_DRV_ESP32C("!%s\n", rx_buffer);
            /* catch possible IPD - IPD do not post pone /r/n - */
            uint32_t ipdLen = 0;
            LinkId linkIdBuf;
            uint8_t* ipdStartPtr = nullptr;
            uint8_t* ipd_det = GetIPD((uint8_t*)rx_buffer, ipdStartPtr, &ipdLen, sizeof(rx_buffer), &linkIdBuf);
            /* this comparison checks pointer validity*/
            if(ipd_det > rx_buffer)
            {
                uint32_t ipdDataStaIdx = (uint32_t)(ipdStartPtr-rx_buffer);
                if(rx_index - ipdDataStaIdx >= ipdLen)
                {
                    /* we received termination characters on the unsolicited message*/
                    ESP32_Callback(Event::UnsolicitedMsgEnd, this, (uint8_t*)rx_buffer, rx_index);
                }
            }

        }
#endif
        else{
            ESP32_Callback(Event::UnsolicitedMsgCont, this, (uint8_t*)rx_buffer, 1);
        }

    }
    else
    {
        /* We expect a reponse to a given command         */
        /* scan all possible responses strings in the list*/
        for(int i = 0; i<expectedResponsesListLength; i++)
        {
            /* if we have received enough data to match the current response string*/
            if(rx_index+1 >= expectedResponsesList[i].stringLen)
            {
                /* compare the response in the list with the data received*/
                if( 0 == memcmp((char*)expectedResponsesList[i].string,
                        (char*)&rx_buffer[(rx_index+1)-expectedResponsesList[i].stringLen],
                        expectedResponsesList[i].stringLen))
                {
                    /* in case we have a match*/
                    //char buffer[50] = {0};
                    //memcpy(buffer, (char*)rx_buffer, sizeof(buffer)-1);
                    //TRACE_DRV_ESP32C("%s", buffer);

                    /* pre-check event type*/
                    ESP32C3::Event evt = Event::RespReceived;
                    if(expectedResponsesList[i].string == expected_ERROR)
                    {
                        evt = Event::RespErr;
                    }
                    /* if unsolicited messages are supported keep listening otrherwise end uart reception*/
                    if(urc_readiness == false)
                    {
                        /* stop receiving via uart */
                        huart->ReceiveIrqStop();
                    }else
                    {
                        /* void expected responses list attention, break away from for cycle or will hardfault*/
                        expectedResponsesList = nullptr;
                    }
                    /* process will stop*/
                    complete = true;
                    /* invalidate string after completed response*/
                    if((uint16_t)(rx_index+1)<  sizeof(rx_buffer))
                    {
                        rx_buffer[rx_index+1] = 0;
                    }
                    /* buffer byte amount value*/
                    uint32_t rx_index_buffer = rx_index;
                    /* reset receive counter*/
                    rx_index = 0;
                    /* trigger the complete reception callback*/
                    ESP32_Callback(evt , this, (uint8_t*)rx_buffer, rx_index_buffer);

                    /* break away from for cycle*/
                    break;


                }
            }
        }
    }

    if(complete == false )
    {
        /*reception continues, count the received byte*/
        rx_index ++;
        /* overflow guard */
        if(rx_index>= sizeof(rx_buffer))
        {
            rx_index = 0;
        }
        /* trigger reception for 1 additional byte*/
        huart->ReceiveIrqStart((uint8_t*)&rx_buffer[rx_index], 1);
    }else if (true == complete && true == urc_readiness)
    {
        /* trigger reception for first byte */
        huart->ReceiveIrqStart((uint8_t*)&rx_buffer[rx_index], 1);
    }


}
/**
 * @brief called in isr context when there is a uart error
 * @note must be called upon uart error interrupt
 */
void ESP32C3::UartErrCallback()
{
    TRACE_DRV_ESP32C("Uarterr\n");
    /* abort receive*/
    huart->ReceiveIrqStop();
    /* reset byte count*/
    rx_index = 0;
#if 0
    huart->ReceiveIrqStart((uint8_t*)&rx_buffer[0], 1);
#endif
    ESP32_Callback(Event::HwErr, this, (uint8_t*)rx_buffer, rx_index);
}

///@brief sends the AT basic command
bool ESP32C3::Send_AT()
{
    // form ghe possible expected responses list
    static const ResponseInfo_t AT_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT\r\n");
    // physical uart send of the command
    return SendCommand(printedLen);
}

///@brief sends the AT+CWQAP command to disconnect from any AP
bool ESP32C3::Send_AT_CWQAP()
{
    // form ghe possible expected responses list
    static const ResponseInfo_t ATCWQAP_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &ATCWQAP_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(ATCWQAP_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CWQAP\r\n");
    // physical uart send of the command
    return SendCommand(printedLen);
}
///@parse the AT command response
bool ESP32C3::Parse_AT(uint8_t *resp)
{
    return true;
}
/**
 * @brief sends AT+CWSAP command Query the configuration parameters of an ESP32 SoftAP.
 *        https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/Wi-Fi_AT_Commands.html#cmd-sap
 * @param ssid
 * @param pwd
 * @param channel_id
 * @param encryption
 * @param max_conn
 * @param hidden
 * @return
 */
bool ESP32C3::Send_AT_CWSAP(  uint8_t *ssid,
                            uint8_t *pwd,
                            WifiChannelIds channel_id,
                            WifiEncryption encryption,
                            uint8_t max_conn,
                            bool hidden)
{
    /* Sanitize password and ssid */
    uint8_t escaped_ssid[ESP32_SSID_MAX_LEN]   = {0};
    uint8_t escaped_pwd[ESP32_PWD_MAX_LEN]   = {0};
    EscapeStringParameter(ssid, escaped_ssid, ESP32_SSID_MAX_LEN);
    EscapeStringParameter(pwd, escaped_pwd, ESP32_PWD_MAX_LEN);
    // form the possible expected responses list
    static const ResponseInfo_t AT_CWSAP_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CWSAP_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CWSAP_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
#if __ESP8266__
                                   "AT+CWSAP_DEF=\"%s\",\"%s\",%d,%d,%d,%d\r\n",
#else
                                   "AT+CWSAP=\"%s\",\"%s\",%d,%d,%d,%d\r\n",
#endif
                                    escaped_ssid,
                                    escaped_pwd,
                                    channel_id,
                                    encryption,
                                    max_conn,
                                    hidden);
    // physical uart send of the command
    return SendCommand(printedLen);
}
/**
 * @brief examines the AT+CWSAP reponse
 *
 * @return true if OK has been received
 */
bool ESP32C3::Parse_AT_CWSAP()
{
    if(strnstr((char*)rx_buffer,(char*)expected_OK, sizeof(rx_buffer)) != nullptr){
        return true;
    }
    return false;
}

/**
 * @brief AT commands parameters that are strings do not support " , \ as characters and needs to be escaped
 * https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/index.html
 * @param input
 * @param output
 * @param max_output_length
 * @return true no error, false error (length exceeded)
 */
bool ESP32C3::EscapeStringParameter(const uint8_t* input, uint8_t* output, uint32_t max_output_length)
{
    bool retVal = false;
    // consider as legnth all valid characters and also the last 0x00 to close the string
    uint32_t inputLen = strnlen((char*)input, max_output_length)+1;
    uint32_t j = 0; // output counter
    for(uint32_t i = 0; i< inputLen ; i++)
    {
        // check for characters that needs escaping
        if(input[i] == '"' || input[i] == ',' || input[i] == '\\')
        {
            // escape character
            output[j] = '\\';
            // advance output index
            j++;
            // check for overflow
            if(j > max_output_length)
            {
                retVal = false;
                break;
            }
        }
        // copy character from input
        output[j] = input[i];
        // advance output
        j++;
        // check for overflow
        if(j > max_output_length)
        {
            retVal = false;
            break;
        }
    }
    return retVal;
}

/**
 * @brief sends the AT+CWJAP command Connect an ESP32 station to a targeted AP
 * @details https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/Wi-Fi_AT_Commands.html#cmd-jap
 * @param ssid target ssid
 * @param pwd target password
 * @return
 */
bool ESP32C3::Send_AT_CWJAP(uint8_t *ssid, uint8_t *pwd)
{
    /* Sanitize password and ssid */
    uint8_t escaped_ssid[ESP32_SSID_MAX_LEN]   = {0};
    uint8_t escaped_pwd[ESP32_PWD_MAX_LEN]   = {0};
    EscapeStringParameter(ssid, escaped_ssid, ESP32_SSID_MAX_LEN);
    EscapeStringParameter(pwd, escaped_pwd, ESP32_PWD_MAX_LEN);
    // form ghe possible expected responses list
    static const ResponseInfo_t AT_CWJAP_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CWJAP_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CWJAP_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
#if __ESP8266__
                                    "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n",
#else
                                    "AT+CWJAP=\"%s\",\"%s\"\r\n",
#endif
                                    escaped_ssid,
                                    escaped_pwd );
    // physical uart send of the command
    return SendCommand(printedLen);
}
/**
 * @brief parses the received response after a JAP command
 *
 * @return if OK has been received then it's ok
 */
bool ESP32C3::Parse_AT_CWJAP()
{
    if(strnstr((char*)rx_buffer,(char*)expected_OK, sizeof(rx_buffer) ) != nullptr){
        return true;
    }
    return false;
}
/**
 * @brief send AT+CIFSR command Obtain the Local IP Address and MAC Address
 * @details https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/TCP-IP_AT_Commands.html#cmd-ifsr
 * @return
 */
bool ESP32C3::Send_AT_CIFSR()
{
    // form ghe possible expected responses list
    static const ResponseInfo_t AT_CIFSR_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CIFSR_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CIFSR_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIFSR\r\n");
    // physical uart send of the command
    return SendCommand(printedLen);
}
/**
 * @brief incomplete implementation to parse the response to AT+CIFSR
 *
 * @return
 */
bool ESP32C3::Parse_AT_CIFSR()
{
    if(strnstr((char*)rx_buffer,(char*)expected_OK, sizeof(rx_buffer) ) != nullptr ){
        return true;
    }
    return false;
}

/**
 * @brief finalizes the physical transmission and the formatting of an At command
 *  to the ESP32 module
 * @param len
 * @return
 */
bool ESP32C3::SendCommand(uint32_t len)
{
    // stop any ongoing transmission
    huart->ReceiveIrqStop();
    // reset receive index
    rx_index = 0;
    // reset rx buffer content
    memset((char*)rx_buffer,0, sizeof(rx_buffer));
    // check length coherent with buffer capacity
    if(len<sizeof(tx_buffer))
    {
        // make sure last character terminates the string
        tx_buffer[len] = 0;
    }
    TRACE_DRV_ESP32C("Tx->%s",tx_buffer);
    // start receive process
    bool retVal = huart->ReceiveIrqStart((uint8_t*)&rx_buffer[0], 1);
    // check if a send is required
    if(len>0)
    {
        // send via blocking uart
        retVal = huart->TransmitBlocking((uint8_t*)tx_buffer, len, ESP32_DEFAULT_TIMEOUT_MS);
    }
    TRACE_DRV_ESP32C("Snd End");

    // return api call success
    return retVal;

}

/**
 * @brief commences the physical reset of the esp32 module
 *
 */
void ESP32C3::StartReset()
{
    /* disable uart rx*/
    huart->ReceiveIrqStop();
    /* act on the n-reset pin*/
    TRACE_DRV_ESP32C("rstSta");
    enPin->Write(0);

}
/**
 * @brief terminates the physical reset of the module
 *
 */
void ESP32C3::StopReset()
{
   TRACE_DRV_ESP32C("rstEnd");
   enPin->Write(1);

}


/**
 * @brief enables the module
 * @param en
 */
void ESP32C3::Enable(bool en)
{
    enPin->Write(en);

}

/**
 * @brief sends the AT+CIPRECVLEN command to get the received data bytes for every open socket
      https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/TCP-IP_AT_Commands.html#tcpip-at
 * @return
 */
bool ESP32C3::Send_AT_CIPRECEIVELEN()
{
    // form ghe possible expected responses list
    static const ResponseInfo_t AT_CIPRECEIVELEN_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CIPRECEIVELEN_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CIPRECEIVELEN_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPRECVLEN?\r\n");
    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief for the selected socket id check if the AT+CIPRECVLEN has detected pending bytes in the receive buffer
 * example:"+CIPRECVLEN:<data length of link0>,<data length of link1>,<data length of link2>,<data length of link3>,<data length of link4>\r\nOK\r\n"
 * @param link
 * @param len
 * @return
 */
bool ESP32C3::Parse_AT_CIPRECEIVELEN(LinkId link, uint32_t* len)
{
    bool retval = true;
    /* check for receivelen token */
    char* ptr = strnstr((char*)rx_buffer,"+CIPRECVLEN:", sizeof(rx_buffer));
    if(ptr)
    {
        /* first socket id length is right after ':' so it should be at position +12*/
        if(link == LinkId0)
        {
            *len = atoi(ptr+12);
        }
        else
        {
            /* move from ',' to the next until we reach the socket id number we want*/
            ptr+=12;
            for(uint8_t i = (uint8_t)LinkId1; i<= (uint8_t)link; i++)
            {
                ptr = strnstr((char*)ptr,",", sizeof(rx_buffer));
                if(ptr == nullptr){
                    break;
                }else{
                    ptr++;
                }
            }
            /* if we found the nth ',' the number after it is the desired receive length information*/
            if(ptr)
            {
                *len = atoi((char*)ptr);
            }else{
                *len = 0;
                retval = false;
            }
        }
    }else{
        retval = false;
    }
    return retval;
}

/**
 * @brief sends the at+CWMODE command setting the operating mode of the module
 * @details https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/Wi-Fi_AT_Commands.html#cmd-mode
 * @param mode
 * @return
 */
bool ESP32C3::Send_AT_CWMODE(WiFiMode mode)
{
    // form ghe possible expected responses list
    static const ResponseInfo_t AT_CWMODE_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CWMODE_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CWMODE_expectedResponses))/(sizeof(ResponseInfo_t));
#if __ESP8266__
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CWMODE_CUR=%d\r\n", mode);
#else
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CWMODE=%d\r\n", mode);
#endif
    // physical uart send of the command
    return SendCommand(printedLen);
}
/**
 * @brief parses the response of the AT+CWMODE command
 *
 * @return
 */
bool ESP32C3::Parse_AT_CWMODE()
{
    if(strnstr((char*)rx_buffer,(char*)expected_OK, sizeof(rx_buffer) ) != nullptr ){
        return true;
    }
    return false;
}
/**
 * @brief sends the AT+CIPSERVER command setting up a server or deleting it
 * @details https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/TCP-IP_AT_Commands.html#cmd-server
 * @param mode
 * @param port
 * @return
 */
bool ESP32C3::Send_AT_CIPSERVER(ServerMode mode, uint16_t port)
{
    // form ghe possible expected responses list
    static const ResponseInfo_t AT_CIPSERVER_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CIPSERVER_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CIPSERVER_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPSERVER=%d,%d\r\n", mode, port);
    // physical uart send of the command
    return SendCommand(printedLen);
}
/**
 * @brief parse response to the AT+CIPSERVER command
 *
 * @return
 */
bool ESP32C3::Parse_AT_CIPSERVER()
{
    if(strnstr((char*)rx_buffer,(char*)expected_OK, sizeof(rx_buffer)) != nullptr){
        return true;
    }
    return false;
}
/**
 * @brief commence Wait for the ESP32 to receive and relay data from outside
 * @return
 */
bool ESP32C3::WaitData()
{
    // form ghe possible expected responses list
    static const ResponseInfo_t WaitIPD_expectedResponses[] = {{expected_IPD, sizeof(expected_IPD)-1}};
    expectedResponsesList = &WaitIPD_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(WaitIPD_expectedResponses))/(sizeof(ResponseInfo_t));
    // physical uart receive activation
    return SendCommand(0);
}

/**
 * @brief sends AT+CIPMUIX command to support multiple connections
 * @param enable_multiple_connections
 * @return
 */
bool ESP32C3::Send_AT_CIPMUX(bool enable_multiple_connections)
{

    expectedResponsesList = &Basic_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(Basic_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPMUX=%d\r\n", enable_multiple_connections);
    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief parses the response to AT+CIPMUX command
 * @return
 */
bool ESP32C3::Parse_AT_CIPMUX()
{
    if(strnstr((char*)rx_buffer,(char*)expected_OK, sizeof(rx_buffer)) != nullptr){
        return true;
    }
    return false;
}

/**
 * @brief sends the AT+CWSTATE command to query
 *  wifi connection status information
 *  AT+CWSTATE?
 * @note use @Parse_AT_CWSTATE to parse the response
 * @return true or false for command transmission
 */
bool ESP32C3::Send_AT_CWSTATE()
{
    expectedResponsesList = &Basic_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(Basic_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CWSTATE?\r\n");
    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief parses the output of AT+CWSTATE after @Send_AT_CWSTATE
 * +CWSTATE:<state>,<"ssid">

   OK
 * @param state output variable pointer
 * @return true if a valid response is found, false invalid response (disregard output state)
 */
bool ESP32C3::Parse_AT_CWSTATE(ConnectionState_t* state)
{
    char* ptr = strnstr((char*)rx_buffer,(char*)expected_CWSTATE, sizeof(rx_buffer));
    if(ptr != nullptr)
    {
        *state = (ConnectionState_t)(*(ptr+CW_STATE_STATE_OFFSET)-'0');
        if(*state<ConnectionStateMax)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief   sends the AT+CIPSTART command opening a tcp-udp connection
 * @details https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/TCP-IP_AT_Commands.html#cmd-start
 * @param link_id
 * @param conn_type
 * @param ip
 * @param port valid 0-255, remote port for connection
 * @param [optional] local port (used in udp mode) valid 0-255
 * @return
 * @note if it fails try to use a different link id
 */
bool ESP32C3::Send_AT_CIPSTART(   LinkId link_id,
                                ConnectionType conn_type,
                                uint8_t *ip,
                                uint16_t port,
                                uint32_t local_port )
{
    uint8_t escaped_ip[ESP32_IP_MAX_LEN] = {0};
    EscapeStringParameter(ip, escaped_ip, ESP32_IP_MAX_LEN);
    // form the possible expected responses list
    static const ResponseInfo_t AT_CIPSTART_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CIPSTART_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CIPSTART_expectedResponses))/(sizeof(ResponseInfo_t));
    // considering the presence of absence of link id format the command string and retrieve how long is the command string
    uint32_t printedLen = 0;
    if(link_id == LinkId_MuxIsZero)
    {   // case CIPMUX = 0, no multiple connections
        if(local_port <= 0xFFFF)
        {
            printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPSTART=\"%s\",\"%s\",%d,%lu\r\n",
                                                                                ConnectionTypeMap[conn_type],
                                                                                escaped_ip,
                                                                                port, local_port);
        }else
        {
            printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPSTART=\"%s\",\"%s\",%d\r\n",
                                                                                ConnectionTypeMap[conn_type],
                                                                                escaped_ip,
                                                                                port);
        }
    }
    else
    {
        // case of CPIMUX = 1, multiple connection ids are allowed, its must be specified
        if(local_port<= 0xFFFF)
        {
            printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPSTART=%d,\"%s\",\"%s\",%d,%lu\r\n",
                                                                                link_id,
                                                                                ConnectionTypeMap[conn_type],
                                                                                escaped_ip,
                                                                                port, local_port);
        }
        else
        {
            printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPSTART=%d,\"%s\",\"%s\",%d\r\n",
                                                                                link_id,
                                                                                ConnectionTypeMap[conn_type],
                                                                                escaped_ip,
                                                                                port);
        }

    }

    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief parses the response to AT+CIPSTART command
 *
 * @return
 */
bool ESP32C3::Parse_AT_CIPSTART()
{
    if(strnstr((char*)rx_buffer,(char*)expected_OK, sizeof(rx_buffer)) != nullptr){
        return true;
    }
    return false;
}

/**
 * @brief sends a AT+CIPSEND command to begin to transmit a data payload via an open connection
 *        https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/TCP-IP_AT_Commands.html#cmd-send
 * @param link_id
 * @param len
 * @return
 * @note the esp32 will open a terminal to allow the actual delivery of the payload
 */
bool ESP32C3::Send_AT_CIPSEND(LinkId link_id, uint32_t len)
{
    // form the possible expected responses list
    static const ResponseInfo_t AT_CIPSEND_expectedResponses[] = {{expected_SENDREADY, sizeof(expected_SENDREADY)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CIPSEND_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CIPSEND_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = 0;
    if(len> sizeof(tx_buffer))
    {
        TRACE_DRV_ESP32C("ERROR TX OVERFLOW");
    }
    if(link_id == LinkId_MuxIsZero)
    {
        // case CIPMUX = 0, no multiple connections
        printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPSEND=%d\r\n", (int)len);

    }else
    {
        // case of CPIMUX = 1, multiple connection ids are allowed, its must be specified
        printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPSEND=%d,%d\r\n", (int)link_id, (int)len);
    }
    // update internal variable for transmitting data when send mode is entered
    send_data_length = len;
    // physical uart send of the command
    return SendCommand(printedLen);
}
/**
 * @brief parses the AT+CIPSEND esp32 response
 *
 * @return if send ready is found return ok
 */
bool ESP32C3::Parse_AT_CIPSEND()
{
    if(strnstr((char*)rx_buffer,(char*)expected_SENDREADY, sizeof(rx_buffer) ) != nullptr){
        return true;
    }
    return false;
}

/**
 * @brief Sends AT+CIPCLOSE to close an open socket
 *        https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/TCP-IP_AT_Commands.html#cmd-close
 * @param link_id the same used as when opening the connection with @Send_AT_CIPSTART
 * @return
 */
bool ESP32C3::Send_AT_CIPCLOSE(LinkId link_id)
{
    // form the possible expected responses list
    static const ResponseInfo_t AT_CIPCLOSE_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CIPCLOSE_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CIPCLOSE_expectedResponses))/(sizeof(ResponseInfo_t));
    // considering ID = 5 closes all connections format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPCLOSE=%d\r\n", link_id);
    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief Sends AT+SYSSTORE to configure the parameter retention in flash
 *        https://docs.espressif.com/projects/esp-at/en/release-v2.2.0.0_esp8266/AT_Command_Set/Basic_AT_Commands.html#at-sysstore-query-set-parameter-store-mode
 * @param 0 do not store, 1 store in flash
 * @return
 */
bool ESP32C3::Send_AT_SYSSTORE(bool enable)
{
    // form the possible expected responses list
    static const ResponseInfo_t AT_SYSSTORE_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_SYSSTORE_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_SYSSTORE_expectedResponses))/(sizeof(ResponseInfo_t));
    // considering ID = 5 closes all connections format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+SYSSTORE=%d\r\n", enable);
    // physical uart send of the command
    return SendCommand(printedLen);
}
/**
 * @brief parse AT+CIPCLSOE esp32 response
 *
 * @return
 */
bool ESP32C3::Parse_AT_CIPCLOSE()
{
    if(strnstr((char*)rx_buffer,(char*)expected_OK, sizeof(rx_buffer)) != nullptr){
        return true;
    }
    return false;
}

/**
 * @brief deliver data bytes to the esp32 to be sent via open connection
 *        this should be done after a successful AT+CIPSEND  @Send_AT_CIPSEND
 * @param data
 * @return
 */
bool ESP32C3::SendData(const uint8_t* data)
{
    // form the possible expected responses list
    static const ResponseInfo_t SendData_expectedResponses[] = {{expected_SendOK, sizeof(expected_SendOK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &SendData_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(SendData_expectedResponses))/(sizeof(ResponseInfo_t));
    uint32_t printedLen = 0;
    // data is buffered via transparent mode until the number of bytes set with cipsend is sent to the esp32, then the esp32 will trigger transmission
    printedLen = snprintf((char*)tx_buffer, send_data_length+1, "%s", data); // +1 becaus eof snprintf including the 0x00 terminating character in the count

    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief deliver data bytes to the esp32 to be sent via open websocket connection
 *        this should be done after a successful AT+WSSEND  @Send_AT_WSSEND
 * @param data
 * @return
 */
bool ESP32C3::SendWebSocketData(const uint8_t* data)
{
    // form the possible expected responses list
    static const ResponseInfo_t WsSendData_expectedResponses[] = {{expected_WsSendOK, sizeof(expected_WsSendOK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &WsSendData_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(WsSendData_expectedResponses))/(sizeof(ResponseInfo_t));
    uint32_t printedLen = 0;
    // data is buffered via transparent mode until the number of bytes set with cipsend is sent to the esp32, then the esp32 will trigger transmission
    printedLen = snprintf((char*)tx_buffer, send_data_length+1, "%s", data);// +1 becaus eof snprintf including the 0x00 terminating character in the count

    // physical uart send of the command
    return SendCommand(printedLen);
}
/**
 * @brief parses the esp32 response after delivering raw data bytes as payload via an open socket
 *
 * @return
 */
bool ESP32C3::Parse_SendData()
{
    if(strnstr((char*)rx_buffer,(char*)expected_SendOK, sizeof(rx_buffer) ) != nullptr){
        return true;
    }
    return false;
}

///@brief setter fore urc readiness flag enables or disables uart reception of unsolicited messages from the module
void ESP32C3::SetUrcReadiness(bool setting)
{
    urc_readiness = setting;
    FlushRx();
}
///@brief flush the rx process restarting it
void ESP32C3::FlushRx()
{
    /* stop receiving via uart */
    huart->ReceiveIrqStop();
    /*void the rx buffer*/
    rx_index  = 0;
    memset((void*)rx_buffer, 0, sizeof(rx_buffer));
    /* reset expectations */
    expectedResponsesList = nullptr;
    if(urc_readiness)
    {
        /* trigger reception for 1 additional byte*/
        huart->ReceiveIrqStart((uint8_t*)&rx_buffer[rx_index], 1);
    }
}

/**
 * @brief sets up receive mode of the esp32 module, which means
 *   incoming socket data can be buffered internally or streamed directly
 * @param link_id socket id affected
 * @param bufferedMode true, data is stored internally, false data is streamed along +IPD spontaneous urc
          https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/TCP-IP_AT_Commands.html#tcpip-at
 * @return
 */
bool ESP32C3::Send_AT_CIPRECVTYPE(LinkId link_id, bool bufferedMode)
{
    // form the possible expected responses list
    static const ResponseInfo_t AT_CIPRECVTYPE_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CIPRECVTYPE_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CIPRECVTYPE_expectedResponses))/(sizeof(ResponseInfo_t));
    // considering ID = 5 closes all connections format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPRECVTYPE=%d,%d\r\n", link_id, (uint8_t)bufferedMode);
    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief triggers the transmission of buffered incoming data (received by esp32 via opened socket) to the mcu
 * this is used when @Send_AT_CIPRECVTYPE buffered mode is true
 * @param link_id socket that receives data
 * @param maxLen max bytes to stream
 * @return
 */
bool ESP32C3::Send_AT_CIPRECVDATA(LinkId link_id, uint32_t maxLen)
{
    // form the possible expected responses list
    static const ResponseInfo_t AT_CIPRECVDATA_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CIPRECVDATA_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CIPRECVDATA_expectedResponses))/(sizeof(ResponseInfo_t));
    // considering ID = 5 closes all connections format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPRECVDATA=%d,%lu\r\n", link_id, maxLen);
    // physical uart send of the command
    return SendCommand(printedLen);
}
/**
 * @brief buffered data getter after Send_AT_CIPRECVDATA to obtain received data
 * @param buffer output
 * @return false in case of error
 */
bool ESP32C3::Parse_AT_CIPRECVDATA(uint8_t* outputBuffer,uint32_t maxLen)
{
    bool retVal = false;
    /* look for the OK presence*/
    if(strnstr((char*)rx_buffer,(char*)expected_SendOK, sizeof(rx_buffer) ) != nullptr)
    {
        /*+CIPRECVDATA:<actual_len>,<data>
           OK*/
        uint8_t* msgStaPtr = (uint8_t*)strnstr((char*)rx_buffer, "+CIPRECVDATA:", sizeof(rx_buffer));
        if(msgStaPtr > rx_buffer)
        {
            /*advance pointer beyond CIPRECVDATA: */
            msgStaPtr+= 13;
            /* interpret data length field */
            uint32_t len = atoi((char*)msgStaPtr);
            /* buffer overflow guard */
            if(len > maxLen)
            {
                len = maxLen;
            }
            /* search for message payload field*/
            msgStaPtr = (uint8_t*)strnstr((char*)msgStaPtr, ",", sizeof(rx_buffer));
            if(msgStaPtr)
            {
                /* copy data */
                memcpy(outputBuffer, msgStaPtr+1, len );
                /* message correctly parsed and rebuffered */
                retVal =  true;
            }
        }
    }
    return retVal;
}

/**
 * @brief obtain raw rx buffer data frome sp32c module communication
 * @param output data storage
 * @param maxLen size of data storage
 * @return bytes copied
 */
uint32_t ESP32C3::GetRxBufferData(uint8_t *output, uint32_t maxLen)
{
    uint32_t len = strnlen((char*)rx_buffer, sizeof(rx_buffer));
    if(len> maxLen){
        len = maxLen;
    }
    memcpy(output, (void*)rx_buffer, len);
    return len;
}

/**
 * @brief Sends the command to retrieve the ascii-formatted time stampt
 *  this can be used only when a connection is established.
 *  It is required to validate certificates expiration dates
 *  @details
 *  https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/TCP-IP_AT_Commands.html#at-cipsntptime-query-the-sntp-time
 *  Example:
 *  AT+CIPSNTPTIME?
 *  +CIPSNTPTIME:<asctime style time>
 *
 *  AT+CIPSNTPTIME?
    +CIPSNTPTIME:Tue Oct 19 15:17:56 2021
    OK
    @note Parse the response using @Parse_AT_CIPSNTPTIME
 * @return uart command transmission
 */
bool ESP32C3::Send_AT_CIPSNTPTIME()
{
    // form the possible expected responses list
    static const ResponseInfo_t AT_CIPSNTPTIME_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CIPSNTPTIME_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CIPSNTPTIME_expectedResponses))/(sizeof(ResponseInfo_t));
    // considering ID = 5 closes all connections format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer), "AT+CIPSNTPTIME?\r\n");
    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief interprets the result of Send_AT_CIPSNTPTIME populating a standard time structure
 *  for example a string returned by the esp32 could be "Tue Oct 19 15:17:56 2021"
 * @param output time structure, must be a valid pointer
 * @return true if parsing successful, false otherwise
 */
bool ESP32C3::Parse_AT_CIPSNTPTIME(Time_t* timeStruct)
{
#if 0
    // https://www.ibm.com/docs/en/i/7.3?topic=functions-strptime-convert-string-datetime
    char* retPtr = strptime((char*)rx_buffer,"%A %B %d %T %Y" ,timeStruct );
#else
    // open source solution, also check https://stackoverflow.com/questions/26209832/is-there-a-reverse-function-for-local-time-in-c
    char* retPtr = unbound_strpntime((char*)rx_buffer,"%A %B %d %T %Y " ,timeStruct, sizeof(rx_buffer) );
#endif

    // check for ending OK token
    char* retOk = strnstr((char*)rx_buffer, (char*)expected_OK, sizeof(rx_buffer));

    TRACE_DRV_ESP32C("PrsTm=%d,%d,%d,%d:%d:%d", timeStruct->tm_wday, timeStruct->tm_mon, timeStruct->tm_mday,
            timeStruct->tm_hour, timeStruct->tm_min, timeStruct->tm_sec, timeStruct->tm_year);
    return (retOk!= nullptr && retPtr!= nullptr);

}

/**
 * @brief composes AT+WSCFG Sets websocket configuration AT command
 *     https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/websocket_at_commands.html#at-wscfg-set-the-websocket-configuration
 *     also from OCT 2023 see authentication https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/AT_Command_Set/TCP-IP_AT_Commands.html#id94
 * @param link link id for websockets is limited to 0-2
 * @param ping_interval
 * @param ping_timeout
 * @param internal buffer size def 1024
 * @param auth mode choose if none, client certification, server certification or mutual certification
 *     If <auth_mode> is configured to 2 or 3, in order to check the server certificate validity period,
 *     please make sure ESP32-C3 has obtained the current time before sending the AT+CIPSTART command.
 *     (You can send AT+CIPSNTPCFG command to configure SNTP and obtain the current time,
 *     and send AT+CIPSNPTIME? command to query the current time.)
 * @param pki number (only if auth_mode!=0) the index of certificate and private key.
 *          If there is only one certificate and private key, the value should be 0.
 *          to use custom certificate ( wss_ca wss_cert wss_key )
 *          https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/Compile_and_Develop/How_to_update_pki_config.html
 * @param certificate index (only if auth_mode!=0)  (stored in the esp file system) If there is only one CA, the value should be 0.
 * @note This command should be configured before AT+WSOPEN command. Otherwise, it will not take effect
 * @warning This AT command will return error if sent more than one time consecutively with the same parameters
 * @return
 */

bool ESP32C3::Send_AT_WSCFG(LinkId link,
                            uint32_t ping_interval,
                            uint32_t ping_timeout,
                            uint32_t buffer_size ,
                            AuthMode_t auth_mode ,
                            uint32_t pki_number ,
                            uint32_t ca_number )
{
    if(link > LinkId2)
    {
        TRACE_DRV_ESP32C("BadLinkId");
    }
    // form the possible expected responses list
    static const ResponseInfo_t AT_WSCFG_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_WSCFG_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_WSCFG_expectedResponses))/(sizeof(ResponseInfo_t));
#if SECURED_ESP32
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
            "AT+WSCFG=%d,%d,%d,%d,%d,%d,%d\r\n",
            (uint8_t)link,
            ping_interval,
            ping_timeout,
            buffer_size,
            (uint8_t)auth_mode,
            pki_number,
            ca_number);
#else
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
            "AT+WSCFG=%d,%lu,%lu,%lu\r\n",
            (uint8_t)link,
            ping_interval,
            ping_timeout,
            buffer_size);
#endif
    // physical uart send of the command
    return SendCommand(printedLen);
}
/**
 * @brief sends the AT+SYSFLASH Set command to perform an operation on a user flash partition on the esp32
 * command and positive response as follows
 * AT+SYSFLASH=<operation>,<partition>,<offset>,<length>
 * +SYSFLASH:<length>,<data>
   OK
   @note If the operator is write, wrap return > after the write command,
    then you can send the data that you want to write.
    The length should be parameter <length>.
    If the operator is write,
    please make sure that you have already erased this partition.

    // erase the "mfg_nvs" partition in its entirety.
    AT+SYSFLASH=0,"mfg_nvs",4096,8192

    // write a new "mfg_nvs" partition (size: 0x1C000) at offset 0 of the "mfg_nvs" partition.
    AT+SYSFLASH=1,"mfg_nvs",0,0x1C000
 * @details https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/AT_Command_Set/Basic_AT_Commands.html#at-sysflash-query-set-user-partitions-in-flash
 * @param operation type
 * @param partition name
 * @param offset from beginning of the partition memory space
 * @param amount of data to be read/written
 * @note to be followed by @@SendData to stream bytes to the module in case of Write operation (@WriteDataIntoUserPartition)
 * @return transmission successful or not
 */
bool ESP32C3::Send_AT_SYSFLASH(FlashOperation_t operation,
        uint8_t* partition,
        uint32_t offset,
        uint32_t length)
{

    // form the possible expected responses list
    static const ResponseInfo_t AT_SYSFLASH_expectedResponsesWrite[] = {{expected_SENDREADY, sizeof(expected_SENDREADY)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    static const ResponseInfo_t AT_SYSFLASH_expectedResponsesRead[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};

    if(operation == WriteDataIntoUserPartition)
    {
        expectedResponsesList = &AT_SYSFLASH_expectedResponsesWrite[0];
        // how many entries in the expected responses list
        expectedResponsesListLength = (sizeof(AT_SYSFLASH_expectedResponsesWrite))/(sizeof(ResponseInfo_t));

    }else
    {
        expectedResponsesList = &AT_SYSFLASH_expectedResponsesRead[0];
        // how many entries in the expected responses list
        expectedResponsesListLength = (sizeof(AT_SYSFLASH_expectedResponsesRead))/(sizeof(ResponseInfo_t));
    }

    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
            "AT+SYSFLASH=%d,\"%s\",%lu,%lu\r\n",
            (uint8_t)operation,
            partition,
            offset,
            length);

    if(operation == WriteDataIntoUserPartition)
    {
        // update stream length (prepares transmission of flash data)
        send_data_length = length;
    }
    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief Sends the websocket header setting command AT+WSHEAD
 *  https://docs.espressif.com/projects/esp-at/en/latest/esp32/AT_Command_Set/websocket_at_commands.html#at-wshead-set-query-websocket-request-headers
 * @param length of the header to be sent
 * @note can be parsed with @Parse_AT_CIPSEND
 * expected response
 * OK

   >
 * @return
 */
bool ESP32C3::Send_AT_WSHEAD(uint32_t length)
{
    // form the possible expected responses list
    static const ResponseInfo_t AT_WSHEAD_expectedResponses[] = {{expected_SENDREADY, sizeof(expected_SENDREADY)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_WSHEAD_expectedResponses[0];
    ws_header_length = length;
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_WSHEAD_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
            "AT+WSHEAD=%lu\r\n",
            length);
    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief deliver data bytes to the esp32 to be used as ws header
 *        this should be done after a successful AT+WSHEAD
 * @param data
 * @return
 */
bool ESP32C3::Send_WsHeader(const uint8_t* data)
{
    // form the possible expected responses list
    static const ResponseInfo_t WSHeadData_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &WSHeadData_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(WSHeadData_expectedResponses))/(sizeof(ResponseInfo_t));
    uint32_t printedLen = 0;
    // data is buffered via transparent mode until the number of bytes set with cipsend is sent to the esp32, then the esp32 will trigger transmission
    printedLen = snprintf((char*)tx_buffer, ws_header_length+1, "%s", data);

    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief Parses the AT+WSHEAD querty AT command AT+WSHEAD?
 *  +WSHEAD:<index>,<"req_header">

    OK
 * @param index output
 * @param header_ptr output
 * @return
 */
bool ESP32C3::Parse_AT_WSHEAD_GET(uint32_t* index, uint8_t* header_ptr)
{
    bool retVal = false;
    /* look for the OK presence*/
    if(strnstr((char*)rx_buffer,(char*)expected_OK, sizeof(rx_buffer)) != nullptr )
    {
        /*+WSHEAD:<index>,<"req_header">
           OK*/
        uint8_t* msgStaPtr = (uint8_t*)strnstr((char*)rx_buffer, "+WSHEAD:", sizeof(rx_buffer));
        if(msgStaPtr > rx_buffer)
        {
            /*advance pointer beyond +WSHEAD: */
            msgStaPtr+= 8;
            /* interpret data length field */
            *index = (uint32_t)atoi((char*)msgStaPtr);

            /* search for message header payload field*/
            msgStaPtr = (uint8_t*)strnstr((char*)msgStaPtr, "\"", sizeof(rx_buffer));
            uint8_t* headerEnd = (uint8_t*)strnstr((char*)msgStaPtr+1, "\"", sizeof(rx_buffer));
            if(msgStaPtr && headerEnd && (headerEnd>msgStaPtr))
            {
                /* copy data */
                memcpy(header_ptr, msgStaPtr+1, headerEnd-msgStaPtr-1 );
                /* message correctly parsed and re-buffered */
                retVal =  true;
            }
        }
    }
    return retVal;
}

/**
 * @brief opens a websocket using the AT+WSOPEN command, the method formats the string to be sent via uart
 *  expected behavior:
 *  AT+WSOPEN=<link_id>,<"uri">[,<"subprotocol">][,<timeout_ms>][,<"auth">]
    +WS_CONNECTED:<link_id>

    OK
 * @details at webpage https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/AT_Command_Set/websocket_at_commands.html#at-wsopen-query-open-a-websocket-connection
 *

 * @param link (value is 0-2)
 * @param uri string
 * @param optional - sub protocol string (for example "ocpp1.6")
 * @param optional - authorization (optional)
 * @return
 */
bool ESP32C3::Send_AT_WSOPEN(LinkId link, const uint8_t *uri, const uint8_t* sub_protocol = nullptr, const uint8_t* auth = nullptr)
{
    /* Sanitize uri string for esp32c3 input*/
    uint8_t escaped_uri[ESP32_URI_MAX_LEN] = {0};
    EscapeStringParameter(uri, escaped_uri, ESP32_URI_MAX_LEN);
    uint8_t escaped_sub_protocol[ESP32_URI_MAX_LEN] = {0};
    if(sub_protocol != nullptr)
    {
        EscapeStringParameter(sub_protocol, escaped_sub_protocol, ESP32_URI_MAX_LEN);
    }
    uint8_t escaped_auth[ESP32_URI_MAX_LEN] = {0};
    if(auth != nullptr)
    {
        EscapeStringParameter(auth, escaped_auth, ESP32_URI_MAX_LEN);
    }
    static const ResponseInfo_t AT_WSOPEN_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_WSOPEN_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_WSOPEN_expectedResponses))/(sizeof(ResponseInfo_t));

    uint32_t printedLen = 0;
    if(auth != nullptr)
    {
        // format the command string and retrieve how long is the command string
        printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
               "AT+WSOPEN=%d,\"%s\",\"%s\",%d,\"%s\"\r\n",
               (uint8_t)link,
               escaped_uri,
               escaped_sub_protocol,
               ESP32_DEFAULT_WS_TIMEOUT_MS,
               escaped_auth);
    }else{
        printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
               "AT+WSOPEN=%d,\"%s\",\"%s\",%d\r\n",
               (uint8_t)link,
               escaped_uri,
               escaped_sub_protocol,
               ESP32_DEFAULT_WS_TIMEOUT_MS
               );
    }




    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief used to parse the response to the AT+WSOPEN
 *  +WS_CONNECTED:<link_id>

    OK
   @param[out] link id to be returned, only valid if function return true
 * @return true if iopened and link parameter gets populated, false otherwise
 */
bool ESP32C3::Parse_AT_WSOPEN(LinkId* link)
{
    bool retVal = false;
    /* look for the OK presence*/
    if(strnstr((char*)rx_buffer,(char*)expected_OK, sizeof(rx_buffer)) != nullptr)
    {
        uint8_t* msgStaPtr = (uint8_t*)strnstr((char*)rx_buffer, "+WS_CONNECTED:", sizeof(rx_buffer));
        if(msgStaPtr > rx_buffer)
        {
            /*advance pointer beyond +WS_CONNECTED: */
            msgStaPtr+= 14;
            /* interpret data length field */
            *link = (LinkId)atoi((char*)msgStaPtr);
            if(*link < (LinkIdMax))
            {
                retVal = true;
            }
        }
    }
    return retVal;
}

/**
 * @brief prepares for transmission of data via opened websocket by sending the AT+WSSEND command
 * AT+WSSEND=<link_id>,<length>[,<opcode>][,<timeout_ms>]
 * OK

   >

   @details https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/AT_Command_Set/websocket_at_commands.html#at-wssend-send-data-to-a-websocket-connection
 * @param link of the socket (0-2)
 * @param length data length in bytes to be sent
 * @param opcode data type
 * @note can be parsed with @Parse_AT_CIPSEND(), to be followed by @SendData
 * @return
 */
bool ESP32C3::Send_AT_WSSEND(LinkId link, uint32_t length,
        WebSockedtOpCode_t opcode)
{
    // form the possible expected responses list
    static const ResponseInfo_t AT_WSSEND_expectedResponses[] = {{expected_SENDREADY, sizeof(expected_SENDREADY)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_WSSEND_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_WSSEND_expectedResponses))/(sizeof(ResponseInfo_t));
    if(length> sizeof(tx_buffer))
    {
        TRACE_DRV_ESP32C("ERROR WS TX OVERFLOW");
    }
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
            "AT+WSSEND=%d,%lu,%d\r\n",
            (uint8_t)link,
            length,
            (uint8_t)opcode);
    send_data_length = length;
    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief Closes a previously opened websocket with the AT+WSCLOSE command sent to the esp32c3 module
 * AT+WSCLOSE=<link_id>
 * OK
 * @details https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/AT_Command_Set/websocket_at_commands.html#at-wsclose-close-a-websocket-connection
 * @param link
 * @return
 */
bool ESP32C3::Send_AT_WSCLOSE(LinkId link)
{
    // form the possible expected responses list
    static const ResponseInfo_t AT_WSCLOSE_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_WSCLOSE_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_WSCLOSE_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
            "AT+WSCLOSE=%d\r\n",
            (uint8_t)link);
    // physical uart send of the command
    return SendCommand(printedLen);
}

/**
 * @brief prepares and sends AT+CIPSERVERMAXCONN command to set the max number of connection in server mode
 * https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/AT_Command_Set/TCP-IP_AT_Commands.html#at-cipservermaxconn-query-set-the-maximum-connections-allowed-by-a-server
 * @param num
 * @return
 */
bool DRV::ESP32C3::Send_AT_CIPSERVERMAXCONN(uint8_t num)
{
    // form the possible expected responses list
    static const ResponseInfo_t AT_CIPSERVERMAXCONN_expectedResponses[] = {{expected_OK, sizeof(expected_OK)-1}, {expected_ERROR, sizeof(expected_ERROR)-1}};
    expectedResponsesList = &AT_CIPSERVERMAXCONN_expectedResponses[0];
    // how many entries in the expected responses list
    expectedResponsesListLength = (sizeof(AT_CIPSERVERMAXCONN_expectedResponses))/(sizeof(ResponseInfo_t));
    // format the command string and retrieve how long is the command string
    uint32_t printedLen = snprintf((char*)tx_buffer, sizeof(tx_buffer),
            "AT+CIPSERVERMAXCONN=%d\r\n",
            (uint8_t)num);
    // physical uart send of the command
    return SendCommand(printedLen);
}

