#ifndef BlinkyLoraGateway_h
#define BlinkyLoraGateway_h
#include "Arduino.h"
#include <CRC8.h>
#include <SPI.h>
#include <LoRa.h>
 
struct GatewayDataHeader
{
  uint8_t icrc;
  uint8_t istate;
  int16_t inodeAddr;
  int16_t igatewayAddr;
  int16_t iwatchdog;
  int16_t iforceArchive;  
  int16_t irssi;  
  int16_t isnr;  
}; 

class BlinkyLoraGatewayClass
{
  private: 
    GatewayDataHeader    _gatewayDataHeader;
    CRC8                _crc;
    boolean             _chattyCathy = false;
    size_t              _sizeofNodeData;
    size_t              _sizeofGatewayDataHeader;
    size_t              _sizeOfTransferData;
    uint8_t*            _pgatewayDataSend = nullptr;
    uint8_t*            _pnodeDataRecv = nullptr;
    volatile boolean    _gatewayHasDataToRead = false;
    volatile boolean    _nodeHasDataToRead = false;
    void                receiveData(int packetSize);

  public:
    BlinkyLoraGatewayClass(boolean chattyCathy);
    void            begin(size_t nodeDataSize, boolean chattyCathy, int16_t gateWayAddress, int loraChipSelectPin, int loraResetPin, int loraIRQPin, long loraFreq, int loraSpreadingFactor, long loraSignalBandwidth);
    boolean         retrieveNodeData(uint8_t* ploraData);
    boolean         publishGatewayData(uint8_t* pgatewayData);
    static void     onLoraTxDone();
    static void     onLoRaReceive(int packetSize);
    static void     rxMode();
    static void     txMode();
    static void     onCadDone(bool signalDetected);
    void            beginSendingLoraData();
    void            finishSendingLoraData();
        
};
extern BlinkyLoraGatewayClass BlinkyLoraGateway;
#endif
