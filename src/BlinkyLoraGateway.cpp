#include "BlinkyLoraGateway.h"

BlinkyLoraGatewayClass::BlinkyLoraGatewayClass(boolean chattyCathy)
{
  _chattyCathy = chattyCathy;
  _sizeofGatewayDataHeader = sizeof(_gatewayDataHeader);
  

}
void BlinkyLoraGatewayClass::begin(size_t nodeDataSize, boolean chattyCathy, int16_t gateWayAddress, int loraChipSelectPin, int loraResetPin, int loraIRQPin, long loraFreq, int loraSpreadingFactor, long loraSignalBandwidth)
{
  _chattyCathy = chattyCathy;
  _sizeofNodeData = nodeDataSize;
  _sizeOfTransferData = _sizeofGatewayDataHeader + _sizeofNodeData;
  _pgatewayDataSend = new (std::nothrow) uint8_t [_sizeOfTransferData];
  _pnodeDataRecv = new (std::nothrow) uint8_t [_sizeOfTransferData];
  
  _gatewayDataHeader.istate = 0;
  _gatewayDataHeader.iforceArchive = 0;
  _gatewayDataHeader.inodeAddr = 0;
  _gatewayDataHeader.igatewayAddr = (int16_t) gateWayAddress;
  _gatewayDataHeader.iwatchdog = 0;
  _gatewayDataHeader.irssi = 0;  
  _gatewayDataHeader.isnr = 0;  
  _gatewayHasDataToRead = false;

  LoRa.setPins(loraChipSelectPin, loraResetPin, loraIRQPin);
  
  if (!LoRa.begin(loraFreq)) 
  {
    if (_chattyCathy) Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  LoRa.setSpreadingFactor(loraSpreadingFactor);
  LoRa.setSignalBandwidth(loraSignalBandwidth);
  if (_chattyCathy)
  {
    Serial.println("LoRa init succeeded.");
    Serial.println();
    Serial.println("LoRa Simple Gateway");
    Serial.println("Only receive messages from nodes");
    Serial.println("Tx: invertIQ enable");
    Serial.println("Rx: invertIQ disable");
    Serial.println();
  }
  LoRa.onReceive(BlinkyLoraGatewayClass::onLoRaReceive);
  LoRa.onTxDone(BlinkyLoraGatewayClass::onLoraTxDone);
  LoRa.onCadDone(BlinkyLoraGatewayClass::onCadDone);
  BlinkyLoraGatewayClass::rxMode();

  return;
}
void BlinkyLoraGatewayClass::rxMode()
{
  LoRa.disableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                        // set receive mode
}

void BlinkyLoraGatewayClass::txMode()
{
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();                // normal mode
}
boolean BlinkyLoraGatewayClass::publishGatewayData(uint8_t* pgatewayData) 
{
  if (_gatewayHasDataToRead) return false;
  if (_pgatewayDataSend == nullptr) return false;
  
  uint8_t* memPtr = _pgatewayDataSend;
  uint8_t* datPtr = pgatewayData;
  for (int ii = 0; ii < _sizeOfTransferData; ++ii)
  {
    *memPtr = *datPtr;
    ++memPtr;
    ++datPtr;
  }
      
  _crc.restart();
  memPtr = _pgatewayDataSend;
  ++memPtr;
  for (int ii = 1; ii < _sizeOfTransferData; ii++)
  {
    _crc.add(*memPtr);
    ++memPtr;
  }
  memPtr = _pgatewayDataSend;
  *memPtr = _crc.calc();
  _gatewayHasDataToRead = true;
  beginSendingLoraData();
  return true;
}
void BlinkyLoraGatewayClass::beginSendingLoraData() 
{
  if (!_gatewayHasDataToRead) return;
  BlinkyLoraGatewayClass::txMode();                      // set tx mode
  LoRa.beginPacket();                                 // start packet
  LoRa.channelActivityDetection();
}
void BlinkyLoraGatewayClass::finishSendingLoraData()
{
  LoRa.write(_pgatewayDataSend, _sizeOfTransferData);    // add payload
  LoRa.endPacket(true);                               // finish packet and send it
 _gatewayHasDataToRead = false;
}
void BlinkyLoraGatewayClass::receiveData(int packetSize)
{
  if (_nodeHasDataToRead) return;;
  uint8_t numBytes = 0;
  uint8_t garbageByte = 0;
  
  if (_chattyCathy) Serial.print("Received LoRa data at: ");
  if (_chattyCathy) Serial.println(millis());
  numBytes = LoRa.available();
  if (numBytes != _sizeOfTransferData)
  {
    for (int ii = 0; ii < numBytes; ++ii) garbageByte = (uint8_t) LoRa.read();
    if (_chattyCathy)
    {
      Serial.print("LoRa bytes do not match. Bytes Received: ");
      Serial.print(numBytes);
      Serial.print(", Bytes expected: ");
      Serial.println(_sizeOfTransferData);
    }
    return;
  }
  LoRa.readBytes(_pnodeDataRecv, _sizeOfTransferData);
  
  _crc.restart();
  uint8_t* memPtr = _pnodeDataRecv;
  ++memPtr;
  for (int ii = 1; ii < _sizeOfTransferData; ii++)
  {
    _crc.add(*memPtr);
    ++memPtr;
  }
  uint8_t crcCalc = _crc.calc();
  if (crcCalc != *_pnodeDataRecv) 
  {
    if (_chattyCathy)
    {
      Serial.print("LoRa CRC does not match. CRC Received: ");
      Serial.print(*_pnodeDataRecv);
      Serial.print(", CRC expected: ");
      Serial.println(crcCalc);
    }
    return;
  }

  GatewayDataHeader* _pgatewayDataHeader = (GatewayDataHeader*)  _pnodeDataRecv;
  if (_pgatewayDataHeader->igatewayAddr != _gatewayDataHeader.igatewayAddr) 
  {
    if (_chattyCathy)
    {
      Serial.println("LoRa Gateway address do not match. Addr Received: ");
      Serial.print(_pgatewayDataHeader->igatewayAddr);
      Serial.print(", Addr expected: ");
      Serial.println(_gatewayDataHeader.igatewayAddr);
    }
    return;
  }
  _pgatewayDataHeader->irssi = (int16_t)   LoRa.packetRssi();
  _pgatewayDataHeader->isnr  = (int16_t)  (LoRa.packetSnr() * 100);
  
  if (_chattyCathy)
  {
    Serial.print("Node Receive: ");
    Serial.println(numBytes);
    Serial.print("packetRssi     : ");
    Serial.println(_pgatewayDataHeader->irssi);
    Serial.print("packetSnr      : ");
    float fsnr = ((float)_pgatewayDataHeader->isnr) / 100;
    Serial.println(fsnr);
    Serial.print("icrc           : ");
    Serial.println(_pgatewayDataHeader->icrc);
  }
  _nodeHasDataToRead = true;

}
boolean BlinkyLoraGatewayClass::retrieveNodeData(uint8_t* pgatewayData)
{
  if (!_nodeHasDataToRead) return false;
  uint8_t* memPtr = _pnodeDataRecv;
  for (int ii = 0; ii < _sizeOfTransferData; ii++)
  {
    *pgatewayData = *memPtr;
    ++memPtr;
    ++pgatewayData;
  }
  _nodeHasDataToRead = false;
  return true;
}
void BlinkyLoraGatewayClass::onLoRaReceive(int packetSize) 
{
  BlinkyLoraGateway.receiveData(packetSize);
  return;
}
void BlinkyLoraGatewayClass::onLoraTxDone() 
{
  if (BlinkyLoraGateway._chattyCathy) Serial.println("TxDone");
  BlinkyLoraGatewayClass::rxMode();
}
void BlinkyLoraGatewayClass::onCadDone(bool signalDetected) 
{
  if (signalDetected)
  {
    delay(10);
    BlinkyLoraGateway.beginSendingLoraData();
    return;
  }
  BlinkyLoraGateway.finishSendingLoraData();
}



BlinkyLoraGatewayClass BlinkyLoraGateway(false);
