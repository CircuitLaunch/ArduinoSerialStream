#include <Arduino.h>
#include "SerialStream.hpp"

using namespace trajectory_publisher;

#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

SerialStream::Packet::~Packet()
{
  if(manage) if(data) delete [] data;
}

uint16_t SerialStream::Packet::size()
{
  switch(type) {
    case STRING:
      return byteSize;
    case INT32:
    case FLOAT32:
      return byteSize >> 2;
  }
  return 0;
}

void SerialStream::Packet::resizeIfNeeded()
{
  if(byteSize > capacity) {
    if(manage && data) delete [] data;
    manage = true;
    data = new uint8_t[byteSize];
    capacity = byteSize;
  }
}

void SerialStream::Packet::convert()
{
  if(type == STRING) return;
  int i = byteSize >> 2;
  uint32_t *ptr = (uint32_t *) data;
  while(i--) ptr[i] = htonl(ptr[i]);
}

vector<SerialStream::Packet *> SerialStream::Message::packetPool;

SerialStream::Message::Message()
: packets() { }

SerialStream::Message::~Message()
{
  while(packets.size()) {
    SerialStream::Packet *pkt = packets.back();
    packets.pop_back();
    SerialStream::Message::packetPool.push_back(pkt);
  }
}

void SerialStream::Message::setPacketCount(uint8_t iCount)
{
  while(packets.size() < iCount) {
    SerialStream::Packet *pkt;
    if(packetPool.size()) {
      pkt = packetPool.back();
      packetPool.pop_back();
    } else {
      pkt = new SerialStream::Packet();
    }
    packets.push_back(pkt);
  }
  while(packets.size() > iCount) {
    SerialStream::Packet *pkt = packets.back();
    packets.pop_back();
    packetPool.push_back(pkt);
  }
}

SerialStream::SerialStream()
: recvMsgQueue(), recvMsgPool(), sendMsgQueue(), sendMsgPool(), recvPktCount(0), sendPktCount(0),
  recvPkt(nullptr), readPtr(nullptr), readCount(0), readState(IDLE),
  sendPkt(nullptr), writePtr(nullptr), writeCount(0), writeState(IDLE)
{ }

SerialStream::~SerialStream()
{
  while(recvMsgQueue.size()) {
    Message *m = recvMsgQueue.front();
    recvMsgQueue.pop_front();
    delete m;
  }
  while(sendMsgQueue.size()) {
    Message *m = sendMsgQueue.front();
    sendMsgQueue.pop_front();
    delete m;
  }
  while(recvMsgPool.size()) {
    Message *m = recvMsgPool.front();
    recvMsgPool.pop_back();
    delete m;
  }
  while(sendMsgPool.size()) {
    Message *m = sendMsgPool.front();
    sendMsgPool.pop_back();
    delete m;
  }
}

void SerialStream::queueSendMessage(SerialStream::Message *iMsg)
{
  sendMsgQueue.push_back(iMsg);
}

void SerialStream::recycleRecvMessage(SerialStream::Message *iMsg)
{
  recvMsgPool.push_back(iMsg);
}

SerialStream::Message *SerialStream::getSendMessage()
{
  if(sendMsgPool.size()) {
    SerialStream::Message *msg = sendMsgPool.back();
    sendMsgPool.pop_back();
    return msg;
  }
  return new SerialStream::Message();
}

SerialStream::Message *SerialStream::popRecvMessage()
{
  if(recvMsgQueue.size()) {
    SerialStream::Message *front = recvMsgQueue.front();
    recvMsgQueue.pop_front();
    return front;
  }
  return nullptr;
}

void SerialStream::tick()
{
  if(Serial.availableForWrite()) {
    switch(writeState) {
      case IDLE:
        if(sendMsgQueue.size()) {
          sendMsg = sendMsgQueue.front();
          sendMsgQueue.pop_front();
          writeState = CMD;
          writePtr = &sendMsg->cmd;
          writeCount = 1;
        }
        break;
      case CMD:
        if(!doWrite()) {
          sendPktCount = uint8_t(sendMsg->packets.size());
          writeState = PKTS;
          writePtr = &sendPktCount;
          writeCount = 1;
        } break;
      case PKTS:
        if(!doWrite()) {
          sendPktIndex = 0;
          sendPkt = sendMsg->packets[sendPktIndex];
          writeState = TYPE;
          writePtr = &sendPkt->type;
          writeCount = 1;
        } break;
      case TYPE:
        if(!doWrite()) {
          writeState = SIZE;
          netWrite16 = htons(sendPkt->byteSize);
          writePtr = (uint8_t *) &netWrite16;
          writeCount = sizeof(sendPkt->byteSize);
        }
        break;
      case SIZE:
        if(!doWrite()) {
          writeState = DATA;
          writePtr = (uint8_t *) sendPkt->data;
          writeCount = sendPkt->byteSize;
          sendPkt->convert();
        }
        break;
      case DATA:
        if(!doWrite()) {
          sendPktIndex++;
          if(sendPktIndex == sendPktCount) {
            recycleSendMessage(sendMsg);
            sendPktIndex = 0;
            sendPktCount = 0;
            writeState = IDLE;
            writePtr = nullptr;
            writeCount = 0;
            sendPkt = nullptr;
            sendMsg = nullptr;
          } else {
            sendPkt = sendMsg->packets[sendPktIndex];
            writeState = TYPE;
            writePtr = &sendPkt->type;
            writeCount = 1;
          }
        }
        break;
    }
  }

  if(Serial.available()) {
    switch(readState) {
      case IDLE:
        if(recvMsgPool.size()) {
          recvMsg = getRecvMessage();
          readState = CMD;
          readPtr = &recvMsg->cmd;
          readCount = 1;
        }
        break;
      case CMD:
        if(!doRead()) {
          readState = PKTS;
          readPtr = &recvPktCount;
          readCount = 1;
        }
        break;
      case PKTS:
        if(!doRead()) {
          recvMsg->setPacketCount(recvPktCount);
          recvPktIndex = 0;
          recvPkt = recvMsg->packets[recvPktIndex];
          readState = TYPE;
          readPtr = &recvPkt->type;
          readCount = 1;
        } break;
      case TYPE:
        if(!doRead()) {
          readState = SIZE;
          readPtr = (uint8_t *) &netRead16;
          readCount = sizeof(recvPkt->byteSize);
        }
        break;
      case SIZE:
        if(!doRead()) {
          recvPkt->byteSize = ntohs(netRead16);
          readState = DATA;
          recvPkt->resizeIfNeeded();
          readPtr = (uint8_t *) recvPkt->data;
          readCount = recvPkt->byteSize;
        }
        break;
      case DATA:
        if(!doRead()) {
          recvPkt->convert();
          recvPktIndex++;
          if(recvPktIndex == recvPktCount) {
            recvMsgQueue.push_back(recvMsg);
            recvPktIndex = 0;
            recvPktCount = 0;
            readState = IDLE;
            readPtr = nullptr;
            readCount = 0;
            recvPkt = nullptr;
            recvMsg = nullptr;
          } else {
            recvPkt = recvMsg->packets[recvPktIndex];
            readState = TYPE;
            readPtr = &recvPkt->type;
            readCount = 1;
          }
        }
        break;
    }
  }
}

int SerialStream::doWrite()
{
  if(!writeCount) return 0;
  int bytesWritten = Serial.write(writePtr, writeCount);
  if(bytesWritten < 0) return writeCount;
  writeCount -= bytesWritten;
  writePtr += bytesWritten;
  return writeCount;
}

int SerialStream::doRead()
{
  if(!readCount) return 0;
  int bytesRead = Serial.readBytes(readPtr, readCount);
  if(bytesRead < 0) return readCount;
  readCount -= bytesRead;
  readPtr += bytesRead;
  return readCount;
}

SerialStream::Message *SerialStream::getRecvMessage()
{
  if(recvMsgPool.size()) {
    SerialStream::Message *msg = recvMsgPool.back();
    recvMsgPool.pop_back();
    return msg;
  }
  return new SerialStream::Message();
}

void SerialStream::recycleSendMessage(SerialStream::Message *iMsg)
{
  sendMsgPool.push_back(iMsg);
}
