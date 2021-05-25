#include <StandardCplusplus.h>
#include <deque>

#include "SerialStream.hpp"

using namespace std;
using namespace trajectory_publisher;

SerialStream ss;

String tempStrBuf;

void processMsg(SerialStream::Message &iMsg)
{
  tempStrBuf = "Arduino received Message with " + String(iMsg.packets.size()) + " packets";

  for(auto pkt : iMsg.packets) {
    switch(pkt->type) {
      case STRING: {
        char *data = pkt->getData<char>();
        tempStrBuf += "\n STRING: ";
        tempStrBuf += data;
        } break;
      case INT32: {
        uint32_t *data = pkt->getData<uint32_t>();
        tempStrBuf += "\n INT32 array: [" +  String(data[0]) + ", " + String(data[1]) + ", " + String(data[2]) + ", " + String(data[3]) + " ... ]";
        } break;
      case FLOAT32: {
        float *data = pkt->getData<float>();
        tempStrBuf += "\n FLOAT32 array: [" +  String(data[0]) + ", " + String(data[1]) + ", " + String(data[2]) + ", " + String(data[3]) + " ... ]";
        } break;
    }
  }

  SerialStream::Message *newMsg = ss.getSendMessage();
  newMsg->setPacketCount(1);
  SerialStream::Packet *newPkt = newMsg->packets[0];
  newPkt->type = STRING;
  newPkt->byteSize = tempStrBuf.length() + 1;
  newPkt->resizeIfNeeded();
  memmove(newPkt->getData<char>(), tempStrBuf.c_str(), newPkt->byteSize);

  ss.queueSendMessage(newMsg);
}

uint32_t testIntData[10];
float testFloatData[10];

void setup() {
  for(int i = 0; i < 10; i++) {
    testIntData[i] = uint32_t(i);
  }
  for(int i = 0; i < 10; i++) {
    testFloatData[i] = float(i);
  }
  Serial.begin(115200);
  ss.recycleRecvMessage(new SerialStream::Message());
  while(!Serial);
  delay(5000);
}

unsigned long time = millis();
bool start = false;
void loop() {
  /*
  while(Serial.available()) {
    char ch = Serial.read();
    Serial.write(ch);
  }
  */

  SerialStream::Message *recvMsg;

  if((millis() - time) > 66) {
    SerialStream::Message *msg = ss.getSendMessage();
    msg->setPacketCount(3);

    SerialStream::Packet *pkt0 = msg->packets[0];
    pkt0->type = STRING;
    pkt0->byteSize = 5;
    pkt0->resizeIfNeeded();
    memmove(pkt0->data, "ping", 5);

    SerialStream::Packet *pkt1 = msg->packets[1];
    pkt1->type = INT32;
    pkt1->byteSize = 10 * 4;
    pkt1->resizeIfNeeded();
    memmove(pkt1->data, (char *) testIntData, pkt1->byteSize);

    SerialStream::Packet *pkt2 = msg->packets[2];
    pkt2->type = FLOAT32;
    pkt2->byteSize = 10 * 4;
    pkt2->resizeIfNeeded();
    memmove(pkt2->data, (char *) testFloatData, pkt2->byteSize);

    ss.queueSendMessage(msg);

    time = millis();
  } else {
    if(recvMsg = ss.popRecvMessage()) {
      processMsg(*recvMsg);
      ss.recycleRecvMessage(recvMsg);
    }
  }

  ss.tick();
}
