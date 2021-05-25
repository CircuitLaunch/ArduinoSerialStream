#ifndef __SERIALSTREAM_HPP__
#define __SERIALSTREAM_HPP__

#include <StandardCplusplus.h>
#include <deque>

using namespace std;

namespace ass
{
  enum Types {
    STRING = 1,
    INT32,
    FLOAT32
  };

  class SerialStream
  {
  public:
    class Packet
    {
      public:
        Packet(uint8_t iType = FLOAT32, uint16_t iByteSize = 0, void *iData = nullptr, bool iManage = true)
        : type(iType), byteSize(iByteSize), data((uint8_t *) iData), capacity(0), manage(iManage) { }
        virtual ~Packet();

        uint16_t size();
        template <typename T>
        T *getData() { return (T *) data; }

        void resizeIfNeeded();

        uint8_t type;
        uint16_t byteSize;
        uint8_t *data;
        uint16_t capacity;
        bool manage;

        void convert();
    };

    class Message
    {
      public:
        Message();
        virtual ~Message();

        void setPacketCount(uint8_t iCount);

        uint8_t cmd;
        vector<Packet *> packets;

      protected:
        static vector<Packet *> packetPool;
    };

  public:
    SerialStream(int iFD = 0);
    virtual ~SerialStream();

    Message *getSendMessage();

    void queueSendMessage(Message *iMsg);
    void recycleRecvMessage(Message *iMsg);
    Message *popRecvMessage();

    void tick();

  protected:
    int doWrite();
    int doRead();

    Message *getRecvMessage();
    void recycleSendMessage(Message *iMsg);

  protected:
    int fd;

    enum State {
      IDLE = 0,
      CMD,
      PKTS,
      TYPE,
      SIZE,
      DATA
    };

    vector<Message *> recvMsgPool;
    deque<Message *> recvMsgQueue;
    Message *recvMsg;
    uint8_t recvPktCount;
    uint32_t recvPktIndex;
    Packet *recvPkt;
    uint8_t *readPtr;
    uint32_t readCount;
    uint32_t readState;

    vector<Message *> sendMsgPool;
    deque<Message *> sendMsgQueue;
    Message *sendMsg;
    uint8_t sendPktCount;
    uint32_t sendPktIndex;
    Packet *sendPkt;
    uint8_t *writePtr;
    uint32_t writeCount;
    uint32_t writeState;

    uint16_t netWrite16;
    uint16_t netRead16;
};

}

#endif
