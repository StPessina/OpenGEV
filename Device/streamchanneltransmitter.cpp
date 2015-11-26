#include "streamchanneltransmitter.h"

StreamChannelTransmitter::StreamChannelTransmitter(int id)
{
    this->id = id;
    initStreamDataDelayTimer();
    initRegisterMap();

    int sourcePort = rand()*10000+40000; //select random port between rangeport 40000-50000

    registers[sourcePortRegCode]->setValue(sourcePort);

    setupStardardRegistersValue();
}

StreamChannelTransmitter::StreamChannelTransmitter(int id, int sourcePort)
{
    this->id = id;
    initRegisterMap();

    registers[sourcePortRegCode]
            ->setValue(sourcePort);

    setupStardardRegistersValue();
}

StreamChannelTransmitter::~StreamChannelTransmitter()
{
    if(isChannelOpen())
        closeStreamChannel();

    foreach(auto reg, registers)
        delete reg.second;

    delete dataStreamDelay;
    delete dataStreamDelayLoop;
}

BootstrapRegister *StreamChannelTransmitter::getRegister(int regType)
{
    int regCode = DeviceRegisterConverter::getStreamChannelRegister(id, regType);

    return registers[regCode];
}

BootstrapRegister *StreamChannelTransmitter::getRegisterByAbsoluteRegCode(int regCode)
{
    return registers[regCode];
}

Status StreamChannelTransmitter::setRegister(int registerCode, int value, QHostAddress senderAddr, quint16 senderPort)
{
    BootstrapRegister* reg = getRegisterByAbsoluteRegCode(registerCode);
    if(reg==NULL) //CR-175cd
        return GEV_STATUS_INVALID_ADDRESS;

    int access = reg->getAccessType();

    if(access==RegisterAccess::RA_READ) //CR-175cd
        return GEV_STATUS_ACCESS_DENIED;

    int regType = DeviceRegisterConverter::getRegTypeFromStreamChannel(registerCode);

    switch (regType) {
        case REG_STREAM_CHANNEL_PORT:
            if(value==0)
                closeStreamChannel();
            else
                openStreamChannel(senderAddr,value);
        break;
        case REG_STREAM_CHANNEL_PACKET_DESTINATION_ADDRESS:
            //TODO: can or can't write this reg directly?
        break;
    default:
        reg->setValue(value);
        break;
    }

    return GEV_STATUS_SUCCESS;
}

void StreamChannelTransmitter::openStreamChannel(QHostAddress destAddress, quint16 destPort)
{
    closeStreamChannel(); //Check port different from 0 (R-087ca)

    BootstrapRegister* gvspSCPD = registers[sourcePortRegCode];

    streamChannelTransmitter = new UDPChannelTransmitter(QHostAddress::Any, gvspSCPD->getValue());
    streamChannelTransmitter->initSocket();

    BootstrapRegister* gvspSCP = registers[channelPortRegCode];

    gvspSCP->setValue(
                (gvspSCP->getValue() | (destPort & 0x0000FFFF)) //set new destination port value
                );
    this->destPort = destPort & 0x0000FFFF;

    BootstrapRegister* gvspSCDA =
            registers[packetDestinationAddressRegCode];
    gvspSCDA->setValue((int) destAddress.toIPv4Address());
    this->destAddress = destAddress;

    blockId=1;
}

void StreamChannelTransmitter::closeStreamChannel()
{
    if(isChannelOpen()) {
        destPort = 0;
        BootstrapRegister* gvspSCP = registers[channelPortRegCode];
        gvspSCP->setValue(gvspSCP->getValue() & 0xFFFF0000); //Reset port


        destAddress.clear();
        BootstrapRegister* gvspSCDA = registers[packetDestinationAddressRegCode];
        gvspSCDA->setValue(0);

        blockId=1;

        delete streamChannelTransmitter;
    }
}

bool StreamChannelTransmitter::isChannelOpen()
{
    BootstrapRegister* gvspSCP = registers[channelPortRegCode];

    if((gvspSCP->getValue() & 0xFFFF) != 0)
        return true;
    return false;
}

void StreamChannelTransmitter::initRegisterMap()
{
    channelPortRegCode=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_PORT);
    packetSizeRegCode=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_PACKET_SIZE);
    packetDelayRegCode=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_PACKET_DELAY);
    packetDestinationAddressRegCode=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_PACKET_DESTINATION_ADDRESS);
    sourcePortRegCode=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_SOURCE_PORT);
    channelCapabilityRegCode=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_CAPABILITY);
    channelConfigurationRegCode=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_CONFIGURATION);
    channelZoneRegCode=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_ZONE);
    channelZoneDirectionRegCode=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_ZONE_DIRECTION);

    registers[channelPortRegCode]
            = new  BootstrapRegister(channelPortRegCode, "Stream channel port (Stream channel #" +
                                     to_string(id) + ")",RA_READ_WRITE, 4);
    registers[packetSizeRegCode]
            = new  BootstrapRegister(packetSizeRegCode, "Stream channel packet size (Stream channel #" +
                                     to_string(id) + ")",RA_READ_WRITE, 4);
    registers[packetDelayRegCode]
            = new  BootstrapRegister(packetDelayRegCode, "Stream channel packet delay (Stream channel #" +
                                     to_string(id) + ")",RA_READ_WRITE, 4);
    registers[packetDestinationAddressRegCode]
            = new  BootstrapRegister(packetDestinationAddressRegCode, "Stream channel destination address (Stream channel #" +
                                     to_string(id) + ")",RA_READ_WRITE, 4);
    registers[sourcePortRegCode]
            = new  BootstrapRegister(sourcePortRegCode, "Stream channel source port (Stream channel #" +
                                     to_string(id) + ")",RA_READ, 4);
    registers[channelCapabilityRegCode]
            = new  BootstrapRegister(channelCapabilityRegCode, "Stream channel capability (Stream channel #" +
                                     to_string(id) + ")",RA_READ, 4);
    registers[channelConfigurationRegCode]
            = new  BootstrapRegister(channelConfigurationRegCode, "Stream channel configuration (Stream channel #" +
                                     to_string(id) + ")",RA_READ_WRITE, 4);
    registers[channelZoneRegCode]
            = new  BootstrapRegister(channelZoneRegCode, "Stream channel zone (Stream channel #" +
                                     to_string(id) + ")",RA_READ, 4);
    registers[channelZoneDirectionRegCode]
            = new  BootstrapRegister(channelZoneDirectionRegCode, "Stream channel zone direction (Stream channel #" +
                                     to_string(id) + ")",RA_READ, 4);
}

void StreamChannelTransmitter::setupStardardRegistersValue()
{
    registers[packetSizeRegCode]->setValue(576); //Stream packet size max 576 byte including headers

    // Delay between payload packet (CR-491cd)
    registers[packetDelayRegCode]->setValue(0);
}

void StreamChannelTransmitter::initStreamDataDelayTimer()
{
    this->dataStreamDelay = new QTimer(this);
    this->dataStreamDelay->setSingleShot(true);

    this->dataStreamDelay->setTimerType(Qt::PreciseTimer);

    this->dataStreamDelayLoop = new QEventLoop(this);

    connect(dataStreamDelay, SIGNAL(timeout()), dataStreamDelayLoop, SLOT(quit()));
}

int StreamChannelTransmitter::writeIncomingData(PixelMap<Pixel>::Ptr datapacket)
{
    if(!isChannelOpen())
        return 0;

    if(datapacket->dataLength<=0)
        return 0;

    //CR-489cd
    quint32 packetSize = (registers[packetSizeRegCode]->getValue() & 0x0000FFFF)
            -20 //bytes IP header
            -8 //bytes UDP header
            -20; //bytes for GVSP header

    QByteArray data (datapacket->getImagePixelData(), datapacket->dataLength);

    //Send data leader packet
    quint32 packetId=1;

    StreamImageDataLeader leader (destAddress, destPort,
           blockId, packetId, datapacket->pixelFormat,
           datapacket->sizex, datapacket->sizey,
           datapacket->offsetx, datapacket->offsety,
           datapacket->paddingx, datapacket->paddingy);

    streamChannelTransmitter->fastSendCommand(&leader);

    //Delay before start payloads
    dataStreamDelay->start(1);
    dataStreamDelayLoop->exec();

    //Compute how many packets need to be send
    quint32 packetsToSend = floor(datapacket->dataLength / packetSize);
    quint32 lastPacketsDimension = datapacket->dataLength % packetSize;

    //send packets
    QByteArray empty;
    StreamImageDataPayload payload (destAddress, destPort,
                                    blockId, 0,
                                    empty);
    for (quint32 i = 0; i < packetsToSend; ++i) {
        packetId++;
        payload.renew(packetId,
                      data.mid(((packetId-2)*packetSize),packetSize));

        streamChannelTransmitter->fastSendCommand(&payload);

        //CR-491cd delay
        quint32 delay = registers[packetDelayRegCode]->getValue();
        if(delay>0) {
            dataStreamDelay->start(delay);
            dataStreamDelayLoop->exec();
        }
    }

    //send last packets if need
    if(lastPacketsDimension>0) {
        packetId++;
        StreamImageDataPayload payload (destAddress, destPort,
                                        blockId, packetId,
                                        data.mid(((packetId-2)*packetSize)));
        streamChannelTransmitter->fastSendCommand(&payload);
    }

    //Delay before send data trailer after payloads
    dataStreamDelay->start(1);
    dataStreamDelayLoop->exec();

    //send trailer packet
    packetId++;
    StreamImageDataTrailer trailer(destAddress, destPort,
           blockId, packetId, datapacket->sizey);

    streamChannelTransmitter->fastSendCommand(&trailer);

    //Delete generated data
    data.clear();

    //Increment block id for the next data block
    blockId++;

    return packetsToSend+1;
}
