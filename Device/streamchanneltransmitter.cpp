#include "streamchanneltransmitter.h"

StreamChannelTransmitter::StreamChannelTransmitter(int id)
{
    this->id = id;
    initRegisterMap();

    int sourcePort = rand()*10000+40000; //select random port between rangeport 40000-50000

    registers[sourcePortRegCode]->setValueNumb(sourcePort);

    setupStardardRegistersValue();
}

StreamChannelTransmitter::StreamChannelTransmitter(int id, int sourcePort)
{
    this->id = id;
    initRegisterMap();

    registers[sourcePortRegCode]
            ->setValueNumb(sourcePort);

    setupStardardRegistersValue();
}

StreamChannelTransmitter::~StreamChannelTransmitter()
{
    if(isChannelOpen())
        closeStreamChannel();

    foreach(auto reg, registers)
        delete reg.second;
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

void StreamChannelTransmitter::openStreamChannel(QHostAddress destAddress, quint16 destPort)
{
    closeStreamChannel(); //Check port different from 0 (R-087ca)

    BootstrapRegister* gvspSCPD = registers[sourcePortRegCode];

    streamChannelTransmitter = new UDPChannelTransmitter(destAddress, gvspSCPD->getValueNumb());
    streamChannelTransmitter->initSocket();

    BootstrapRegister* gvspSCP = registers[channelPortRegCode];

    gvspSCP->setValueNumb(
                (gvspSCP->getValueNumb() | (destPort & 0x00FF)) //set new destination port value
                );
    this->destPort = destPort;

    BootstrapRegister* gvspSCDA =
            registers[packetDestinationAddressRegCode];
    gvspSCDA->setValueNumb((int) destAddress.toIPv4Address());
    this->destAddress = destAddress;

    blockId=1;
}

void StreamChannelTransmitter::closeStreamChannel()
{
    if(isChannelOpen()) {
        destPort = 0;
        BootstrapRegister* gvspSCP = registers[channelPortRegCode];
        gvspSCP->setValueNumb(gvspSCP->getValueNumb() & 0xFF00); //Reset port


        destAddress.clear();
        BootstrapRegister* gvspSCDA = registers[packetDestinationAddressRegCode];
        gvspSCDA->setValueNumb(0);

        blockId=1;

        delete streamChannelTransmitter;
    }
}

bool StreamChannelTransmitter::isChannelOpen()
{
    BootstrapRegister* gvspSCP = registers[channelPortRegCode];

    if((gvspSCP->getValueNumb() & 0xFF) != 0)
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
    registers[packetSizeRegCode]->setValueNumb(576); //Stream packet size max 576 including headers
}

void StreamChannelTransmitter::writeIncomingData(PixelsMap *datapacket)
{
    if(!isChannelOpen())
        return;

    int packetSize = registers[packetSizeRegCode]->getValueNumb()-200; //200 should be header payload

    char* data = datapacket->getImagePixelData();

    quint32 packetId=1;

    StreamImageDataLeader* leader = new StreamImageDataLeader(destAddress, destPort,
           blockId, packetId, datapacket->getPixelFormat(),
           datapacket->getSizeX(), datapacket->getSizeY(),
           datapacket->getOffsetX(), datapacket->getOffsetY(),
           datapacket->getPaddingX(), datapacket->getPaddingY());

    streamChannelTransmitter->sendCommand(leader);
    delete leader;

    //TODO:da fare decentemente
    int pointer=0;
    while(pointer<datapacket->getDataLength()) {
        packetId++;
        char* actualPayloadData = new char[packetSize];
        for (int i = 0; i < packetSize; ++i)
            actualPayloadData[i]=data[pointer+i];
        StreamImageDataPayload* payload = new StreamImageDataPayload(destAddress, destPort,
                                                                     blockId, packetId,
                                                                     actualPayloadData, packetSize);
        streamChannelTransmitter->sendCommand(payload);
        delete payload;
        delete actualPayloadData;
        pointer += packetSize;
    }

    int residualPacket = datapacket->getDataLength()-pointer;
    if(residualPacket>0) {
        packetId++;
        char* actualPayloadData = new char[packetSize];
        for (int i = 0; i < residualPacket; ++i)
            actualPayloadData[i]=data[pointer+i];
        StreamImageDataPayload* payload = new StreamImageDataPayload(destAddress, destPort,
                                                                     blockId, packetId,
                                                                     actualPayloadData, packetSize);
        streamChannelTransmitter->sendCommand(payload);
        delete payload;
        delete actualPayloadData;
    }

    packetId++;
    StreamImageDataTrailer* trailer = new StreamImageDataTrailer(destAddress, destPort,
           blockId, packetId, datapacket->getSizeY());

    streamChannelTransmitter->sendCommand(trailer);
    delete trailer;
}
