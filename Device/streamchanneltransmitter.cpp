#include "streamchanneltransmitter.h"

StreamChannelTransmitter::StreamChannelTransmitter(int id)
{
    this->id = id;
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

    streamChannelTransmitter = new UDPChannelTransmitter(destAddress, gvspSCPD->getValue());
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

void StreamChannelTransmitter::writeIncomingData(PixelsMap *datapacket)
{
    if(!isChannelOpen())
        return;

    if(datapacket->getDataLength()<=0)
        return;

    //CR-489cd
    quint32 packetSize = (registers[packetSizeRegCode]->getValue() & 0x0000FFFF)
            -20 //bytes IP header
            -8 //bytes UDP header
            -20; //bytes for GVSP header

    char* data = datapacket->getImagePixelData();

    //Send data leader packet
    quint32 packetId=1;

    StreamImageDataLeader* leader = new StreamImageDataLeader(destAddress, destPort,
           blockId, packetId, datapacket->getPixelFormat(),
           datapacket->getSizeX(), datapacket->getSizeY(),
           datapacket->getOffsetX(), datapacket->getOffsetY(),
           datapacket->getPaddingX(), datapacket->getPaddingY());

    streamChannelTransmitter->sendCommand(leader);
    delete leader;

    //Compute how many packets need to be send
    quint32 packetsToSend = floor(datapacket->getDataLength() / packetSize);
    quint32 lastPacketsDimension = datapacket->getDataLength() % packetSize;

    //send packets
    for (quint32 i = 0; i < packetsToSend; ++i) {
        quint32 pointer = i*packetSize;
        char* actualPayloadData = new char[packetSize];
        for (quint32 i = 0; i < packetSize; ++i)
            actualPayloadData[i]=data[pointer+i];
        packetId++;
        StreamImageDataPayload* payload = new StreamImageDataPayload(destAddress, destPort,
                                                                     blockId, packetId,
                                                                     actualPayloadData, packetSize);
        streamChannelTransmitter->sendCommand(payload);
        delete payload;
        delete actualPayloadData;

        //CR-491cd delay
        /*
        if(registers[packetDelayRegCode]->getValue()>0) {
            //TODO: not implementet yet
        }
        */
    }

    //send last packets if need
    if(lastPacketsDimension>0) {
        packetId++;
        quint32 pointer = packetsToSend*packetSize;
        char* actualPayloadData = new char[lastPacketsDimension];
        for (quint32 i = 0; i < lastPacketsDimension; ++i)
            actualPayloadData[i]=data[pointer+i];
        StreamImageDataPayload* payload = new StreamImageDataPayload(destAddress, destPort,
                                                                     blockId, packetId,
                                                                     actualPayloadData, lastPacketsDimension);
        streamChannelTransmitter->sendCommand(payload);
        delete payload;
        delete actualPayloadData;
    }

    //send trailer packet
    packetId++;
    StreamImageDataTrailer* trailer = new StreamImageDataTrailer(destAddress, destPort,
           blockId, packetId, datapacket->getSizeY());

    streamChannelTransmitter->sendCommand(trailer);
    delete trailer;

    //Increment block id for the next data block
    blockId++;
}
