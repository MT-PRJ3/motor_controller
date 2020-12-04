/* Generated: Wed Jun 22 2016 14:15:21 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "util/dataadaptersupport.h"
#include "class/dataadapter.gen.h"
#include "class/dataadapter.gen.c"

// Access the PhidgetIRSupport struct via the channel private pointer
#define DATAADAPTER_SUPPORT(ch) ((PhidgetDataAdapterSupportHandle)(((PhidgetChannelHandle)(ch))->private))

static void CCONV
PhidgetDataAdapter_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetDataAdapter_free(PhidgetChannelHandle *ch) {
	if (ch && *ch)
		PhidgetDataAdapterSupport_free((PhidgetDataAdapterSupportHandle *)&(*ch)->private);
	_free(ch);
}

API_PRETURN
PhidgetDataAdapter_create(PhidgetDataAdapterHandle *phidp) {

	PhidgetReturnCode res;

	res = _create(phidp);
	if (res == EPHIDGET_OK)
		res = PhidgetDataAdapterSupport_create((PhidgetDataAdapterSupportHandle *)&(*phidp)->phid.private);

	return (res);
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	//return (_setStatus(phid, bp));
	PhidgetDataAdapterHandle ch;
	const char* tmpString;
	int version;

	ch = (PhidgetDataAdapterHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 3) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 1 - functionality may be limited.", phid, version);
	}
	if (version >= 3) {
		tmpString = getBridgePacketStringByName(bp, "I2CFormat");
		memcpy(&ch->I2CFormat, tmpString, strlen(tmpString)+1); //Copy the null terminator
	}
	if (version >= 3)
		ch->baudRate = getBridgePacketUInt32ByName(bp, "baudRate");
	if (version >= 3)
		ch->minBaudRate = getBridgePacketUInt32ByName(bp, "minBaudRate");
	if (version >= 3)
		ch->maxBaudRate = getBridgePacketUInt32ByName(bp, "maxBaudRate");
	if (version >= 3)
		ch->dataBits = getBridgePacketUInt32ByName(bp, "dataBits");
	if (version >= 3)
		ch->minDataBits = getBridgePacketUInt32ByName(bp, "minDataBits");
	if (version >= 3)
		ch->maxDataBits = getBridgePacketUInt32ByName(bp, "maxDataBits");
	if (version >= 3)
		ch->deviceAddress = getBridgePacketUInt32ByName(bp, "deviceAddress");
	if (version >= 3)
		ch->handshakeMode = getBridgePacketInt32ByName(bp, "handshakeMode");
	if (version >= 3)
		ch->endianness = getBridgePacketInt32ByName(bp, "endianness");
	if (version >= 3)
		ch->IOVoltage = getBridgePacketInt32ByName(bp, "IOVoltage");
	if (version >= 3)
		ch->maxSendPacketLength = getBridgePacketUInt32ByName(bp, "maxSendPacketLength");
	if (version >= 3)
		ch->maxReceivePacketLength = getBridgePacketUInt32ByName(bp, "maxReceivePacketLength");
	if (version >= 3)
		ch->maxSendWaitPacketLength = getBridgePacketUInt32ByName(bp, "maxSendWaitPacketLength");
	if (version >= 3)
		ch->parity = getBridgePacketInt32ByName(bp, "parity");
	if (version >= 3)
		ch->protocol = getBridgePacketInt32ByName(bp, "protocol");
	if (version >= 3)
		ch->SPIMode = getBridgePacketInt32ByName(bp, "SPIMode");
	if (version >= 3)
		ch->stopBits = getBridgePacketInt32ByName(bp, "stopBits");
	if (version >= 3)
		ch->timeout = getBridgePacketUInt32ByName(bp, "timeout");
	if (version >= 3)
		ch->minTimeout = getBridgePacketUInt32ByName(bp, "minTimeout");
	if (version >= 3)
		ch->maxTimeout = getBridgePacketUInt32ByName(bp, "maxTimeout");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	//return (_getStatus(phid, bp));
	PhidgetDataAdapterHandle ch;

	ch = (PhidgetDataAdapterHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
		",I2CFormat=%s"
		",baudRate=%u"
		",minBaudRate=%u"
		",maxBaudRate=%u"
		",dataBits=%u"
		",minDataBits=%u"
		",maxDataBits=%u"
		",deviceAddress=%u"
		",handshakeMode=%d"
		",endianness=%d"
		",IOVoltage=%d"
		",maxSendPacketLength=%u"
		",maxReceivePacketLength=%u"
		",maxSendWaitPacketLength=%u"
		",parity=%d"
		",protocol=%d"
		",SPIMode=%d"
		",stopBits=%d"
		",timeout=%u"
		",minTimeout=%u"
		",maxTimeout=%u"
		, 3 /* class version */
		, ch->I2CFormat
		, ch->baudRate
		, ch->minBaudRate
		, ch->maxBaudRate
		, ch->dataBits
		, ch->minDataBits
		, ch->maxDataBits
		, ch->deviceAddress
		, ch->handshakeMode
		, ch->endianness
		, ch->IOVoltage
		, ch->maxSendPacketLength
		, ch->maxReceivePacketLength
		, ch->maxSendWaitPacketLength
		, ch->parity
		, ch->protocol
		, ch->SPIMode
		, ch->stopBits
		, ch->timeout
		, ch->minTimeout
		, ch->maxTimeout
		));


}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetDataAdapterSupport_init(DATAADAPTER_SUPPORT(phid));
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetDataAdapter_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDataAdapter_Protocol protocol;
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode res;
	uint32_t dataLen;
	int err;

	ch = (PhidgetDataAdapterHandle)phid;

	switch (bp->vpkt) {
	case BP_DATAIN:
		dataLen = getBridgePacketArrayCnt(bp, 0);

	//	PhidgetRunLock(ch);
		if (ch->lastDataLen == PUNK_SIZE) {
			ch->lastDataLen = 0;
		}

		memcpy(ch->eventData, getBridgePacketUInt8Array(bp, 0), dataLen);
		ch->eventDataLen = dataLen;

		if (dataLen > 0) {
			if ((ch->lastDataIndex + dataLen) < 8192) {
				memcpy(&ch->lastData[ch->lastDataIndex], getBridgePacketUInt8Array(bp, 0), dataLen);
				ch->lastDataIndex += dataLen;
			}
			else {
				int overhang = (ch->lastDataIndex + dataLen) % 8192;
				memcpy(&ch->lastData[ch->lastDataIndex], getBridgePacketUInt8Array(bp, 0), dataLen - overhang);
				memcpy(ch->lastData, &(getBridgePacketUInt8Array(bp, 0)[(dataLen - overhang)]), overhang);
				ch->lastDataIndex = overhang;
			}

			ch->lastDataLen += dataLen;
			if (ch->lastDataLen > 8192)
				ch->lastDataLen = 8192;
		}

		ch->newDataAvailable = 1;
		err = getBridgePacketUInt32(bp, 1);
		err |= getBridgePacketUInt32(bp, 2);

		ch->eventDataError = getBridgePacketUInt32(bp, 1);

		if (err) {
			ch->lastDataError = 1;
		}
//		PhidgetRunUnlock(ch);
		if(dataLen != 0)
			FIRECH(ch, Packet, ch->eventData, dataLen, err);

	//	PhidgetRunLock(ch);
		PhidgetLock(ch);
		ch->responseID = getBridgePacketUInt16(bp, 3);
		PhidgetBroadcast(ch);
		PhidgetUnlock(ch);
	//	PhidgetRunUnlock(ch);

		res = EPHIDGET_OK;
		break;
	case BP_SETPROTOCOL:
		protocol = (PhidgetDataAdapter_Protocol)getBridgePacketInt32(bp, 0);
		if (!supportedProtocol(phid, protocol))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Specified Protocol is unsupported by this device."));
		switch (protocol) {
		case PROTOCOL_I2C:
			ch->maxSendPacketLength = 512;
			ch->maxReceivePacketLength = 512;
			ch->maxSendWaitPacketLength = 512;
			ch->maxBaudRate = 400000;
			ch->minBaudRate = 10000;
			ch->minDataBits = 8;
			ch->maxDataBits = 8;
			ch->dataBits = 8;
			break;
		case PROTOCOL_SPI:
			ch->maxSendPacketLength = 512;
			ch->maxReceivePacketLength = 512;
			ch->maxSendWaitPacketLength = 512;
			ch->maxBaudRate = 1500000;
			ch->minBaudRate = 187500;
			ch->minDataBits = 4;
			ch->maxDataBits = 8;
			ch->dataBits = 8;
			break;
		case PROTOCOL_RS422:
		case PROTOCOL_RS485:
		case PROTOCOL_UART:
			ch->maxSendPacketLength = 10000000;
			ch->maxReceivePacketLength = 8192;
			ch->maxSendWaitPacketLength = 1024;
			ch->maxBaudRate = 2500000;
			ch->minBaudRate = 800;
			ch->minDataBits = 7;
			ch->maxDataBits = 8;
			ch->dataBits = 8;
			break;
		case PROTOCOL_DMX512:
			ch->maxSendPacketLength = 513;
			ch->maxReceivePacketLength = 0;
			ch->maxSendWaitPacketLength = 513;
			ch->maxBaudRate = 250000;
			ch->baudRate = 250000;
			ch->minBaudRate = 250000;
			ch->minDataBits = 8;
			ch->maxDataBits = 8;
			ch->dataBits = 8;
			ch->stopBits = STOP_BITS_TWO;
			break;
		case PROTOCOL_MODBUS_RTU:
			ch->maxSendPacketLength = 256;
			ch->maxReceivePacketLength = 256;
			ch->maxSendWaitPacketLength = 256;
			ch->maxBaudRate = 2500000;
			ch->minBaudRate = 800;
			ch->minDataBits = 8;
			ch->maxDataBits = 8;
			ch->dataBits = 8;
			ch->stopBits = STOP_BITS_ONE;
			break;
		}
		res = _bridgeInput(phid, bp);
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	if (bp->vpkt == BP_SETBAUDRATE && res == EPHIDGET_OK) {
		uint32_t baudRate = ch->baudRate;
		switch (ch->protocol) {
		case PROTOCOL_I2C:
			if (baudRate >= 400000)
				ch->baudRate = 400000;
			else if (baudRate >= 100000)
				ch->baudRate = 100000;
			else
				ch->baudRate = 10000;
			break;
		case PROTOCOL_SPI:
			if (baudRate >= 1500000)
				ch->baudRate = 1500000;
			else if (baudRate >= 750000)
				ch->baudRate = 750000;
			else if (baudRate >= 375000)
				ch->baudRate = 375000;
			else
				ch->baudRate = 187500;
			break;
		}
	}
	return (res);
}

static void
PhidgetDataAdapter_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetDataAdapter_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetDataAdapter_sendPacket(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length) {
	PhidgetReturnCode res = EPHIDGET_OK;
	uint32_t  maxBridgeLength;
	uint32_t i;

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	if (ch->protocol == PUNK_ENUM)
		return (PHID_RETURN_ERRSTR(EPHIDGET_NOTCONFIGURED, "Protocol needs to be set before packets can be sent."));

	PhidgetRunLock(ch);
	if (length > ch->maxSendPacketLength) {
		PhidgetRunUnlock(ch);
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Packet length too long."));
	}
	//Break the packet into smaller chunks if the baud rate is low enough to avoid timing out the bridge packet
	if (ch->baudRate > 6400)
		maxBridgeLength = 8192;
	else if (ch->baudRate > 3200)
		maxBridgeLength = 4096;
	else if (ch->baudRate > 1600)
		maxBridgeLength = 2048;
	else
		maxBridgeLength = 1024;

	for (i = 0; i < (uint32_t)length; i += maxBridgeLength) {
		uint32_t tmpLength = ((maxBridgeLength <= (length - i)) ? maxBridgeLength : (length % maxBridgeLength));
		res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DATAOUT, NULL, NULL, "%*R", tmpLength, &data[i]); //break long packets into bridge-packet sized chunks
		if (res != EPHIDGET_OK) {
			PhidgetRunUnlock(ch);
			return res;
		}
	}
	PhidgetRunUnlock(ch);
	return res;
}

API_VRETURN
PhidgetDataAdapter_sendPacket_async(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length,
  Phidget_AsyncCallback fptr, void *ctx) {
	PhidgetReturnCode res = EPHIDGET_OK;
	uint32_t maxBridgeLength;
	uint32_t i;

	if (ch == NULL) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (ch->phid.class != PHIDCHCLASS_DATAADAPTER) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
		return;
	}
	if (!ISATTACHED(ch)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
		return;
	}
	if (ch->protocol == PUNK_ENUM) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTCONFIGURED);
		return;
	}
	if (length > ch->maxSendPacketLength) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}

	//Break the packet into smaller chunks if the baud rate is low enough to avoid timing out the bridge packet
	if (ch->baudRate > 6400)
		maxBridgeLength = 8192;
	else if (ch->baudRate > 3200)
		maxBridgeLength = 4096;
	else if (ch->baudRate > 1600)
		maxBridgeLength = 2048;
	else
		maxBridgeLength = 1024;


	for (i = 0; i < (uint32_t)length; i += maxBridgeLength) {
		uint32_t tmpLength = ((maxBridgeLength <= (length - i)) ? maxBridgeLength : (length % maxBridgeLength));
		res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DATAOUT, fptr, ctx, "%*R", tmpLength, &data[i]); //break long packets into bridge-packet sized chunks
		if (res != EPHIDGET_OK)
			break;
	}

	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}

API_PRETURN
PhidgetDataAdapter_getLastData(PhidgetDataAdapterHandle ch, uint8_t *data, size_t *length, int *error) {
	TESTPTR_PR(ch);
	TESTPTR_PR(data);
	TESTPTR_PR(length);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);
	int err = 0;

	PhidgetRunLock(ch);

	if (ch->lastDataLen == PUNK_SIZE) {
		PhidgetRunUnlock(ch);
		return (EPHIDGET_UNKNOWNVAL);
	}

	size_t dataLen = ch->lastDataLen;
	if (*length < ch->lastDataLen) {
		dataLen = *length;
		err = 1;
	}

	size_t lastDataStartIndex = ch->lastDataIndex - dataLen;
	lastDataStartIndex %= 8192;

	if ((lastDataStartIndex + dataLen) < 8192) {
		memcpy(data, &ch->lastData[lastDataStartIndex], dataLen);
	} else {
		int overhang = (lastDataStartIndex + dataLen) % 8192;
		memcpy(data, &ch->lastData[lastDataStartIndex], dataLen - overhang);
		memcpy(&data[overhang], ch->lastData, overhang);
	}

	*length = dataLen;
	*error = (err || ch->lastDataError);

	ch->newDataAvailable = 0;
	ch->lastDataLen = 0;
	ch->lastDataError = 0;
	PhidgetRunUnlock(ch);
	return (EPHIDGET_OK);
}


API_PRETURN
PhidgetDataAdapter_sendPacketWaitResponse(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length, uint32_t milliseconds, uint8_t *recvData, size_t *recvDataLen, int* error) {
	PhidgetReturnCode res;
	TESTPTR_PR(ch);
	TESTPTR_PR(data);
	TESTPTR_PR(recvData);
	TESTPTR_PR(recvDataLen);
	TESTPTR_PR(error);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);
	uint8_t response[2];
	size_t responseLen = 2;
	uint16_t packetID;
	mostime_t duration;
	mostime_t start;

	if (ch->protocol == PUNK_ENUM)
		return (PHID_RETURN_ERRSTR(EPHIDGET_NOTCONFIGURED, "Protocol needs to be set before packets can be sent."));

	if ((uint32_t)length > ch->maxSendWaitPacketLength) {
		*recvDataLen = 0;
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Packet length too long."));
	}

	if (milliseconds < ch->timeout) {
		*recvDataLen = 0;
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Timeout cannot be less than the timeout set for the device."));
	}

	//if (milliseconds)
	start = mos_gettime_usec();
	PhidgetRunLock(ch);

	duration = (mos_gettime_usec() - start) / 1000;
	if (duration >= (milliseconds)) {
		PhidgetRunUnlock(ch);
		*recvDataLen = 0;
		return (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "Timed out before data could send. Other Send Data functions could be holding this one up."));
	}

	do { //retry if the packet is rejected until this function times out
		res = bridgeSendToDeviceWithReply((PhidgetChannelHandle)ch, BP_DATAEXCHANGE, NULL, NULL, (uint8_t *)response, (uint32_t *)&responseLen, "%*R", length, data);
		duration = (mos_gettime_usec() - start) / 1000;
		if (duration >= milliseconds) {
			PhidgetRunUnlock(ch);
			*recvDataLen = 0;
			return (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "Timed out before data could send. Other Send Data functions could be holding this one up."));
		}
		if (responseLen == 0)
			MOS_PANIC("The bridge packet was lost");
	} while (res == EPHIDGET_INTERRUPTED);

	if (res) {
		PhidgetRunUnlock(ch);
		*recvDataLen = 0;
		if(res == EPHIDGET_TIMEOUT)
			return (PHID_RETURN_ERRSTR(EPHIDGET_INTERRUPTED, "Data could not be sent."));
		return res;
	}

	packetID = (((uint16_t)response[0]) << 8);
	packetID |= response[1];

	//PhidgetRunLock(ch);
	PhidgetLock(ch);
	for (;;) {
		if (ch->responseID == packetID) {
			break;
		}

		/*if (!(ISATTACHED(ch))) {
			PhidgetRunUnlock(ch);
			*recvDataLen = 0;
			PhidgetUnlock(ch);
			return (EPHIDGET_CLOSED);
		}*/

		if (milliseconds) {
			duration = (mos_gettime_usec() - start) / 1000;
			if (duration >= milliseconds) {

				*recvDataLen = 0;
				PhidgetUnlock(ch);
				PhidgetRunUnlock(ch);
				return (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "Timed out before a response was received. Consider increasing the Milliseconds parameter."));
			}
			
			PhidgetTimedWait(ch, milliseconds - (uint32_t)duration);
			
		}
	}
	PhidgetUnlock(ch);

	if (*recvDataLen < ch->eventDataLen) {
		PhidgetRunUnlock(ch);
		*recvDataLen = 0;
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Receive array length too short."));
	}

	memcpy(recvData, ch->eventData, ch->eventDataLen);
	*recvDataLen = ch->eventDataLen;
	*error = ch->eventDataError;

	PhidgetRunUnlock(ch);

	return (res);

}

API_VRETURN
PhidgetDataAdapter_sendPacketWaitResponse_async(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length, uint32_t milliseconds, uint8_t *recvData, size_t *recvDataLen, int* error, Phidget_AsyncCallback fptr, void *ctx)
{
	//PhidgetReturnCode res;
	//uint8_t response[8192];
	//size_t len = 8192;

	if (fptr)
		fptr((PhidgetHandle)ch, ctx, EPHIDGET_UNSUPPORTED);

	return;

	//if (ch == NULL) {
	//	if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
	//	return;
	//}
	//if (ch->phid.class != PHIDCHCLASS_DATAADAPTER) {
	//	if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
	//	return;
	//}
	//if (!ISATTACHED(ch)) {
	//	if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
	//	return;
	//}
	//if (length > ch->maxSendPacketLength) {
	//	if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
	//	return;
	//}

	//if ((uint32_t)length > ch->maxSendPacketLength)
	//	if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);

	//if (milliseconds < ch->timeout)
	//	if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);

	////This function currently makes little sense, as response and len can't do anyhting in this context
	//res = bridgeSendToDeviceWithReply((PhidgetChannelHandle)ch, BP_DATAEXCHANGE, fptr, ctx, (uint8_t *)response, (uint32_t *)&len, "%*R%u", length, data, milliseconds);

	//if (res != EPHIDGET_OK && fptr != NULL)
	//	fptr((PhidgetHandle)ch, ctx, res);

}
