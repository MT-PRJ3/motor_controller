/*
 * This file is part of libphidget22
 *
 * Copyright 2015 Phidgets Inc <patrick@phidgets.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/>
 */

#include "phidgetbase.h"
#include "gpp.h"
#include "device/dataadapterdevice.h"
#include "class/dataadapter.gen.h"

// === Internal Functions === //

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetDataAdapterDevice_initAfterOpen(PhidgetDeviceHandle device) {

	PhidgetDataAdapterDeviceHandle phid = (PhidgetDataAdapterDeviceHandle)device;
	//PhidgetReturnCode ret;
	assert(phid);

	switch (device->deviceInfo.UDD->uid) {
	case PHIDUID_ADP_RS232_USB:
		phid->inputsEnabled = 0;
		phid->outputsEnabled = 0;
		break;
	case PHIDUID_ADP_RS485_422_USB:
		phid->inputsEnabled = 0;
		phid->outputsEnabled = 0;
		break;
	case PHIDUID_ADP_SERIAL_USB:
		phid->inputsEnabled = 0;
		phid->outputsEnabled = 0;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode checkIOValid(PhidgetChannelHandle ch) {
	PhidgetDataAdapterDeviceHandle phid = (PhidgetDataAdapterDeviceHandle)ch->parent;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_ADP_SERIAL_USB:
		if (ch->class == PHIDCHCLASS_DIGITALINPUT) {
			if ((1 << ch->index) & phid->outputsEnabled)
				return EPHIDGET_NOTCONFIGURED;
		}

		if (ch->class == PHIDCHCLASS_DIGITALOUTPUT) {
			if ((1 << ch->index) & phid->inputsEnabled)
				return EPHIDGET_NOTCONFIGURED;
		}

		if (phid->protocol == PROTOCOL_SPI) {
			if (ch->uniqueIndex >= 0 && ch->uniqueIndex <= 2)
				return EPHIDGET_NOTCONFIGURED;
		} else if (phid->protocol == PROTOCOL_I2C) {
			if (ch->uniqueIndex == 4 && ch->uniqueIndex == 5)
				return EPHIDGET_NOTCONFIGURED;
		} else if (phid->protocol == PROTOCOL_UART) {
			if (ch->uniqueIndex == 1 && ch->uniqueIndex == 2)
				return EPHIDGET_NOTCONFIGURED;
			if (phid->handshakeMode == HANDSHAKE_MODE_READY_TO_RECEIVE || phid->handshakeMode == HANDSHAKE_MODE_REQUEST_TO_SEND) {
				if (ch->uniqueIndex == 3 && ch->uniqueIndex == 4)
					return EPHIDGET_NOTCONFIGURED;
			}
		}
		return EPHIDGET_OK;
	default:
		MOS_PANIC("Unexpected device");
	}
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetDataAdapterDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetDataAdapterDeviceHandle phid = (PhidgetDataAdapterDeviceHandle)device;
	PhidgetChannelHandle channel;
	uint8_t lastInputState[DATAADAPTER_MAXINPUTS];
	PhidgetReturnCode ret;
	int i, j;

	assert(phid);
	assert(buffer);

	//Parse device packets - store data locally
	switch (device->deviceInfo.UDD->uid) {
	case PHIDUID_ADP_RS232_USB:
		if (length > 0) {
			switch (buffer[0]) {
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA_ERROR:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA_END:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_TIMEOUT:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DROPPED:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_ACK:
				if ((channel = getChannel(phid, 0)) != NULL) {
					ret = PhidgetDataAdapterSupport_dataInput(channel, buffer, length);
					PhidgetRelease(&channel);
					return ret;
				}
				return EPHIDGET_OK;
			case STATE_CHANGE:
				for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
	//				inputState[j] = PUNK_BOOL;
					lastInputState[j] = phid->inputState[j];
					phid->inputState[j] = ((buffer[1] & (1 << j)) != 0);
				}

				for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
					int chIndex = i + phid->devChannelCnts.numDataAdapters;
					if ((channel = getChannel(phid, chIndex)) != NULL) {
						if (phid->inputState[i] != PUNK_BOOL && phid->inputState[i] != lastInputState[i]) {
							bridgeSendToChannel(channel, BP_STATECHANGE, "%d", (int)(phid->inputState[i]));
						}
						PhidgetRelease(&channel);
					}
				}

				return (EPHIDGET_OK);
			case STATE_INVALID:
				for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
					//				inputState[j] = PUNK_BOOL;
					lastInputState[j] = PUNK_BOOL;
					phid->inputState[j] = PUNK_BOOL;
				}

				for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
					int chIndex = i + phid->devChannelCnts.numDataAdapters;
					if ((channel = getChannel(phid, chIndex)) != NULL) {
						SEND_ERROR_EVENT(channel, EEPHIDGET_INVALIDSTATE, "Invalid State Detected");
						PhidgetRelease(&channel);
					}
				}
				return (EPHIDGET_OK);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		}
		MOS_PANIC("Unexpected packet type");
	case PHIDUID_ADP_RS485_422_USB:
		if (length > 0) {
			switch (buffer[0]) {
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA_ERROR:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA_END:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_TIMEOUT:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DROPPED:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_ACK:
				if ((channel = getChannel(phid, 0)) != NULL) {
					ret = PhidgetDataAdapterSupport_dataInput(channel, buffer, length);
					PhidgetRelease(&channel);
					return ret;
				}
				return EPHIDGET_OK;
			default:
				MOS_PANIC("Unexpected packet type");
			}
		}
		MOS_PANIC("Unexpected packet type");
	case PHIDUID_ADP_SERIAL_USB:
		if (length > 0) {
			switch (buffer[0]) {
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA_ERROR:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA_END:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_TIMEOUT:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DROPPED:
			case VINT_PACKET_TYPE_DATAADAPTER_PACKET_ACK:
				if ((channel = getChannel(phid, 0)) != NULL) {
					ret = PhidgetDataAdapterSupport_dataInput(channel, buffer, length);
					PhidgetRelease(&channel);
					return ret;
				}
				return EPHIDGET_OK;
			case VINT_PACKET_TYPE_DATAADAPTER_VOLTAGE_ERROR:
				if ((channel = getChannel(phid, 0)) != NULL) {
					SEND_ERROR_EVENT(channel, EEPHIDGET_VOLTAGEERROR, "Voltage Error Detected");
					PhidgetRelease(&channel);
				}
				return (EPHIDGET_OK);
			case STATE_CHANGE:
				for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
					//				inputState[j] = PUNK_BOOL;
					lastInputState[j] = phid->inputState[j];
					phid->inputState[j] = ((buffer[1] & (1 << j)) != 0);
				}

				for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
					int chIndex = i + phid->devChannelCnts.numDataAdapters;
					if ((channel = getChannel(phid, chIndex)) != NULL) {
						if (phid->inputState[i] != PUNK_BOOL && phid->inputState[i] != lastInputState[i]) {
							bridgeSendToChannel(channel, BP_STATECHANGE, "%d", (int)(phid->inputState[i]));
						}
						PhidgetRelease(&channel);
					}
				}
				return (EPHIDGET_OK);
			case STATE_INVALID:
				for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
					//				inputState[j] = PUNK_BOOL;
					lastInputState[j] = PUNK_BOOL;
					phid->inputState[j] = PUNK_BOOL;
				}

				for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
					int chIndex = i + phid->devChannelCnts.numDataAdapters;
					if ((channel = getChannel(phid, chIndex)) != NULL) {
						SEND_ERROR_EVENT(channel, EEPHIDGET_INVALIDSTATE, "Invalid State Detected");
						PhidgetRelease(&channel);
					}
				}
				return (EPHIDGET_OK);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		}
		MOS_PANIC("Unexpected packet type");
	default:
		MOS_PANIC("Unexpected device");
	}

}

static PhidgetReturnCode CCONV
PhidgetDataAdapterDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetDataAdapterDeviceHandle phid = (PhidgetDataAdapterDeviceHandle)ch->parent;
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	PhidgetChannelHandle channel;
	PhidgetReturnCode ret;
	double dutyCycle;
	int32_t state;
	size_t len;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_DATAADAPTER);


	switch (((PhidgetDeviceHandle)phid)->deviceInfo.UDD->uid) {
	case PHIDUID_ADP_RS232_USB:
		switch (ch->class) {
		case PHIDCHCLASS_DATAADAPTER:
			if ((channel = getChannel(phid, 0)) != NULL) {
				ret = PhidgetDataAdapterSupport_bridgeInput(channel, bp);
				PhidgetRelease(&channel);
				return ret;
			}
			return EPHIDGET_OK;
		case PHIDCHCLASS_DIGITALINPUT:
			switch (bp->vpkt) {
			case BP_OPENRESET:
			case BP_CLOSERESET:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, ch->uniqueIndex, buffer, &len, 100);
			case BP_ENABLE:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, ch->uniqueIndex, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		case PHIDCHCLASS_DIGITALOUTPUT:
			switch (bp->vpkt) {
			case BP_SETDUTYCYCLE:
				dutyCycle = getBridgePacketDouble(bp, 0);
				if (dutyCycle != 0 && dutyCycle != 1)
					return EPHIDGET_INVALIDARG;
				buffer[0] = (uint8_t)dutyCycle;
				len = 1;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_SETSTATE:
				if (phid->protocol != PROTOCOL_RS232)
					return EPHIDGET_NOTCONFIGURED;
				state = getBridgePacketInt32(bp, 0);
				if (state != 0 && state != 1)
					return EPHIDGET_INVALIDARG;
				buffer[0] = (uint8_t)state;
				len = 1;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_OPENRESET:
			case BP_CLOSERESET:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, ch->uniqueIndex, buffer, &len, 100);
			case BP_ENABLE:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, ch->uniqueIndex, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		default:
			MOS_PANIC("Unexpected Channel Class");
		}
	case PHIDUID_ADP_RS485_422_USB:
		if ((channel = getChannel(phid, 0)) != NULL) {
			ret = PhidgetDataAdapterSupport_bridgeInput(channel, bp);
			PhidgetRelease(&channel);
			return ret;
		}
		return EPHIDGET_OK;
	case PHIDUID_ADP_SERIAL_USB:
		switch (ch->class) {
		case PHIDCHCLASS_DATAADAPTER:
			switch (bp->vpkt) {
			case BP_DATAOUT:
			case BP_DATAEXCHANGE:
			case BP_SETBAUDRATE:
			case BP_SETDATABITS:
			case BP_SETSPIMODE:
			case BP_SETENDIANNESS:
			case BP_SETTIMEOUT:
			case BP_SETPARITY:
			case BP_SETSTOPBITS:
			case BP_SETI2CFORMAT:
			case BP_SETIOVOLTAGE:
				break;
			case BP_OPENRESET:
			case BP_CLOSERESET:
				phid->protocol = PUNK_ENUM;
				phid->handshakeMode = HANDSHAKE_MODE_NONE;
				break;
			case BP_ENABLE:
				break;
			case BP_SETPROTOCOL:
				phid->protocol = (uint8_t)getBridgePacketInt32(bp, 0);
				switch (phid->protocol) {
				case PROTOCOL_SPI:
					if ((phid->outputsEnabled | phid->inputsEnabled) & ((1 << 0) | (1 << 1) | (1 << 2)))
						return EPHIDGET_NOTCONFIGURED;
					break;
				case PROTOCOL_I2C:
					if ((phid->outputsEnabled | phid->inputsEnabled) & ((1 << 3) | (1 << 4)))
						return EPHIDGET_NOTCONFIGURED;
					break;
				case PROTOCOL_UART:
					if ((phid->outputsEnabled | phid->inputsEnabled) & ((1 << 1) | (1 << 2)))
						return EPHIDGET_NOTCONFIGURED;
					if (phid->handshakeMode == HANDSHAKE_MODE_READY_TO_RECEIVE || phid->handshakeMode == HANDSHAKE_MODE_REQUEST_TO_SEND) {
						if (ch->uniqueIndex == 3 && ch->uniqueIndex == 4)
							return EPHIDGET_NOTCONFIGURED;
					}
					break;
				}
				break;
			case BP_SETHANDSHAKEMODE:
				if (phid->protocol != PROTOCOL_UART)
					return EPHIDGET_NOTCONFIGURED;
				if ((phid->outputsEnabled | phid->inputsEnabled) & ((1 << 3) | (1 << 4)))
					return EPHIDGET_NOTCONFIGURED;
				phid->handshakeMode = (uint8_t)getBridgePacketInt32(bp, 0);
				break;
			case BP_SETADDRESS:
				if (phid->protocol != PROTOCOL_SPI && phid->protocol != PROTOCOL_I2C)
					return EPHIDGET_NOTCONFIGURED;
				//if (phid->protocol == PROTOCOL_SPI)
#pragma message ("TODO - Lock out SPI addresses if GPIO in use")

				break;
			default:
				break;
			}
			if ((channel = getChannel(phid, 0)) != NULL) {
				ret = PhidgetDataAdapterSupport_bridgeInput(channel, bp);
				PhidgetRelease(&channel);
				return ret;
			}
			return EPHIDGET_OK;
		case PHIDCHCLASS_DIGITALINPUT:
			switch (bp->vpkt) {
				case BP_OPENRESET:
				case BP_CLOSERESET:
					ret = checkIOValid(ch);
					if (ret)
						return ret;
					phid->inputsEnabled &= ~(1 << ch->index);
					len = 0;
					return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, ch->uniqueIndex, buffer, &len, 100);
				case BP_ENABLE:
					ret = checkIOValid(ch);
					if (ret)
						return ret;
					phid->inputsEnabled |= (1 << ch->index);
					len = 0;
					return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, ch->uniqueIndex, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		case PHIDCHCLASS_DIGITALOUTPUT:
			switch (bp->vpkt) {
			case BP_SETDUTYCYCLE:
				ret = checkIOValid(ch);
				if (ret)
					return ret;
				if (phid->protocol != PROTOCOL_UART)
					return EPHIDGET_NOTCONFIGURED;
				dutyCycle = getBridgePacketDouble(bp, 0);
				if (dutyCycle != 0 && dutyCycle != 1)
					return EPHIDGET_INVALIDARG;
				buffer[0] = (uint8_t)dutyCycle;
				len = 1;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_SETSTATE:
				ret = checkIOValid(ch);
				if (ret)
					return ret;
				if (phid->protocol != PROTOCOL_UART)
					return EPHIDGET_NOTCONFIGURED;
				state = getBridgePacketInt32(bp, 0);
				if (state != 0 && state != 1)
					return EPHIDGET_INVALIDARG;
				buffer[0] = (uint8_t)state;
				len = 1;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_OPENRESET:
			case BP_CLOSERESET:
				ret = checkIOValid(ch);
				if (ret)
					return ret;
				phid->outputsEnabled &= ~(1 << ch->index);
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, ch->uniqueIndex, buffer, &len, 100);
			case BP_ENABLE:
				ret = checkIOValid(ch);
				if (ret)
					return ret;
				phid->outputsEnabled |= (1 << ch->index);
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, ch->uniqueIndex, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		default:
			MOS_PANIC("Unexpected Channel Class");
		}
	default:
		MOS_PANIC("Unexpected device");
	}
}

static void CCONV
PhidgetDataAdapterDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetDataAdapterDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetDataAdapterDevice_create(PhidgetDataAdapterDeviceHandle *phidp) {
	DEVICECREATE_BODY(DataAdapterDevice, PHIDCLASS_DATAADAPTER);
	return (EPHIDGET_OK);
}
