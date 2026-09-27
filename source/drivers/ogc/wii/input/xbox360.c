#ifdef HW_RVL
#include <gccore.h>
#include <ogc/usb.h>
#include <string.h>

#define USB_CLASS_XBOX 0xFF
#define MICROSOFT_VID 0x045e
#define XBOX360_PID 0x028e
#define XBOX360_V2_PID 0x028f

#define GIP_CMD_VIRTUAL_KEY 0x07
#define GIP_CMD_INPUT 0x20

#define XBOX_ONE_PACKET_SIZE 64
#define XBOX360_PACKET_SIZE 20
#define STICK_THRESHOLD 16384
#define TRIGGER_THRESHOLD 512

typedef enum
{
	XBOX_TYPE_NONE = 0,
	XBOX_TYPE_360,
	XBOX_TYPE_ONE
} xbox_type;

typedef struct
{
	u16 vid;
	u16 pid;
	xbox_type type;
	bool needsSInit;
} xbox_device;

/*
 * Microsoft Xbox One-family USB IDs supported by the Linux xpad driver.
 * The first hardware target for this Wii port is 045e:02ea (Xbox One S).
 * Other entries use the same GIP input layout and can be tested incrementally.
 */
static const xbox_device supportedDevices[] = {
	{ MICROSOFT_VID, XBOX360_PID,    XBOX_TYPE_360, false },
	{ MICROSOFT_VID, XBOX360_V2_PID, XBOX_TYPE_360, false },
	{ MICROSOFT_VID, 0x02d1,         XBOX_TYPE_ONE, false }, // Xbox One
	{ MICROSOFT_VID, 0x02dd,         XBOX_TYPE_ONE, false }, // Xbox One, 2015 firmware
	{ MICROSOFT_VID, 0x02e3,         XBOX_TYPE_ONE, false }, // Xbox One Elite
	{ MICROSOFT_VID, 0x02ea,         XBOX_TYPE_ONE, true  }, // Xbox One S
	{ MICROSOFT_VID, 0x0b00,         XBOX_TYPE_ONE, true  }, // Xbox One Elite Series 2
};

static const int supportedDeviceCount = sizeof(supportedDevices) / sizeof(supportedDevices[0]);

static bool setup = false;
static bool replugRequired = false;
static s32 deviceId = 0;
static u8 endpoint_in = 0x81;
static u8 endpoint_out = 0x01;
static u8 bMaxPacketSize = XBOX360_PACKET_SIZE;
static u8 bConfigurationValue = 1;
static u8 ATTRIBUTE_ALIGN(32) buf[XBOX_ONE_PACKET_SIZE];
static bool isReading = false;
static u32 jp = 0;
static u8 player = 0;
static u32 xboxButtonCount = 0;
static bool nextPlayer = false;
static bool guidePressed = false;
static xbox_type controllerType = XBOX_TYPE_NONE;
static const xbox_device *activeDevice = NULL;

static const xbox_device *findSupportedDevice(usb_device_entry dev)
{
	for (int i = 0; i < supportedDeviceCount; ++i)
	{
		if (dev.vid == supportedDevices[i].vid && dev.pid == supportedDevices[i].pid)
		{
			return &supportedDevices[i];
		}
	}
	return NULL;
}

static bool isExpectedGamepad(usb_devdesc devdesc, const xbox_device *dev)
{
	return dev != NULL && devdesc.idVendor == dev->vid && devdesc.idProduct == dev->pid;
}

/* Find the interrupt IN/OUT endpoints on the controller data interface. */
static bool discoverEndpoints(usb_devdesc *devdesc)
{
	bool foundIn = false;
	bool foundOut = false;

	if (devdesc == NULL || devdesc->configurations == NULL)
	{
		return false;
	}

	for (unsigned char c = 0; c < devdesc->bNumConfigurations; c++)
	{
		const usb_configurationdesc *config = &devdesc->configurations[c];
		if (config->interfaces == NULL)
			continue;

		for (unsigned i = 0; i < (unsigned)config->bNumInterfaces; i++)
		{
			const usb_interfacedesc *inter = &config->interfaces[i];
			if (inter->endpoints == NULL)
				continue;

			for (unsigned k = 0; k < (unsigned)inter->bNumEndpoints; k++)
			{
				const usb_endpointdesc *epdesc = &inter->endpoints[k];
				bool isInt = (epdesc->bmAttributes & 0x03) == USB_ENDPOINT_INTERRUPT;
				if (!isInt)
					continue;

				if (!foundIn && (epdesc->bEndpointAddress & 0x80) == USB_ENDPOINT_IN)
				{
					endpoint_in = epdesc->bEndpointAddress;
					u16 packetSize = epdesc->wMaxPacketSize;
					if (packetSize > sizeof(buf))
						packetSize = sizeof(buf);
					if (packetSize > 0)
						bMaxPacketSize = (u8)packetSize;
					foundIn = true;
				}
				else if (!foundOut && (epdesc->bEndpointAddress & 0x80) == USB_ENDPOINT_OUT)
				{
					endpoint_out = epdesc->bEndpointAddress;
					foundOut = true;
				}
			}

			if (foundIn && foundOut)
				return true;
		}
	}

	return foundIn && foundOut;
}

static int writePacket(const u8 *data, u8 len)
{
	if (deviceId == 0 || data == NULL || len == 0 || len > 32)
		return -1;

	u8 ATTRIBUTE_ALIGN(32) out[32];
	memset(out, 0, sizeof(out));
	memcpy(out, data, len);
	return USB_WriteIntrMsg(deviceId, endpoint_out, len, out);
}

static void initXboxOne()
{
	/* GIP power-on packet required by Xbox One controllers with newer firmware. */
	static const u8 powerOn[] = { 0x05, 0x20, 0x00, 0x01, 0x00 };
	/* Required by Xbox One S / Elite 2 after the controller has used Bluetooth. */
	static const u8 oneSInit[] = { 0x05, 0x20, 0x00, 0x0f, 0x06 };

	writePacket(powerOn, sizeof(powerOn));
	if (activeDevice != NULL && activeDevice->needsSInit)
	{
		writePacket(oneSInit, sizeof(oneSInit));
	}
}

static int read(s32 device_id, u8 endpoint, u8 packetSize);

static void start_reading(s32 device_id, u8 endpoint, u8 packetSize)
{
	if (isReading)
	{
		return;
	}
	isReading = true;
	read(device_id, endpoint, packetSize);
}

static void stop_reading()
{
	isReading = false;
}

static int16_t readS16LE(const u8 *data)
{
	return (int16_t)((u16)data[0] | ((u16)data[1] << 8));
}

static u16 readU16LE(const u8 *data)
{
	return (u16)data[0] | ((u16)data[1] << 8);
}

static void parseXbox360Packet(int res)
{
	if (res != XBOX360_PACKET_SIZE)
		return;

	jp = 0;
	jp |= (buf[2] & 0x01) ? PAD_BUTTON_UP    : 0;
	jp |= (buf[2] & 0x02) ? PAD_BUTTON_DOWN  : 0;
	jp |= (buf[2] & 0x04) ? PAD_BUTTON_LEFT  : 0;
	jp |= (buf[2] & 0x08) ? PAD_BUTTON_RIGHT : 0;

	jp |= (buf[3] & 0x10) ? PAD_BUTTON_B : 0; // Xbox A -> SNES B
	jp |= (buf[3] & 0x20) ? PAD_BUTTON_A : 0; // Xbox B -> SNES A
	jp |= (buf[3] & 0x40) ? PAD_BUTTON_Y : 0; // Xbox X -> SNES Y
	jp |= (buf[3] & 0x80) ? PAD_BUTTON_X : 0; // Xbox Y -> SNES X

	jp |= (buf[3] & 0x01) ? PAD_TRIGGER_L : 0;
	jp |= (buf[3] & 0x02) ? PAD_TRIGGER_R : 0;
	jp |= (buf[2] & 0x10) ? PAD_BUTTON_START : 0;
	jp |= (buf[2] & 0x20) ? PAD_TRIGGER_Z : 0;

	jp |= (buf[4] > 128) ? PAD_TRIGGER_L : 0;
	jp |= (buf[5] > 128) ? PAD_TRIGGER_R : 0;

	int16_t lx = readS16LE(&buf[6]);
	int16_t ly = readS16LE(&buf[8]);
	int16_t rx = readS16LE(&buf[10]);
	int16_t ry = readS16LE(&buf[12]);

	jp |= (ly >  STICK_THRESHOLD) ? PAD_BUTTON_UP    : 0;
	jp |= (ly < -STICK_THRESHOLD) ? PAD_BUTTON_DOWN  : 0;
	jp |= (lx < -STICK_THRESHOLD) ? PAD_BUTTON_LEFT  : 0;
	jp |= (lx >  STICK_THRESHOLD) ? PAD_BUTTON_RIGHT : 0;
	jp |= (ry >  STICK_THRESHOLD) ? PAD_BUTTON_UP    : 0;
	jp |= (ry < -STICK_THRESHOLD) ? PAD_BUTTON_DOWN  : 0;
	jp |= (rx < -STICK_THRESHOLD) ? PAD_BUTTON_LEFT  : 0;
	jp |= (rx >  STICK_THRESHOLD) ? PAD_BUTTON_RIGHT : 0;

	if (buf[3] & 0x04)
	{
		xboxButtonCount++;
		if (xboxButtonCount >= 2)
		{
			nextPlayer = true;
			xboxButtonCount = 0;
		}
	}
}

static void parseXboxOnePacket(int res)
{
	if (res < 5)
		return;

	/* Guide/Xbox button is reported separately from the normal 0x20 input packet. */
	if (buf[0] == GIP_CMD_VIRTUAL_KEY)
	{
		bool pressed = (buf[4] & 0x01) != 0;
		if (pressed && !guidePressed)
			nextPlayer = true;
		guidePressed = pressed;
		return;
	}

	/* Standard GIP button report: 4-byte header + 14-byte payload. */
	if (buf[0] != GIP_CMD_INPUT || res < 18)
		return;

	jp = 0;

	/* b[4]: Menu, View, A, B, X, Y */
	jp |= (buf[4] & 0x04) ? PAD_BUTTON_START : 0;
	jp |= (buf[4] & 0x08) ? PAD_TRIGGER_Z : 0;
	jp |= (buf[4] & 0x10) ? PAD_BUTTON_B : 0; // Xbox A -> SNES B
	jp |= (buf[4] & 0x20) ? PAD_BUTTON_A : 0; // Xbox B -> SNES A
	jp |= (buf[4] & 0x40) ? PAD_BUTTON_Y : 0; // Xbox X -> SNES Y
	jp |= (buf[4] & 0x80) ? PAD_BUTTON_X : 0; // Xbox Y -> SNES X

	/* b[5]: D-pad, bumpers and stick clicks */
	jp |= (buf[5] & 0x01) ? PAD_BUTTON_UP    : 0;
	jp |= (buf[5] & 0x02) ? PAD_BUTTON_DOWN  : 0;
	jp |= (buf[5] & 0x04) ? PAD_BUTTON_LEFT  : 0;
	jp |= (buf[5] & 0x08) ? PAD_BUTTON_RIGHT : 0;
	jp |= (buf[5] & 0x10) ? PAD_TRIGGER_L : 0;
	jp |= (buf[5] & 0x20) ? PAD_TRIGGER_R : 0;

	u16 lt = readU16LE(&buf[6]);
	u16 rt = readU16LE(&buf[8]);
	jp |= (lt > TRIGGER_THRESHOLD) ? PAD_TRIGGER_L : 0;
	jp |= (rt > TRIGGER_THRESHOLD) ? PAD_TRIGGER_R : 0;

	int16_t lx = readS16LE(&buf[10]);
	int16_t ly = readS16LE(&buf[12]);
	int16_t rx = readS16LE(&buf[14]);
	int16_t ry = readS16LE(&buf[16]);

	jp |= (ly >  STICK_THRESHOLD) ? PAD_BUTTON_UP    : 0;
	jp |= (ly < -STICK_THRESHOLD) ? PAD_BUTTON_DOWN  : 0;
	jp |= (lx < -STICK_THRESHOLD) ? PAD_BUTTON_LEFT  : 0;
	jp |= (lx >  STICK_THRESHOLD) ? PAD_BUTTON_RIGHT : 0;
	jp |= (ry >  STICK_THRESHOLD) ? PAD_BUTTON_UP    : 0;
	jp |= (ry < -STICK_THRESHOLD) ? PAD_BUTTON_DOWN  : 0;
	jp |= (rx < -STICK_THRESHOLD) ? PAD_BUTTON_LEFT  : 0;
	jp |= (rx >  STICK_THRESHOLD) ? PAD_BUTTON_RIGHT : 0;
}

static int read_cb(int res, void *usrdata)
{
	if (!isReading)
	{
		return 1;
	}

	if (controllerType == XBOX_TYPE_ONE)
		parseXboxOnePacket(res);
	else if (controllerType == XBOX_TYPE_360)
		parseXbox360Packet(res);

	if (deviceId != 0 && isReading)
		read(deviceId, endpoint_in, bMaxPacketSize);

	return 1;
}

static int read(s32 device_id, u8 endpoint, u8 packetSize)
{
	u32 len = packetSize;
	if (len == 0 || len > sizeof(buf))
		len = sizeof(buf);

	/* Async is required because USB_ReadIntrMsg blocks until input changes. */
	return USB_ReadIntrMsgAsync(device_id, endpoint, len, buf, &read_cb, NULL);
}

static void turnOnLED()
{
	if (controllerType != XBOX_TYPE_360)
		return;

	uint8_t ATTRIBUTE_ALIGN(32) led[] = { 0x01, 0x03, 0x06 + player };
	USB_WriteIntrMsg(deviceId, endpoint_out, sizeof(led), led);
}

static void increasePlayer()
{
	player++;
	if (player > 3)
		player = 0;

	turnOnLED();
}

void rumble(s32 device_id, u8 left, u8 right)
{
	if (controllerType != XBOX_TYPE_360)
		return;

	uint8_t ATTRIBUTE_ALIGN(32) rumblePacket[] = { 0x00, 0x08, 0x00, left, right, 0x00, 0x00, 0x00 };
	USB_WriteIntrMsg(device_id, endpoint_out, sizeof(rumblePacket), rumblePacket);
}

static int removal_cb(int result, void *usrdata)
{
	s32 fd = (s32)usrdata;
	if (fd == deviceId)
	{
		stop_reading();
		deviceId = 0;
		controllerType = XBOX_TYPE_NONE;
		activeDevice = NULL;
		jp = 0;
		guidePressed = false;
	}
	return 1;
}

static void open()
{
	if (deviceId != 0)
		return;

	usb_device_entry dev_entry[8];
	u8 dev_count;
	if (USB_GetDeviceList(dev_entry, 8, USB_CLASS_XBOX, &dev_count) < 0)
		return;

	for (int i = 0; i < dev_count; ++i)
	{
		const xbox_device *candidate = findSupportedDevice(dev_entry[i]);
		if (candidate == NULL)
			continue;

		s32 fd;
		if (USB_OpenDevice(dev_entry[i].device_id, dev_entry[i].vid, dev_entry[i].pid, &fd) < 0)
			continue;

		usb_devdesc devdesc;
		if (USB_GetDescriptors(fd, &devdesc) < 0)
		{
			replugRequired = true;
			USB_CloseDevice(&fd);
			break;
		}

		controllerType = candidate->type;
		activeDevice = candidate;
		endpoint_in = 0x81;
		endpoint_out = 0x01;
		bMaxPacketSize = (controllerType == XBOX_TYPE_ONE) ? XBOX_ONE_PACKET_SIZE : XBOX360_PACKET_SIZE;

		bool endpointsOK = discoverEndpoints(&devdesc);
		if (isExpectedGamepad(devdesc, candidate) && endpointsOK &&
			USB_SetConfiguration(fd, bConfigurationValue) >= 0)
		{
			deviceId = fd;
			replugRequired = false;
			jp = 0;
			guidePressed = false;

			if (controllerType == XBOX_TYPE_ONE)
				initXboxOne();
			else
				turnOnLED();

			USB_DeviceRemovalNotifyAsync(fd, &removal_cb, (void *)fd);
			break;
		}
		else
		{
			controllerType = XBOX_TYPE_NONE;
			activeDevice = NULL;
			USB_CloseDevice(&fd);
		}
	}

	setup = true;
}

void XBOX360_ScanPads()
{
	if (deviceId == 0)
		return;

	start_reading(deviceId, endpoint_in, bMaxPacketSize);
}

u32 XBOX360_ButtonsHeld(int chan)
{
	if (!setup)
		open();

	if (deviceId == 0)
		return 0;

	if (nextPlayer)
	{
		nextPlayer = false;
		increasePlayer();
	}

	if (chan != player)
		return 0;

	return jp;
}

char *XBOX360_Status()
{
	open();
	if (replugRequired)
		return "please replug";
	if (deviceId == 0)
		return "not found";
	return controllerType == XBOX_TYPE_ONE ? "connected (Xbox One)" : "connected";
}

#endif
