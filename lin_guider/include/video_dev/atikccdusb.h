/*
 * ATIK CCD INDI Driver
 *
 * Copyright (c) 2013 CloudMakers, s. r. o. All Rights Reserved.
 *
 * The code is based upon Linux library source developed by
 * Jonathan Burch, Artemis CCD Ltd. It is provided by CloudMakers
 * and contributors "AS IS", without warranty of any kind.
 */

#ifndef ATIKCCDUSB_H_
#define ATIKCCDUSB_H_

#define GUIDE_EAST             0x04     /* RA+ */
#define GUIDE_NORTH            0x01     /* DEC+ */
#define GUIDE_SOUTH            0x02     /* DEC- */
#define GUIDE_WEST             0x08     /* RA- */
#define GUIDE_CLEAR_WE         (0x0F & ~(GUIDE_WEST | GUIDE_EAST))
#define GUIDE_CLEAR_NS         (0x0F & ~(GUIDE_NORTH | GUIDE_SOUTH))

#define QUICKER_START_EXPOSURE_DELAY  1
#define QUICKER_READ_CCD_DELAY        2
#define MAX_PACKET_SIZE               3

extern bool AtikDebug;
extern bool AtikDebugOn;

enum CAMERA_TYPE {
	ORIGINAL_HSC = 1, IC24, QUICKER
};

enum COOLER_TYPE {
  COOLER_NONE, COOLER_ALWAYSON, COOLER_ONOFF, COOLER_SELECTPOWER, COOLER_SETPOINT
};

enum COOLING_STATE {
  COOLING_INACTIVE, COOLING_ON, COOLING_SETPOINT, WARMING_UP
};

class AtikCamera {
	public:
		static int list(AtikCamera **cameras, int max);
		virtual const char *getName() = 0;
		virtual bool open() = 0;
		virtual void close() = 0;
		virtual bool setParam(int code, long value) = 0;
		virtual bool getCapabilities(const char **name, CAMERA_TYPE *type, bool *hasShutter, bool* hasGuidePort, unsigned* pixelCountX, unsigned* pixelCountY, double* pixelSizeX, double* pixelSizeY, unsigned* maxBinX, unsigned* maxBinY, COOLER_TYPE* cooler) = 0;
		virtual bool initiateWarmUp() = 0;
		virtual bool getCoolerState(COOLING_STATE *state, float* targetTemp, float* currentTemp) = 0;
		virtual bool setCooling(float targetTemp) = 0;
		virtual bool startExposure(bool amp) = 0;
		virtual bool abortExposure() = 0;
		virtual bool readCCD(unsigned startX, unsigned startY, unsigned sizeX, unsigned sizeY, unsigned binX, unsigned binY) = 0;
		virtual bool readCCD(unsigned startX, unsigned startY, unsigned sizeX, unsigned sizeY, unsigned binX, unsigned binY, double delay) = 0;
		virtual bool getImage(unsigned short* imgBuf, unsigned imgSize) = 0;
		virtual bool setShutter(bool open) = 0;
		virtual bool setGuideRelays(short mask) = 0;
		virtual unsigned int delay(double delay) = 0;
    virtual ~AtikCamera() { };
};


extern "C" int AtikCamera_list(AtikCamera **cameras, int max);
extern "C" void AtikCamera_destroy(AtikCamera *camera);

#endif
