#if 1
#include <math.h>
#endif
#include <time.h>

#include <cstdio>

#include <AK8963_Magnetometer.h>

static const float ak8963_scale[] = {
	[MFS_14BITS] = 0.6,
	[MFS_16BITS] = 0.15,
};

AK8963_Magnetometer::AK8963_Magnetometer(const char * i2cDeviceFilePath) :
	i2cObject(i2cDeviceFilePath)
{
//	raw.x = 0;
//	raw.y = 0;
//	raw.z = 0;
  i2cObject.addressSet(AK8963_ADDRESS);
}

//#define SELF_TEST

void AK8963_Magnetometer::begin(void) {
  // Specify sensor full scale
  const uint8_t Mscale = MFS_16BITS;          // 16-bit magnetometer resolution
  const uint8_t Mmode = 0x02;                 // 2 for 8 Hz, 6 for 100 Hz continuous
                                      // magnetometer data read
#ifdef SELF_TEST
  const uint8_t CntlSelfTest = 0x08;          // 16-bit magnetometer resolution
#endif

  i2cObject.writeByte(AK8963_CNTL, 0x0F);
  i2cObject.readBlock(AK8963_ASAX, sizeof(adjust), adjust);
  printf("adjust 0x%x 0x%x 0x%x\n", adjust[0], adjust[1], adjust[2]);
#ifndef SELF_TEST
  i2cObject.writeByte(AK8963_CNTL, Mscale << 4 | Mmode);
#else
  i2cObject.writeByte(AK8963_CNTL, CntlSelfTest);
  i2cObject.writeByte(AK8963_ASTC, 0x40);
#endif
}

#define ak8963_adjust(x, a) ((((float)(a) - 128.) / 256. + 1.) * (x))

void AK8963_Magnetometer::read(void) {
  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  uint8_t block[7];
  // wait for magnetometer data ready bit to be set
#if 0
  unsigned int i;
  for (i = 100; i && !(i2cObject.readByte(AK8963_ST1) & 0x01); i--) {
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 1000000; // 1ms

    nanosleep(&ts, NULL);
  }
  if (i)
#else
  if (i2cObject.readByte(AK8963_ST1) & 0x01)
#endif
  {
    // Read the six raw data and ST2 registers sequentially into data array
    // End data read by reading ST2 register
    i2cObject.readBlock(AK8963_XOUT_L, sizeof(block), block);
    uint8_t c = block[6];
    // Check if magnetic sensor overflow set, if not then report data
    if (!(c & 0x08)) {
      // Turn the MSB and LSB into a signed 16-bit value
      // Data stored as little Endian
      short int x, y, z;
      x = ((int16_t)block[1] << 8) | block[0];
      y = ((int16_t)block[3] << 8) | block[2];
      z = ((int16_t)block[5] << 8) | block[4];

      raw.x = ak8963_adjust(x, adjust[0]) * ak8963_scale[MFS_16BITS];
      raw.y = ak8963_adjust(y, adjust[1]) * ak8963_scale[MFS_16BITS];
      raw.z = ak8963_adjust(z, adjust[2]) * ak8963_scale[MFS_16BITS];
//      printf("Mag %hd %hd %hd after %d\n", raw.x, raw.y, raw.z, 100 - i);
//      printf("Mag %x %x %x angle %f\n", x, y, z, atan2f(-raw.y, raw.x));
//      printf("Mag %f %f %f\n", ak8963_adjust(x, adjust[0]),
//	     ak8963_adjust(y, adjust[1]),
//	     ak8963_adjust(z, adjust[2]));
      printf("Mag %f %f %f angle %f\n", raw.x, raw.y, raw.z, atan2f(-raw.y, raw.x));
    } else {
      printf("Mag overflow\n");
    }
#if 0
  } else {
    printf("Mag no data\n");
#endif
  }
}
