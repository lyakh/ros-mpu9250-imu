#include <limits.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#include <pigpio.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <exceptions.h>
#include <AK8963_Magnetometer.h>
#include <MPU9250_Acc_Gyro.h>
#include <mpu9250_9dof_imu/mpu9250.h>
//#include <AK8963_Magnetometer.cpp>
//#include <MPU9250_Acc_Gyro.cpp>
//#include <I2CBus.cpp>

/* Constants */
#define PI                                3.14159265F
#define GYRO_SENSITIVITY_2000DPS          0.070F
#define SENSORS_GRAVITY_EARTH             9.80665F              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH / 0x4000)
#define SENSORS_DPS_TO_RADS               (PI / 180.)             /**< Degrees/s to rad/s multiplier */
#define SENSORS_MILIGAUSS_TO_TESLA        10000000


const int mpu9250_interrupt_pin = 26;

/* Calibration results */
const float scale_x = 0.999;
const float offset_x = -837.447;
const float scale_y = 0.988;
const float offset_y = -82.552;
const float scale_z = 0.981;
const float offset_z = 905.131;

void mpu9250_imu_read(struct mpu9250 *imu_d)
{
    float roll, yaw, pitch;
    float acc_x, acc_y, acc_z;
    sensor_msgs::Imu imu;

    imu_d->acc_gyro->read();
    imu_d->mag->read();

    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "imu_link";

    imu.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    imu.angular_velocity_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    imu.linear_acceleration_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};

    acc_x = scale_x * imu_d->acc_gyro->raw.acc_x + offset_x;
    acc_y = scale_y * imu_d->acc_gyro->raw.acc_y + offset_y;
    acc_z = scale_z * imu_d->acc_gyro->raw.acc_z + offset_z;

    roll = (float)atan2(acc_y, acc_z);
    imu.orientation.x = roll;

    float den = acc_y * sin(roll) + acc_z * cos(roll);
    if (den) {
      pitch = (float)atan(-acc_x / den);
    } else {
      if (acc_x > 0) {
        pitch = PI / 2;
      } else {
        pitch = -PI / 2;
      }
    }

    imu.orientation.y = pitch;

#if 0
    yaw = (float)atan2(imu_d->mag->raw.z * sin(roll) - imu_d->mag->raw.y * cos(roll),
		       imu_d->mag->raw.x * cos(pitch) +
		       imu_d->mag->raw.y * sin(pitch) * sin(roll) +
		       imu_d->mag->raw.z * sin(pitch) * cos(roll));
#else
    yaw = atan2f(-imu_d->mag->raw.y, imu_d->mag->raw.x);
#endif
    imu.orientation.z = yaw;

    imu.orientation.w = 1;

    imu.angular_velocity.x = imu_d->acc_gyro->raw.gyr_x * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
    imu.angular_velocity.y = imu_d->acc_gyro->raw.gyr_y * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
    imu.angular_velocity.z = imu_d->acc_gyro->raw.gyr_z * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;

    imu.linear_acceleration.x = acc_x * SENSORS_GRAVITY_STANDARD;
    imu.linear_acceleration.y = acc_y * SENSORS_GRAVITY_STANDARD;
    imu.linear_acceleration.z = acc_z * SENSORS_GRAVITY_STANDARD;

    imu_d->imu_pub.publish(imu);

    imu_d->count++;
}

static void interrupt_callback(int user_gpio, int level,
			       unsigned int tick, void *userdata)
{
  struct mpu9250 *imu_d = (struct mpu9250 *)userdata;

  if (!ros::ok())
    return;

//  int ret = gpioRead(mpu9250_interrupt_pin);
//  printf("IRQ: Interrupt pin %d\n", ret);

//  struct timespec ts;
//  ts.tv_sec = 0;
//  ts.tv_nsec = 10000000; // 10ms
//
//  nanosleep(&ts, NULL);

#ifdef PUBLISH_FROM_INTERRUPT
  mpu9250_imu_read(imu_d);
#endif
}

int mpu9250_init(ros::NodeHandle &nh, struct mpu9250 &imu_d)
{
  const char* i2cDevice = "/dev/i2c-1";
  int ret;

  imu_d.acc_gyro = new MPU9250_Acc_Gyro(i2cDevice);
  imu_d.mag = new AK8963_Magnetometer(i2cDevice);

  imu_d.acc_gyro->begin();
  imu_d.mag->begin();

  ret = gpioSetPullUpDown(mpu9250_interrupt_pin, PI_PUD_DOWN);
  if (ret < 0)
    throw posix_error("initialize: Pull down failed.");

  ret = gpioRead(mpu9250_interrupt_pin);
  printf("Interrupt pin %d: %d\n", mpu9250_interrupt_pin, ret);

  ret = gpioSetISRFuncEx(mpu9250_interrupt_pin, RISING_EDGE, 0, interrupt_callback, &imu_d);
  if (ret < 0)
    throw posix_error("initialize: Interrupt failed.");

  imu_d.imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

  mpu9250_imu_read(&imu_d);

  return 0;
}

void mpu9250_free(void)
{
  gpioSetISRFuncEx(mpu9250_interrupt_pin, RISING_EDGE, 0, NULL, NULL);
}
