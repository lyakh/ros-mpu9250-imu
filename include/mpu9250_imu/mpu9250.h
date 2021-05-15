#ifndef MPU9250_H
#define MPU9250_H

#define PUBLISH_FROM_INTERRUPT

class MPU9250_Acc_Gyro;
class AK8963_Magnetometer;

struct mpu9250 {
  MPU9250_Acc_Gyro *acc_gyro;
  AK8963_Magnetometer *mag;
  ros::Publisher imu_pub;
  unsigned int count = 0;
};

void mpu9250_imu_read(struct mpu9250 *imu_d);
int mpu9250_init(ros::NodeHandle &nh, struct mpu9250 &imu_d);
void mpu9250_free(void);

#endif
