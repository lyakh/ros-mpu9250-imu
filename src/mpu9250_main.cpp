#include <errno.h>

#include <pigpio.h>

#include <ros/ros.h>

#include <mpu9250_imu/mpu9250.h>

int main(int argc, char **argv)
{
  struct mpu9250 imu_d;

  ros::init(argc, argv, "imu_node");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  int ret = gpioInitialise();
  if (ret == PI_INIT_FAILED)
    return EIO;

  ret = mpu9250_init(nh, imu_d);
  if (ret < 0)
    return EIO;

#if !defined(PUBLISH_FROM_INTERRUPT)
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
#else
    ros::spin();
#endif

    mpu9250_free();

    return 0;
}
