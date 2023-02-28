package swervelib.imu;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * IMU Swerve class for the {@link ADXRS450_Gyro} device.
 */
public class ADXRS450Swerve extends SwerveIMU
{

  /**
   * {@link ADXRS450_Gyro} device to read the current headings from.
   */
  private final ADXRS450_Gyro imu;
  /**
   * Offset for the ADXRS450 yaw reading.
   */
  private       double        yawOffset = 0;

  /**
   * Construct the ADXRS450 imu and reset default configurations. Publish the gyro to the SmartDashboard.
   */
  public ADXRS450Swerve()
  {
    imu = new ADXRS450_Gyro();
    factoryDefault();
    SmartDashboard.putData(imu);
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    yawOffset = imu.getAngle() % 360;
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
    // Do nothing.
  }

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Yaw angle in degrees.
   */
  @Override
  public void setYaw(double yaw)
  {
    yawOffset = (yaw % 360) + (imu.getAngle() % 360);
  }

  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  @Override
  public void getYawPitchRoll(double[] yprArray)
  {
    yprArray[0] = (imu.getAngle() % 360) - yawOffset;
    yprArray[1] = 0;
    yprArray[2] = 0;
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return imu;
  }
}
