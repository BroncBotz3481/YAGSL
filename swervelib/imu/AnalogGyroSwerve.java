package swervelib.imu;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Creates a IMU for {@link edu.wpi.first.wpilibj.AnalogGyro} devices, only uses yaw.
 */
public class AnalogGyroSwerve extends SwerveIMU
{

  /**
   * Gyroscope object.
   */
  private final AnalogGyro gyro;
  /**
   * The yaw offset for the gyroscope.
   */
  private       double     yawOffset;

  /**
   * Analog port in which the gyroscope is connected. Can only be attached to analog ports 0 or 1.
   *
   * @param channel Analog port 0 or 1.
   */
  public AnalogGyroSwerve(int channel)
  {
    if (!(channel == 0 || channel == 1))
    {
      throw new RuntimeException(
          "Analog Gyroscope must be attached to port 0 or 1 on the roboRIO.\n");
    }
    gyro = new AnalogGyro(channel);
    SmartDashboard.putData(gyro);
    factoryDefault();
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    yawOffset = Math.IEEEremainder(gyro.getAngle(), 360);
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
    yawOffset = Math.IEEEremainder(yaw, 360) + Math.IEEEremainder(gyro.getAngle(), 360);
  }

  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  @Override
  public void getYawPitchRoll(double[] yprArray)
  {
    yprArray[0] = Math.IEEEremainder(gyro.getAngle(), 360) - yawOffset;
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
    return gyro;
  }
}
