package swervelib.imu;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

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
    yawOffset = gyro.getAngle() % 360;
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
    yawOffset = (yaw % 360) + (gyro.getAngle() % 360);
  }

  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  @Override
  public void getYawPitchRoll(double[] yprArray)
  {
    yprArray[0] = (gyro.getAngle() % 360) - yawOffset;
    yprArray[1] = 0;
    yprArray[2] = 0;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  public Rotation3d getRotation3d()
  {
    return new Rotation3d(0, 0, gyro.getAngle())
        .minus(new Rotation3d(0, 0, Math.toRadians(yawOffset)));
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
   * empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  @Override
  public Optional<Translation3d> getAccel()
  {
    return Optional.empty();
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
