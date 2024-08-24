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
  private final AnalogGyro imu;
  /**
   * Offset for the analog gyro.
   */
  private       Rotation3d offset      = new Rotation3d();
  /**
   * Inversion for the gyro
   */
  private       boolean    invertedIMU = false;

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
    imu = new AnalogGyro(channel);
    factoryDefault();
    SmartDashboard.putData(imu);
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    imu.calibrate();
    offset = new Rotation3d(0, 0, 0);
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
   * Set the gyro offset.
   *
   * @param offset gyro offset as a {@link Rotation3d}.
   */
  public void setOffset(Rotation3d offset)
  {
    this.offset = offset;
  }

  /**
   * Set the gyro to invert its default direction
   *
   * @param invertIMU invert gyro direction
   */
  public void setInverted(boolean invertIMU)
  {
    invertedIMU = invertIMU;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  public Rotation3d getRawRotation3d()
  {
    Rotation3d reading = new Rotation3d(0, 0, Math.toRadians(-imu.getAngle()));
    return invertedIMU ? reading.unaryMinus() : reading;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRotation3d()
  {
    return getRawRotation3d().minus(offset);
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
   * Fetch the rotation rate from the IMU in degrees per second. If rotation rate isn't supported returns empty.
   *
   * @return {@link Double} of the rotation rate as an {@link Optional}.
   */
  public double getRate()
  {
    return imu.getRate();
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
