package swervelib.imu;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * IMU Swerve class for the {@link ADIS16448_IMU} device.
 */
public class ADIS16448Swerve extends SwerveIMU
{

  /**
   * {@link ADIS16448_IMU} device to read the current headings from.
   */
  private final ADIS16448_IMU imu;
  /**
   * Offset for the ADIS16448.
   */
  private       Rotation3d    offset = new Rotation3d();

  /**
   * Construct the ADIS16448 imu and reset default configurations. Publish the gyro to the SmartDashboard.
   */
  public ADIS16448Swerve()
  {
    imu = new ADIS16448_IMU();
    factoryDefault();
    SmartDashboard.putData(imu);
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    offset = new Rotation3d(0, 0, 0);
    imu.calibrate();
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
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  public Rotation3d getRawRotation3d()
  {
    return new Rotation3d(Math.toRadians(-imu.getGyroAngleX()),
                          Math.toRadians(-imu.getGyroAngleY()),
                          Math.toRadians(-imu.getGyroAngleZ()));
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
   * @return {@link Translation3d} of the acceleration.
   */
  @Override
  public Optional<Translation3d> getAccel()
  {
    return Optional.of(new Translation3d(imu.getAccelX(), imu.getAccelY(), imu.getAccelZ()));
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
