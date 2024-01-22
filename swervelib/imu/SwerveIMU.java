package swervelib.imu;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

/**
 * Swerve IMU abstraction to define a standard interface with a swerve drive.
 */
public abstract class SwerveIMU
{

  /**
   * Reset IMU to factory default.
   */
  public abstract void factoryDefault();

  /**
   * Clear sticky faults on IMU.
   */
  public abstract void clearStickyFaults();

  /**
   * Set the gyro offset.
   *
   * @param offset gyro offset as a {@link Rotation3d}.
   */
  public abstract void setOffset(Rotation3d offset);

  /**
   * Set the gyro to invert its default direction.
   *
   * @param invertIMU gyro direction
   */
  public abstract void setInverted(boolean invertIMU);

  /**
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  public abstract Rotation3d getRawRotation3d();

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  public abstract Rotation3d getRotation3d();

  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
   * empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  public abstract Optional<Translation3d> getAccel();

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  public abstract Object getIMU();
}
