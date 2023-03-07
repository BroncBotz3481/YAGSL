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
   * Set the yaw in degrees.
   *
   * @param yaw Yaw angle in degrees.
   */
  public abstract void setYaw(double yaw);

  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  public abstract void getYawPitchRoll(double[] yprArray);

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
