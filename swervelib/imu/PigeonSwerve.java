package swervelib.imu;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * SwerveIMU interface for the Pigeon.
 */
public class PigeonSwerve extends SwerveIMU
{

  /**
   * Pigeon v1 IMU device.
   */
  WPI_PigeonIMU imu;
  /**
   * Offset for the Pigeon.
   */
  private Rotation3d offset      = new Rotation3d();
  /**
   * Inversion for the gyro
   */
  private boolean    invertedIMU = false;

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon, does not support CANBus.
   */
  public PigeonSwerve(int canid)
  {
    imu = new WPI_PigeonIMU(canid);
    offset = new Rotation3d();
    SmartDashboard.putData(imu);
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    imu.configFactoryDefault();
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
    imu.clearStickyFaults();
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
  @Override
  public Rotation3d getRawRotation3d()
  {
    double[] wxyz = new double[4];
    imu.get6dQuaternion(wxyz);
    Rotation3d reading = new Rotation3d(new Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]));
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
    short[] initial = new short[3];
    imu.getBiasedAccelerometer(initial);
    return Optional.of(new Translation3d(initial[0], initial[1], initial[2]).times(9.81 / 16384.0));
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
