package swervelib.imu;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * SwerveIMU interface for the Pigeon2
 */
public class Pigeon2Swerve extends SwerveIMU
{

  /**
   * Pigeon2 IMU device.
   */
  WPI_Pigeon2 imu;
  /**
   * Offset for the Pigeon 2.
   */
  private Rotation3d offset = new Rotation3d();

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid  CAN ID for the pigeon
   * @param canbus CAN Bus name the pigeon resides on.
   */
  public Pigeon2Swerve(int canid, String canbus)
  {
    imu = new WPI_Pigeon2(canid, canbus);
    Pigeon2Configuration config = new Pigeon2Configuration();
    imu.configAllSettings(config);
    SmartDashboard.putData(imu);
  }

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon
   */
  public Pigeon2Swerve(int canid)
  {
    this(canid, "");
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    imu.configFactoryDefault();
    imu.configEnableCompass(false); // Compass utilization causes readings to jump dramatically in some cases.
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
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRawRotation3d()
  {
    double[] wxyz = new double[4];
    imu.get6dQuaternion(wxyz);
    return new Rotation3d(new Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]));
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
