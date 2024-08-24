package swervelib.imu;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
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
   * Wait time for status frames to show up.
   */
  public static double STATUS_TIMEOUT_SECONDS = 0.04;
  /**
   * Pigeon2 IMU device.
   */
  Pigeon2 imu;
  /**
   * Offset for the Pigeon 2.
   */
  private Rotation3d offset      = new Rotation3d();
  /**
   * Inversion for the gyro
   */
  private boolean    invertedIMU = false;

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid  CAN ID for the pigeon
   * @param canbus CAN Bus name the pigeon resides on.
   */
  public Pigeon2Swerve(int canid, String canbus)
  {
    imu = new Pigeon2(canid, canbus);
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
    Pigeon2Configurator  cfg    = imu.getConfigurator();
    Pigeon2Configuration config = new Pigeon2Configuration();

    // Compass utilization causes readings to jump dramatically in some cases.
    cfg.apply(config.Pigeon2Features.withEnableCompass(false));
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
    Rotation3d reading = imu.getRotation3d();
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
    // TODO: Switch to suppliers.
    StatusSignal<Double> xAcc = imu.getAccelerationX();
    StatusSignal<Double> yAcc = imu.getAccelerationY();
    StatusSignal<Double> zAcc = imu.getAccelerationZ();

    return Optional.of(new Translation3d(xAcc.waitForUpdate(STATUS_TIMEOUT_SECONDS).getValue(),
                                         yAcc.waitForUpdate(STATUS_TIMEOUT_SECONDS).getValue(),
                                         zAcc.waitForUpdate(STATUS_TIMEOUT_SECONDS).getValue()).times(9.81 / 16384.0));
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
