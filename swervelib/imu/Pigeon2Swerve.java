package swervelib.imu;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * SwerveIMU interface for the {@link Pigeon2}
 */
public class Pigeon2Swerve extends SwerveIMU
{

  /**
   * Wait time for status frames to show up.
   */
  public static double              STATUS_TIMEOUT_SECONDS = 0.04;
  /**
   * {@link Pigeon2} IMU device.
   */
  private final Pigeon2             imu;
  /**
   * Offset for the {@link Pigeon2}.
   */
  private       Rotation3d          offset                 = new Rotation3d();
  /**
   * Inversion for the gyro
   */
  private       boolean             invertedIMU            = false;
  /**
   * {@link Pigeon2} configurator.
   */
  private       Pigeon2Configurator cfg;

  /**
   * X Acceleration supplier
   */
  private Supplier<StatusSignal<LinearAcceleration>> xAcc;
  /**
   * Y Accelleration supplier.
   */
  private Supplier<StatusSignal<LinearAcceleration>> yAcc;
  /**
   * Z Acceleration supplier.
   */
  private Supplier<StatusSignal<LinearAcceleration>> zAcc;

  /**
   * Generate the SwerveIMU for {@link Pigeon2}.
   *
   * @param canid  CAN ID for the {@link Pigeon2}
   * @param canbus CAN Bus name the {@link Pigeon2} resides on.
   */
  public Pigeon2Swerve(int canid, String canbus)
  {
    imu = new Pigeon2(canid, canbus);
    this.cfg = imu.getConfigurator();
    xAcc = imu::getAccelerationX;
    yAcc = imu::getAccelerationY;
    zAcc = imu::getAccelerationZ;
    SmartDashboard.putData(imu);
  }

  /**
   * Generate the SwerveIMU for {@link Pigeon2}.
   *
   * @param canid CAN ID for the {@link Pigeon2}
   */
  public Pigeon2Swerve(int canid)
  {
    this(canid, "");
  }

  /**
   * Reset {@link Pigeon2} to factory default.
   */
  @Override
  public void factoryDefault()
  {
    Pigeon2Configuration config = new Pigeon2Configuration();

    // Compass utilization causes readings to jump dramatically in some cases.
    cfg.apply(config.Pigeon2Features.withEnableCompass(false));
  }

  /**
   * Clear sticky faults on {@link Pigeon2}.
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
    // TODO: Implement later.

    return Optional.empty();
  }

  /**
   * Fetch the rotation rate from the IMU in degrees per second. If rotation rate isn't supported returns empty.
   *
   * @return Rotation rate in DegreesPerSecond.
   */
  public double getRate()
  {
    return imu.getAngularVelocityZWorld().waitForUpdate(STATUS_TIMEOUT_SECONDS).getValue().in(DegreesPerSecond);
  }

  /**
   * Get the instantiated {@link Pigeon2} object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return imu;
  }
}
