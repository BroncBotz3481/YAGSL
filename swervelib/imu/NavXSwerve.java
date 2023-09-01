package swervelib.imu;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * Communicates with the NavX as the IMU.
 */
public class NavXSwerve extends SwerveIMU
{

  /**
   * NavX IMU.
   */
  private AHRS       gyro;
  /**
   * Offset for the NavX.
   */
  private Rotation3d offset = new Rotation3d();

  /**
   * Constructor for the NavX swerve.
   *
   * @param port Serial Port to connect to.
   */
  public NavXSwerve(SerialPort.Port port)
  {
    try
    {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      gyro = new AHRS(port);
      factoryDefault();
      SmartDashboard.putData(gyro);
    } catch (RuntimeException ex)
    {
      DriverStation.reportError("Error instantiating navX:  " + ex.getMessage(), true);
    }
  }

  /**
   * Constructor for the NavX swerve.
   *
   * @param port SPI Port to connect to.
   */
  public NavXSwerve(SPI.Port port)
  {
    try
    {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      gyro = new AHRS(port);
      factoryDefault();
      SmartDashboard.putData(gyro);
    } catch (RuntimeException ex)
    {
      DriverStation.reportError("Error instantiating navX:  " + ex.getMessage(), true);
    }
  }

  /**
   * Constructor for the NavX swerve.
   *
   * @param port I2C Port to connect to.
   */
  public NavXSwerve(I2C.Port port)
  {
    try
    {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      gyro = new AHRS(port);
      factoryDefault();
      SmartDashboard.putData(gyro);
    } catch (RuntimeException ex)
    {
      DriverStation.reportError("Error instantiating navX:  " + ex.getMessage(), true);
    }
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    // gyro.reset(); // Reported to be slow
    offset = new Rotation3d(new Quaternion(gyro.getQuaternionW(),
                                           gyro.getQuaternionX(),
                                           gyro.getQuaternionY(),
                                           gyro.getQuaternionZ()));
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
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
    return new Rotation3d(new Quaternion(gyro.getQuaternionW(),
                                         gyro.getQuaternionX(),
                                         gyro.getQuaternionY(),
                                         gyro.getQuaternionZ()));
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
    return Optional.of(
        new Translation3d(
            gyro.getWorldLinearAccelX(),
            gyro.getWorldLinearAccelY(),
            gyro.getWorldLinearAccelZ())
            .times(9.81));
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
