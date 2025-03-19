package swervelib.imu;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * SwerveIMU interface for the {@link WPI_PigeonIMU}.
 */
public class PigeonViaTalonSRXSwerve extends SwerveIMU
{


  /**
   * {@link TalonSRX} TalonSRX the IMU is attached to.
   */
  private final WPI_TalonSRX talon;

  /**
   * {@link WPI_PigeonIMU} IMU device.
   */
  private final WPI_PigeonIMU      imu;
  /**
   * Mutable {@link AngularVelocity} for readings.
   */
  private final MutAngularVelocity yawVel      = new MutAngularVelocity(0, 0, DegreesPerSecond);
  /**
   * Offset for the {@link WPI_PigeonIMU}.
   */
  private       Rotation3d         offset      = new Rotation3d();
  /**
   * Inversion for the gyro
   */
  private       boolean            invertedIMU = false;

  /**
   * Generate the SwerveIMU for {@link WPI_PigeonIMU} attached to a {@link TalonSRX}.
   *
   * @param canid CAN ID for the {@link TalonSRX} the {@link WPI_PigeonIMU} is attached to, does not support CANBus.
   */
  public PigeonViaTalonSRXSwerve(int canid)
  {
    talon = new WPI_TalonSRX(canid);
    imu = new WPI_PigeonIMU(talon);
    offset = new Rotation3d();
    SmartDashboard.putData(imu);
  }

  @Override
  public void close() {
    imu.close();
    talon.close();
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
    return getRawRotation3d().rotateBy(offset.unaryMinus());
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

  @Override
  public MutAngularVelocity getYawAngularVelocity()
  {
    return yawVel.mut_setMagnitude(imu.getRate());
  }

  /**
   * Get the instantiated {@link WPI_PigeonIMU} IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return imu;
  }

}
