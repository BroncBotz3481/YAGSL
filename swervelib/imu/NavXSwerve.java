package swervelib.imu;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Communicates with the NavX as the IMU.
 */
public class NavXSwerve extends SwerveIMU
{

  /**
   * NavX IMU.
   */
  private AHRS   gyro;
  /**
   * Offset for the NavX yaw reading.
   */
  private double yawOffset = 0;

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
      SmartDashboard.putData(gyro);
    } catch (RuntimeException ex)
    {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    // gyro.reset(); // Reported to be slow
    yawOffset = Math.IEEEremainder(gyro.getYaw(), 360);
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
  }

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Yaw angle in degrees.
   */
  @Override
  public void setYaw(double yaw)
  {
    // gyro.reset(); // Reported to be slow using the offset.
    yawOffset = Math.IEEEremainder(yaw, 360) + Math.IEEEremainder(gyro.getYaw(), 360);
  }

  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  @Override
  public void getYawPitchRoll(double[] yprArray)
  {

    yprArray[0] = (Math.IEEEremainder(gyro.getYaw(), 360)) - yawOffset;
    yprArray[1] = Math.IEEEremainder(gyro.getPitch(), 360);
    yprArray[2] = Math.IEEEremainder(gyro.getRoll(), 360);
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
