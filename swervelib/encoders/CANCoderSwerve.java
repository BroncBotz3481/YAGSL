package swervelib.encoders;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Swerve Absolute Encoder for CTRE CANCoders.
 */
public class CANCoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * CANCoder with WPILib sendable and support.
   */
  public WPI_CANCoder encoder;

  /**
   * Initialize the CANCoder on the standard CANBus.
   *
   * @param id CAN ID.
   */
  public CANCoderSwerve(int id)
  {
    encoder = new WPI_CANCoder(id);
  }

  /**
   * Initialize the CANCoder on the CANivore.
   *
   * @param id     CAN ID.
   * @param canbus CAN bus to initialize it on.
   */
  public CANCoderSwerve(int id, String canbus)
  {
    encoder = new WPI_CANCoder(id, canbus);
  }

  /**
   * Reset the encoder to factory defaults.
   */
  @Override
  public void factoryDefault()
  {
    encoder.configFactoryDefault();
  }

  /**
   * Clear sticky faults on the encoder.
   */
  @Override
  public void clearStickyFaults()
  {
    encoder.clearStickyFaults();
  }

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted)
  {
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.sensorDirection = inverted;
    canCoderConfiguration.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
    encoder.configAllSettings(canCoderConfiguration);
  }

  /**
   * Get the absolute position of the encoder. Sets {@link SwerveAbsoluteEncoder#readingError} on erroneous readings.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition()
  {
    readingError = false;
    MagnetFieldStrength strength = encoder.getMagnetFieldStrength();

    if (strength != MagnetFieldStrength.Good_GreenLED)
    {
      DriverStation.reportWarning(
          "CANCoder " + encoder.getDeviceID() + " magnetic field is less than ideal.\n", false);
    }
    if (strength == MagnetFieldStrength.Invalid_Unknown || strength == MagnetFieldStrength.BadRange_RedLED)
    {
      readingError = true;
      DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " reading was faulty.\n", false);
      return 0;
    }
    double angle = encoder.getAbsolutePosition();

    // Taken from democat's library.
    // Source: https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
    ErrorCode code = encoder.getLastError();
    for (int i = 0; i < maximumRetries; i++)
    {
      if (code == ErrorCode.OK)
      {
        break;
      }
      try
      {
        Thread.sleep(10);
      } catch (InterruptedException e)
      {
      }
      angle = encoder.getAbsolutePosition();
      code = encoder.getLastError();
    }
    if (code != ErrorCode.OK)
    {
      readingError = true;
      DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " reading was faulty, ignoring.\n", false);
    }

    return angle;
  }

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  @Override
  public Object getAbsoluteEncoder()
  {
    return encoder;
  }
}
