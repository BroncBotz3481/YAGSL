package swervelib.encoders;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Swerve Absolute Encoder for CTRE CANCoders.
 */
public class CANCoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * CANCoder with WPILib sendable and support.
   */
  public CANcoder encoder;

  /**
   * Initialize the CANCoder on the standard CANBus.
   *
   * @param id CAN ID.
   */
  public CANCoderSwerve(int id)
  {
    encoder = new CANcoder(id);
  }

  /**
   * Initialize the CANCoder on the CANivore.
   *
   * @param id     CAN ID.
   * @param canbus CAN bus to initialize it on.
   */
  public CANCoderSwerve(int id, String canbus)
  {
    encoder = new CANcoder(id, canbus);
  }

  /**
   * Reset the encoder to factory defaults.
   */
  @Override
  public void factoryDefault()
  {
    encoder.getConfigurator().apply(new CANcoderConfiguration());
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
    CANcoderConfigurator cfg                       = encoder.getConfigurator();
    MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
    cfg.refresh(magnetSensorConfiguration);
    cfg.apply(magnetSensorConfiguration
                  .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(inverted ? SensorDirectionValue.Clockwise_Positive
                                                : SensorDirectionValue.CounterClockwise_Positive));
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
    MagnetHealthValue strength = encoder.getMagnetHealth().getValue();

    if (strength != MagnetHealthValue.Magnet_Green)
    {
      DriverStation.reportWarning(
          "CANCoder " + encoder.getDeviceID() + " magnetic field is less than ideal.\n", false);
    }
    if (strength == MagnetHealthValue.Magnet_Invalid || strength == MagnetHealthValue.Magnet_Red)
    {
      readingError = true;
      DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " reading was faulty.\n", false);
      return 0;
    }
    StatusSignal<Double> angle = encoder.getAbsolutePosition().refresh();

    // Taken from democat's library.
    // Source: https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
    for (int i = 0; i < maximumRetries; i++)
    {
      if (angle.getStatus() == StatusCode.OK)
      {
        break;
      }
      angle = angle.waitForUpdate(0.01);
    }
    if (angle.getStatus() != StatusCode.OK)
    {
      readingError = true;
      DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " reading was faulty, ignoring.\n", false);
    }

    return angle.getValue() * 360;
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

  /**
   * Sets the Absolute Encoder Offset within the CANcoder's Memory.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point in degrees.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    CANcoderConfigurator cfg    = encoder.getConfigurator();
    MagnetSensorConfigs  magCfg = new MagnetSensorConfigs();
    StatusCode           error  = cfg.refresh(magCfg);
    if (error != StatusCode.OK)
    {
      return false;
    }
    error = cfg.apply(magCfg.withMagnetOffset(offset / 360));
    if (error == StatusCode.OK)
    {
      return true;
    }
    DriverStation.reportWarning(
        "Failure to set CANCoder " + encoder.getDeviceID() + " Absolute Encoder Offset Error: " + error, false);
    return false;
  }

  /**
   * Get the velocity in degrees/sec.
   *
   * @return velocity in degrees/sec.
   */
  @Override
  public double getVelocity()
  {
    return encoder.getVelocity().getValue() * 360;
  }
}
