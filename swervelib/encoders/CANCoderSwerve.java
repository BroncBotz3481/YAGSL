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
import swervelib.telemetry.Alert;

/**
 * Swerve Absolute Encoder for CTRE CANCoders.
 */
public class CANCoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * Wait time for status frames to show up.
   */
  public static double   STATUS_TIMEOUT_SECONDS = 0.02;
  /**
   * CANCoder with WPILib sendable and support.
   */
  public        CANcoder encoder;
  /**
   * An {@link Alert} for if the CANCoder magnet field is less than ideal.
   */
  private       Alert    magnetFieldLessThanIdeal;
  /**
   * An {@link Alert} for if the CANCoder reading is faulty.
   */
  private       Alert    readingFaulty;
  /**
   * An {@link Alert} for if the CANCoder reading is faulty and the reading is ignored.
   */
  private       Alert    readingIgnored;
  /**
   * An {@link Alert} for if the absolute encoder offset cannot be set.
   */
  private       Alert    cannotSetOffset;

  /**
   * Initialize the CANCoder on the standard CANBus.
   *
   * @param id CAN ID.
   */
  public CANCoderSwerve(int id)
  {
    // Empty string uses the default canbus for the system
    this(id, "");
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
    magnetFieldLessThanIdeal = new Alert(
        "Encoders",
        "CANCoder " + encoder.getDeviceID() + " magnetic field is less than ideal.",
        Alert.AlertType.WARNING);
    readingFaulty = new Alert(
        "Encoders",
        "CANCoder " + encoder.getDeviceID() + " reading was faulty.",
        Alert.AlertType.WARNING);
    readingIgnored = new Alert(
        "Encoders",
        "CANCoder " + encoder.getDeviceID() + " reading was faulty, ignoring.",
        Alert.AlertType.WARNING);
    cannotSetOffset = new Alert(
        "Encoders",
        "Failure to set CANCoder "
        + encoder.getDeviceID()
        + " Absolute Encoder Offset",
        Alert.AlertType.WARNING);
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

    magnetFieldLessThanIdeal.set(strength != MagnetHealthValue.Magnet_Green);
    if (strength == MagnetHealthValue.Magnet_Invalid || strength == MagnetHealthValue.Magnet_Red)
    {
      readingError = true;
      readingFaulty.set(true);
      return 0;
    } else
    {
      readingFaulty.set(false);
    }

    StatusSignal<Double> angle = encoder.getAbsolutePosition();

    // Taken from democat's library.
    // Source: https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
    for (int i = 0; i < maximumRetries; i++)
    {
      if (angle.getStatus() == StatusCode.OK)
      {
        break;
      }
      angle = angle.waitForUpdate(STATUS_TIMEOUT_SECONDS);
    }
    if (angle.getStatus() != StatusCode.OK)
    {
      readingError = true;
      readingIgnored.set(true);
    } else
    {
      readingIgnored.set(false);
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
    cannotSetOffset.setText(
        "Failure to set CANCoder "
        + encoder.getDeviceID()
        + " Absolute Encoder Offset Error: "
        + error);
    if (error == StatusCode.OK)
    {
      cannotSetOffset.set(false);
      return true;
    }
    cannotSetOffset.set(true);
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
