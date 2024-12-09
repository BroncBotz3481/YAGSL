package swervelib.encoders;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import swervelib.motors.SwerveMotor;
import swervelib.motors.TalonSRXSwerve;

/**
 * Talon SRX attached absolute encoder.
 */
public class TalonSRXEncoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * Multiplying by this converts native Talon SRX units into degrees.
   */
  private final double       degreesPerSensorUnit;
  /**
   * Reference to a Talon SRX for polling its attached absolute encoder.
   */
  private final WPI_TalonSRX talon;

  /**
   * Creates a {@link TalonSRXEncoderSwerve}.
   *
   * @param motor          motor to poll the sensor from.
   * @param feedbackDevice the feedback device the sensor uses e.g. PWM or Analog.
   */
  public TalonSRXEncoderSwerve(SwerveMotor motor, FeedbackDevice feedbackDevice)
  {
    if (motor instanceof TalonSRXSwerve talonSRXSwerve)
    {
      talonSRXSwerve.setSelectedFeedbackDevice(feedbackDevice);
      this.talon = (WPI_TalonSRX) talonSRXSwerve.getMotor();
      // https://v5.docs.ctr-electronics.com/en/stable/ch14_MCSensor.html#sensor-resolution
      degreesPerSensorUnit = switch (feedbackDevice)
      {
        case Analog -> 360.0 / 1024.0;
        default -> 360.0 / 4096.0;
      };
    } else
    {
      throw new RuntimeException("Motor given to instantiate TalonSRXEncoder is not a WPI_TalonSRX");
    }
  }

  @Override
  public void factoryDefault()
  {
    // Handled in TalonSRXSwerve
  }

  @Override
  public void clearStickyFaults()
  {
    // Handled in TalonSRXSwerve
  }

  @Override
  public void configure(boolean inverted)
  {
    talon.setSensorPhase(inverted);
  }

  @Override
  public double getAbsolutePosition()
  {
    return (talon.getSelectedSensorPosition() * degreesPerSensorUnit) % 360;
  }

  @Override
  public Object getAbsoluteEncoder()
  {
    return talon;
  }

  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    talon.setSelectedSensorPosition(talon.getSelectedSensorPosition() + offset / degreesPerSensorUnit);
    return true;
  }

  @Override
  public double getVelocity()
  {
    return talon.getSelectedSensorVelocity() * 10 * degreesPerSensorUnit;
  }

}
