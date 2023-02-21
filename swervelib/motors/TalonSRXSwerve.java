package swervelib.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;

/**
 * {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX} Swerve Motor.
 */
public class TalonSRXSwerve extends SwerveMotor
{

  /**
   * Factory default already occurred.
   */
  private final boolean factoryDefaultOccurred = false;
  /**
   * Whether the absolute encoder is integrated.
   */
  private final boolean absoluteEncoder        = false;
  /**
   * TalonSRX motor controller.
   */
  WPI_TalonSRX motor;
  /**
   * The position conversion factor.
   */
  private double positionConversionFactor = 1;

  /**
   * Constructor for TalonSRX swerve motor.
   *
   * @param motor        Motor to use.
   * @param isDriveMotor Whether this motor is a drive motor.
   */
  public TalonSRXSwerve(WPI_TalonSRX motor, boolean isDriveMotor)
  {
    this.isDriveMotor = isDriveMotor;
    this.motor = motor;
    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    factoryDefaults();
    clearStickyFaults();
  }

  /**
   * Construct the TalonSRX swerve motor given the ID.
   *
   * @param id           ID of the TalonSRX on the canbus.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public TalonSRXSwerve(int id, boolean isDriveMotor)
  {
    this(new WPI_TalonSRX(id), isDriveMotor);
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    if (!factoryDefaultOccurred)
    {
      motor.configFactoryDefault();
      motor.setSensorPhase(true);
    }
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults()
  {
    motor.clearStickyFaults();
  }

  /**
   * Set the absolute encoder to be a compatible absolute encoder.
   *
   * @param encoder The encoder to use.
   */
  @Override
  public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder)
  {
    return this;
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply for position.
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    this.positionConversionFactor = positionConversionFactor;
  }

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
    int slotIdx = 0;
    motor.config_kP(slotIdx, config.p);
    motor.config_kI(slotIdx, config.i);
    motor.config_kD(slotIdx, config.d);
    motor.config_kF(slotIdx, config.f);
    motor.config_IntegralZone(slotIdx, config.iz);
    motor.configClosedLoopPeakOutput(slotIdx, config.output.max);
  }

  /**
   * Configure the PID wrapping for the position closed loop controller.
   *
   * @param minInput Minimum PID input.
   * @param maxInput Maximum PID input.
   */
  @Override
  public void configurePIDWrapping(double minInput, double maxInput)
  {
    // Do nothing
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    motor.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    motor.setInverted(inverted);
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  @Override
  public void burnFlash()
  {
    // Do nothing
  }

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  @Override
  public void set(double percentOutput)
  {
    motor.set(percentOutput);
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in MPS or Angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   */
  @Override
  public void setReference(double setpoint, double feedforward)
  {
    burnFlash();

    motor.set(
        isDriveMotor ? ControlMode.Velocity : ControlMode.Position,
        convertToNativeSensorUnits(setpoint),
        DemandType.ArbitraryFeedForward,
        feedforward * -0.3);
    // Credit to Team 3181 for the -0.3, I'm not sure why it works, but it does.
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity
   */
  @Override
  public double getVelocity()
  {
    return (motor.getSelectedSensorVelocity() * 10) * positionConversionFactor;
  }

  /**
   * Get the raw position.
   *
   * @return Position in meters or degrees.
   */
  public double getRawPosition()
  {
    return motor.getSelectedSensorPosition() * positionConversionFactor;
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position in Meters or Degrees.
   */
  @Override
  public double getPosition()
  {
    return isDriveMotor ? getRawPosition() : getRawPosition() % 360;
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position. Should be angle in degrees or meters.
   */
  @Override
  public void setPosition(double position)
  {
    if (!absoluteEncoder && !RobotBase.isSimulation())
    {
      motor.setSelectedSensorPosition(convertToNativeSensorUnits(position));
    }
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
    motor.enableVoltageCompensation(true);
    motor.configVoltageCompSaturation(nominalVoltage);
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  @Override
  public void setCurrentLimit(int currentLimit)
  {
    SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
    motor.configSupplyCurrentLimit(config);
    config.currentLimit = currentLimit;
    config.enable = true;
    motor.configSupplyCurrentLimit(config);
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    motor.configClosedloopRamp(rampRate);
    motor.configOpenloopRamp(rampRate);
  }

  /**
   * Get the motor object from the module.
   *
   * @return Motor object.
   */
  @Override
  public Object getMotor()
  {
    return motor;
  }

  /**
   * Queries whether the absolute encoder is directly attached to the motor controller.
   *
   * @return connected absolute encoder state.
   */
  @Override
  public boolean isAttachedAbsoluteEncoder()
  {
    return absoluteEncoder;
  }

  /**
   * Put an angle within the the 360 deg scope of a reference. For example, given a scope reference of 756 degrees,
   * assumes the full scope is (720-1080), and places an angle of 22 degrees into it, returning 742 deg.
   *
   * @param scopeReference Current Angle (deg)
   * @param newAngle       Target Angle (deg)
   * @return Closest angle within scope (deg)
   */
  private double placeInAppropriate0To360Scope(double scopeReference, double newAngle)
  {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;

    // Create the interval from the reference angle.
    if (lowerOffset >= 0)
    {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else
    {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    // Put the angle in the interval.
    while (newAngle < lowerBound)
    {
      newAngle += 360;
    }
    while (newAngle > upperBound)
    {
      newAngle -= 360;
    }
    // Smooth the transition between interval boundaries.
    if (newAngle - scopeReference > 180)
    {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180)
    {
      newAngle += 360;
    }
    return newAngle;
  }

  /**
   * Convert the setpoint into native sensor units.
   *
   * @param setpoint Setpoint to mutate. In meters per second or degrees.
   * @return Setpoint as native sensor units. Encoder ticks per 100ms, or Encoder tick.
   */
  public double convertToNativeSensorUnits(double setpoint)
  {
    setpoint =
        isDriveMotor ? setpoint * .1 : placeInAppropriate0To360Scope(getRawPosition(), setpoint);
    return setpoint / positionConversionFactor;
  }
}
