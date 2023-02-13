package swervelib.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.motors.TalonSRXSwerve.CTRE_remoteSensor;
import swervelib.motors.TalonSRXSwerve.CTRE_slotIdx;
import swervelib.parser.PIDFConfig;

/**
 * {@link com.ctre.phoenix.motorcontrol.can.TalonFX} Swerve Motor.
 */
public class TalonFXSwerve extends SwerveMotor
{

  /**
   * TalonFX motor controller.
   */
  WPI_TalonFX motor;
  /**
   * Factory default already occurred.
   */
  private final boolean factoryDefaultOccurred   = false;
  /**
   * Whether the absolute encoder is integrated.
   */
  private       boolean absoluteEncoder          = false;
  /**
   * The position conversion factor.
   */
  private       double  positionConversionFactor = 1;

  /**
   * Constructor for TalonFX swerve motor.
   *
   * @param motor        Motor to use.
   * @param isDriveMotor Whether this motor is a drive motor.
   */
  public TalonFXSwerve(WPI_TalonFX motor, boolean isDriveMotor)
  {
    this.isDriveMotor = isDriveMotor;
    this.motor = motor;

    factoryDefaults();
    clearStickyFaults();
  }

  /**
   * Construct the TalonFX swerve motor given the ID and CANBus.
   *
   * @param id           ID of the TalonFX on the CANBus.
   * @param canbus       CANBus on which the TalonFX is on.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public TalonFXSwerve(int id, String canbus, boolean isDriveMotor)
  {
    this(new WPI_TalonFX(id, canbus), isDriveMotor);
  }

  /**
   * Construct the TalonFX swerve motor given the ID.
   *
   * @param id           ID of the TalonFX on the canbus.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public TalonFXSwerve(int id, boolean isDriveMotor)
  {
    this(new WPI_TalonFX(id), isDriveMotor);
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
    if (encoder.getAbsoluteEncoder() instanceof CANCoder)
    {
      motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
      motor.configRemoteFeedbackFilter((CANCoder) encoder.getAbsoluteEncoder(),
                                       CTRE_remoteSensor.REMOTE_SENSOR_0.ordinal());
      absoluteEncoder = true;
    }
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
    motor.configSelectedFeedbackCoefficient(positionConversionFactor);
  }

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
    int slotIdx = isDriveMotor ? CTRE_slotIdx.Velocity.ordinal() : CTRE_slotIdx.Turning.ordinal();
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
    motor.set(isDriveMotor ? ControlMode.Velocity : ControlMode.Position, isDriveMotor ? setpoint * .1 : setpoint,
              DemandType.ArbitraryFeedForward,
              feedforward);
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity
   */
  @Override
  public double getVelocity()
  {
    return motor.getSelectedSensorVelocity() * (100 * positionConversionFactor);
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position
   */
  @Override
  public double getPosition()
  {
    return motor.getSelectedSensorPosition() * positionConversionFactor;
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position. Should be angle in degrees or meters per second.
   */
  @Override
  public void setPosition(double position)
  {
    if (!absoluteEncoder)
    {
      motor.setSelectedSensorPosition(position * positionConversionFactor);
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
    motor.configGetSupplyCurrentLimit(config);
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

}
