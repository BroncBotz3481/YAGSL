package swervelib.motors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * {@link TalonFXS} Swerve Motor. Made by Team 1466 WebbRobotics.
 */
public class TalonFXSSwerve extends SwerveMotor
{

  /**
   * Wait time for status frames to show up.
   */
  public static double                STATUS_TIMEOUT_SECONDS  = 0.02;
  /**
   * Factory default already occurred.
   */
  private final boolean               factoryDefaultOccurred  = false;
  /**
   * Whether the absolute encoder is integrated.
   */
  private final boolean               absoluteEncoder         = false;
  /**
   * Motion magic angle voltage setter.
   */
  private final MotionMagicVoltage    m_angleVoltageSetter    = new MotionMagicVoltage(0);
  /**
   * Velocity voltage setter for controlling drive motor.
   */
  private final VelocityVoltage       m_velocityVoltageSetter = new VelocityVoltage(0);
  /**
   * TalonFXS motor controller.
   */
  private final TalonFXS              motor;
  /**
   * Conversion factor for the motor.
   */
  private       double                conversionFactor;
  /**
   * Current TalonFXS configuration.
   */
  private       TalonFXSConfiguration configuration           = new TalonFXSConfiguration();
  /**
   * Current TalonFXS Configurator.
   */
  private       TalonFXSConfigurator  cfg;


  /**
   * Constructor for TalonFXS swerve motor.
   *
   * @param motor        Motor to use.
   * @param isDriveMotor Whether this motor is a drive motor.
   * @param motorType    {@link DCMotor} which the {@link TalonFXS} is attached to.
   */
  public TalonFXSSwerve(TalonFXS motor, boolean isDriveMotor, DCMotor motorType)
  {
    this.isDriveMotor = isDriveMotor;
    this.motor = motor;
    this.cfg = motor.getConfigurator();
    this.simMotor = motorType;

    factoryDefaults();
    clearStickyFaults();

  }

  /**
   * Construct the TalonFXS swerve motor given the ID and CANBus.
   *
   * @param id           ID of the TalonFXS on the CANBus.
   * @param canbus       CANBus on which the TalonFXS is on.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   * @param motorType    {@link DCMotor} which the {@link TalonFXS} is attached to.
   */
  public TalonFXSSwerve(int id, String canbus, boolean isDriveMotor, DCMotor motorType)
  {
    this(new TalonFXS(id, canbus), isDriveMotor, motorType);
  }

  /**
   * Construct the TalonFXS swerve motor given the ID.
   *
   * @param id           ID of the TalonFXS on the canbus.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   * @param motorType    {@link DCMotor} which the {@link TalonFXS} is attached to.
   */
  public TalonFXSSwerve(int id, boolean isDriveMotor, DCMotor motorType)
  {
    this(new TalonFXS(id), isDriveMotor, motorType);
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    if (!factoryDefaultOccurred)
    {
      configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      configuration.ClosedLoopGeneral.ContinuousWrap = true;
      cfg.apply(configuration);

      m_angleVoltageSetter.UpdateFreqHz = 0;
      //      m_angleVoltageExpoSetter.UpdateFreqHz = 0;
      m_velocityVoltageSetter.UpdateFreqHz = 0;
      //      motor.configFactoryDefault();
      //      motor.setSensorPhase(true);
      //      motor.configSelectedFeedbackSensor(TalonFXSFeedbackDevice.IntegratedSensor, 0, 30);
      //      motor.configNeutralDeadband(0.001);
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
    // Do not support.
    return this;
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply for position.
   *                                 <p><br>
   *                                 Degrees: <br>
   *                                 <code>
   *                                 360 / (angleGearRatio * encoderTicksPerRotation)
   *                                 </code><br>
   *                                 <p><br>
   *                                 Meters:<br>
   *                                 <code>
   *                                 (Math.PI * wheelDiameter) / (driveGearRatio * encoderTicksPerRotation)
   *                                 </code>
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    cfg.refresh(configuration);

    positionConversionFactor = 1 / positionConversionFactor;
    if (!isDriveMotor)
    {
      positionConversionFactor *= 360;
    }
    conversionFactor = positionConversionFactor;

    configuration.MotionMagic =
        configuration.MotionMagic.withMotionMagicCruiseVelocity(100.0 / positionConversionFactor)
                                 .withMotionMagicAcceleration((100.0 / positionConversionFactor) / 0.100)
                                 .withMotionMagicExpo_kV(0.12 * positionConversionFactor)
                                 .withMotionMagicExpo_kA(0.1);

    /*
    configuration.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                          .withSensorToMechanismRatio(positionConversionFactor);
     */

    cfg.apply(configuration);
  }

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {

    cfg.refresh(configuration.Slot0);
    cfg.apply(
        configuration.Slot0.withKP(config.p).withKI(config.i).withKD(config.d).withKS(config.f));
    //    configuration.slot0.integralZone = config.iz;
    //    configuration.slot0.closedLoopPeakOutput = config.output.max;
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
    cfg.refresh(configuration.ClosedLoopGeneral);
    configuration.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.apply(configuration.ClosedLoopGeneral);
  }

  /**
   * Disable PID Wrapping on the motor.
   */
  @Override
  public void disablePIDWrapping()
  {
    cfg.refresh(configuration.ClosedLoopGeneral);
    configuration.ClosedLoopGeneral.ContinuousWrap = false;
    cfg.apply(configuration.ClosedLoopGeneral);
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    motor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    //    Timer.delay(1);
    cfg.refresh(configuration.MotorOutput);
    configuration.MotorOutput.withInverted(
        inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);
    cfg.apply(configuration.MotorOutput);
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
    setReference(setpoint, feedforward, getPosition());
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in meters per second or angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   * @param position    Only used on the angle motor, the position of the motor in degrees.
   */
  @Override
  public void setReference(double setpoint, double feedforward, double position)
  {
    //    if (SwerveDriveTelemetry.isSimulation)
    //    {
    //      PhysicsSim.getInstance().run();
    //    }

    if (isDriveMotor)
    {
      motor.setControl(m_velocityVoltageSetter.withVelocity(setpoint).withFeedForward(feedforward));
    } else
    {
      motor.setControl(m_angleVoltageSetter.withPosition(setpoint / 360.0));
    }
  }

  /**
   * Get the voltage output of the motor controller.
   *
   * @return Voltage output.
   */
  @Override
  public double getVoltage()
  {
    return motor.getMotorVoltage().waitForUpdate(STATUS_TIMEOUT_SECONDS).getValue().in(Volts);
  }

  /**
   * Set the voltage of the motor.
   *
   * @param voltage Voltage to set.
   */
  @Override
  public void setVoltage(double voltage)
  {
    motor.setVoltage(voltage);
  }

  /**
   * Get the applied dutycycle output.
   *
   * @return Applied dutycycle output to the motor.
   */
  @Override
  public double getAppliedOutput()
  {
    return motor.getDutyCycle().waitForUpdate(STATUS_TIMEOUT_SECONDS).getValue();
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity in Meters Per Second, or Degrees per Second.
   */
  @Override
  public double getVelocity()
  {
    return motor.getVelocity().getValue().magnitude();
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position in Meters or Degrees.
   */
  @Override
  public double getPosition()
  {
    return motor.getPosition().getValue().magnitude();
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position. Should be angle in degrees or meters.
   */
  @Override
  public void setPosition(double position)
  {
    if (!absoluteEncoder && !SwerveDriveTelemetry.isSimulation)
    {
      cfg.setPosition(Degrees.of(position).in(Rotations));
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
    // Do not implement
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
    cfg.refresh(configuration.CurrentLimits);
    cfg.apply(
        configuration.CurrentLimits.withSupplyCurrentLimit(currentLimit)
                                   .withSupplyCurrentLimitEnable(true));
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    cfg.refresh(configuration.ClosedLoopRamps);
    cfg.apply(configuration.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(rampRate));
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
   * Get the {@link DCMotor} of the motor class.
   *
   * @return {@link DCMotor} of this type.
   */
  @Override
  public DCMotor getSimMotor()
  {
    if (simMotor == null)
    {
      simMotor = DCMotor.getKrakenX60(1);
    }
    return simMotor;
  }

  /**
   * Queries whether the absolute encoder is directly attached to the motor controller.
   *
   * @return connected absolute encoder state.
   */
  @Override
  public boolean usingExternalFeedbackSensor()
  {
    return absoluteEncoder;
  }

  /**
   * Closes handles for unit testing.
   */
  @Override
  public void close()
  {
    motor.close();
  }
}
