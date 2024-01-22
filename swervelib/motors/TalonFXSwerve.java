package swervelib.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * {@link com.ctre.phoenix6.hardware.TalonFX} Swerve Motor. Made by Team 1466 WebbRobotics.
 */
public class TalonFXSwerve extends SwerveMotor
{

  /**
   * Factory default already occurred.
   */
  private final boolean            factoryDefaultOccurred  = false;
  /**
   * Whether the absolute encoder is integrated.
   */
  private final boolean            absoluteEncoder         = false;
  /**
   * Motion magic angle voltage setter.
   */
  private final MotionMagicVoltage m_angleVoltageSetter    = new MotionMagicVoltage(0);
  /**
   * Velocity voltage setter for controlling drive motor.
   */
  private final VelocityVoltage    m_velocityVoltageSetter = new VelocityVoltage(0);
  /**
   * TalonFX motor controller.
   */
  TalonFX motor;
  /**
   * Conversion factor for the motor.
   */
  private       double             conversionFactor;
  /**
   * Current TalonFX configuration.
   */
  private TalonFXConfiguration configuration = new TalonFXConfiguration();

  /**
   * Constructor for TalonFX swerve motor.
   *
   * @param motor        Motor to use.
   * @param isDriveMotor Whether this motor is a drive motor.
   */
  public TalonFXSwerve(TalonFX motor, boolean isDriveMotor)
  {
    this.isDriveMotor = isDriveMotor;
    this.motor = motor;

    factoryDefaults();
    clearStickyFaults();

    //    if (SwerveDriveTelemetry.isSimulation)
    //    {
    ////      PhysicsSim.getInstance().addTalonFX(motor, .25, 6800);
    //    }
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
    this(new TalonFX(id, canbus), isDriveMotor);
  }

  /**
   * Construct the TalonFX swerve motor given the ID.
   *
   * @param id           ID of the TalonFX on the canbus.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public TalonFXSwerve(int id, boolean isDriveMotor)
  {
    this(new TalonFX(id), isDriveMotor);
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    if (!factoryDefaultOccurred)
    {
      TalonFXConfigurator cfg = motor.getConfigurator();
      configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      configuration.ClosedLoopGeneral.ContinuousWrap = true;
      cfg.apply(configuration);

      m_angleVoltageSetter.UpdateFreqHz = 0;
      //      m_angleVoltageExpoSetter.UpdateFreqHz = 0;
      m_velocityVoltageSetter.UpdateFreqHz = 0;
      //      motor.configFactoryDefault();
      //      motor.setSensorPhase(true);
      //      motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
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
    TalonFXConfigurator cfg = motor.getConfigurator();
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

    configuration.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                          .withSensorToMechanismRatio(positionConversionFactor);

    cfg.apply(configuration);
    // Taken from democat's library.
    // https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500DriveControllerFactoryBuilder.java#L16
    configureCANStatusFrames(250);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1 Applied Motor Output, Fault Information, Limit Switch Information
   */
  public void configureCANStatusFrames(int CANStatus1)
  {
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1       Applied Motor Output, Fault Information, Limit Switch Information
   * @param CANStatus2       Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), Brushed Supply Current
   *                         Measurement, Sticky Fault Information
   * @param CANStatus3       Quadrature Information
   * @param CANStatus4       Analog Input, Supply Battery Voltage, Controller Temperature
   * @param CANStatus8       Pulse Width Information
   * @param CANStatus10      Motion Profiling/Motion Magic Information
   * @param CANStatus12      Selected Sensor Position (Aux PID 1), Selected Sensor Velocity (Aux PID 1)
   * @param CANStatus13      PID0 (Primary PID) Information
   * @param CANStatus14      PID1 (Auxiliary PID) Information
   * @param CANStatus21      Integrated Sensor Position (Talon FX), Integrated Sensor Velocity (Talon FX)
   * @param CANStatusCurrent Brushless Supply Current Measurement, Brushless Stator Current Measurement
   */
  public void configureCANStatusFrames(
      int CANStatus1,
      int CANStatus2,
      int CANStatus3,
      int CANStatus4,
      int CANStatus8,
      int CANStatus10,
      int CANStatus12,
      int CANStatus13,
      int CANStatus14,
      int CANStatus21,
      int CANStatusCurrent)
  {
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, CANStatus2);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, CANStatus3);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, CANStatus4);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, CANStatus8);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, CANStatus10);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, CANStatus12);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, CANStatus13);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, CANStatus14);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, CANStatus21);
    //    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current,
    // CANStatusCurrent);

    // TODO: Configure Status Frame 2 thru 21 if necessary
    // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html#setting-status-frame-periods
  }

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {

    TalonFXConfigurator cfg = motor.getConfigurator();
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
    TalonFXConfigurator cfg = motor.getConfigurator();
    cfg.refresh(configuration.ClosedLoopGeneral);
    configuration.ClosedLoopGeneral.ContinuousWrap = true;
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
   * Get the velocity of the integrated encoder.
   *
   * @return velocity in Meters Per Second, or Degrees per Second.
   */
  @Override
  public double getVelocity()
  {
    return motor.getVelocity().getValue();
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position in Meters or Degrees.
   */
  @Override
  public double getPosition()
  {
    return motor.getPosition().getValue();
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
      position = position < 0 ? (position % 360) + 360 : position;
      TalonFXConfigurator cfg = motor.getConfigurator();
      cfg.setPosition(position / 360);
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
    TalonFXConfigurator cfg = motor.getConfigurator();
    cfg.refresh(configuration.CurrentLimits);
    cfg.apply(
        configuration.CurrentLimits.withStatorCurrentLimit(currentLimit)
                                   .withStatorCurrentLimitEnable(true));
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    TalonFXConfigurator cfg = motor.getConfigurator();
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
