package swervelib;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.encoders.SparkMaxEncoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.math.SwerveMath;
import swervelib.motors.SparkMaxBrushedMotorSwerve;
import swervelib.motors.SparkMaxSwerve;
import swervelib.motors.SwerveMotor;
import swervelib.parser.Cache;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.simulation.SwerveModuleSimulation;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The Swerve Module class which represents and controls Swerve Modules for the swerve drive.
 */
public class SwerveModule implements AutoCloseable
{

  /**
   * Swerve module configuration options.
   */
  public final  SwerveModuleConfiguration configuration;
  /**
   * Absolute encoder position cache.
   */
  public final  Cache<Double>             absolutePositionCache;
  /**
   * Drive motor position cache.
   */
  public final  Cache<Double>             drivePositionCache;
  /**
   * Drive motor velocity cache.
   */
  public final  Cache<Double>             driveVelocityCache;
  /**
   * Module number for kinematics, usually 0 to 3. front left -> front right -> back left -> back right.
   */
  public final  int                       moduleNumber;
  /**
   * Swerve Motors.
   */
  private final SwerveMotor               angleMotor, driveMotor;
  /**
   * Absolute encoder for swerve drive.
   */
  private final SwerveAbsoluteEncoder  absoluteEncoder;
  /**
   * An {@link Alert} for if pushing the Absolute Encoder offset to the encoder fails.
   */
  private final Alert                  encoderOffsetWarning;
  /**
   * An {@link Alert} for if there is no Absolute Encoder on the module.
   */
  private final Alert                  noEncoderWarning;
  /**
   * An {@link Alert} for if there is no Absolute Encoder on the module.
   */
  private final Alert            externalSensorIsNull         = new Alert("No absolute Encoder found.",
                                                                          AlertType.kError);
  /**
   * An {@link Alert} for if the offset is 0 degrees.
   */
  private final Alert            internalOffsetIsZero         = new Alert(
      "Absolute encoder offset is 0, this may be a problem.",
      AlertType.kWarning);
  /**
   * An {@link Alert} for if the angle/steer/azimuth motor is incompatible with the absolute encoder.
   */
  private final Alert            externalFeedbackIncompatible = new Alert(
      "Absolute encoder is incompatible, cannot set as an external feedback device.",
      AlertType.kError);
  /**
   * An {@link Alert} for if the absolute encoder cannot set an offset.
   */
  private final Alert            externalOffsetIncompatible   = new Alert(
      "Absolute encoder is incompatible, cannot set an offset internally.",
      AlertType.kError);
  /**
   * NT4 Raw Absolute Angle publisher for the absolute encoder.
   */
  private final DoublePublisher  rawAbsoluteAnglePublisher;
  /**
   * NT4 Adjusted Absolute angle publisher for the absolute encoder.
   */
  private final DoublePublisher  adjAbsoluteAnglePublisher;
  /**
   * NT4 Absolute encoder read issue.
   */
  private final BooleanPublisher absoluteEncoderIssuePublisher;
  /**
   * NT4 raw angle motor.
   */
  private final DoublePublisher  rawAnglePublisher;
  /**
   * NT4 Raw drive motor.
   */
  private final DoublePublisher  rawDriveEncoderPublisher;
  /**
   * NT4 Raw drive motor.
   */
  private final DoublePublisher  rawDriveVelocityPublisher;
  /**
   * Speed setpoint publisher for the module motor-controller PID.
   */
  private final DoublePublisher  speedSetpointPublisher;
  /**
   * Angle setpoint publisher for the module motor-controller PID.
   */
  private final DoublePublisher  angleSetpointPublisher;
  /**
   * Maximum {@link LinearVelocity} for the drive motor of the swerve module.
   */
  private       LinearVelocity         maxDriveVelocity;
  /**
   * Maximum velocity for the drive motor of the swerve module.
   */
  private       double           maxDriveVelocityMetersPerSecond;
  /**
   * Maximum {@link AngularVelocity} for the azimuth/angle motor of the swerve module.
   */
  private       AngularVelocity        maxAngularVelocity;
  /**
   * Feedforward for the drive motor during closed loop control.
   */
  private       SimpleMotorFeedforward driveMotorFeedforward;
  /**
   * Anti-Jitter AKA auto-centering disabled.
   */
  private       boolean          antiJitterEnabled            = true;
  /**
   * Last swerve module state applied.
   */
  private       SwerveModuleState      lastState;
  /**
   * Angle offset from the absolute encoder.
   */
  private       double                 angleOffset;
  /**
   * Simulated swerve module.
   */
  private       SwerveModuleSimulation simModule;
  /**
   * Enables utilization off {@link SwerveModuleState#optimize(Rotation2d)}
   */
  private       boolean          optimizeSwerveModuleState    = true;
  /**
   * Encoder synchronization queued.
   */
  private       boolean          synchronizeEncoderQueued     = false;
  /**
   * Encoder, Absolute encoder synchronization enabled.
   */
  private       boolean          synchronizeEncoderEnabled    = false;
  /**
   * Encoder synchronization deadband in degrees.
   */
  private       double           synchronizeEncoderDeadband   = 3;


  /**
   * Construct the swerve module and initialize the swerve module motors and absolute encoder.
   *
   * @param moduleNumber        Module number for kinematics.
   * @param moduleConfiguration Module constants containing CAN ID's and offsets.
   */
  public SwerveModule(int moduleNumber, SwerveModuleConfiguration moduleConfiguration)
  {
    //    angle = 0;
    //    speed = 0;
    //    omega = 0;
    //    fakePos = 0;
    this.moduleNumber = moduleNumber;
    configuration = moduleConfiguration;
    angleOffset = moduleConfiguration.angleOffset;

    // Create motors from configuration and reset them to defaults.
    angleMotor = moduleConfiguration.angleMotor;
    driveMotor = moduleConfiguration.driveMotor;
    angleMotor.factoryDefaults();
    driveMotor.factoryDefaults();

    // Initialize Feedforwards.
    driveMotorFeedforward = getDefaultFeedforward();

    // Configure voltage comp, current limit, and ramp rate.
    angleMotor.setVoltageCompensation(configuration.physicalCharacteristics.optimalVoltage);
    driveMotor.setVoltageCompensation(configuration.physicalCharacteristics.optimalVoltage);
    angleMotor.setCurrentLimit(configuration.physicalCharacteristics.angleMotorCurrentLimit);
    driveMotor.setCurrentLimit(configuration.physicalCharacteristics.driveMotorCurrentLimit);
    angleMotor.setLoopRampRate(configuration.physicalCharacteristics.angleMotorRampRate);
    driveMotor.setLoopRampRate(configuration.physicalCharacteristics.driveMotorRampRate);

    // Config angle encoders
    absoluteEncoder = moduleConfiguration.absoluteEncoder;
    if (absoluteEncoder != null)
    {
      absoluteEncoder.factoryDefault();
      absoluteEncoder.configure(moduleConfiguration.absoluteEncoderInverted);
    }

    if (SwerveDriveTelemetry.isSimulation)
    {
      simModule = new SwerveModuleSimulation();
    }

    // Setup the cache for the absolute encoder position.
    absolutePositionCache = new Cache<>(this::getRawAbsolutePosition, 20);

    // Config angle motor/controller
    if (!angleMotor.usingExternalFeedbackSensor())
    {
      angleMotor.configureIntegratedEncoder(moduleConfiguration.conversionFactors.angle.factor);
    }
    angleMotor.configurePIDF(moduleConfiguration.anglePIDF);
    angleMotor.configurePIDWrapping(0, 360);
    angleMotor.setInverted(moduleConfiguration.angleMotorInverted);
    angleMotor.setMotorBrake(false);

    // Set the position AFTER settings the conversion factor.
    if (absoluteEncoder != null)
    {
      angleMotor.setPosition(getAbsolutePosition());
    }

    // Config drive motor/controller
    driveMotor.configureIntegratedEncoder(moduleConfiguration.conversionFactors.drive.factor);
    driveMotor.configurePIDF(moduleConfiguration.velocityPIDF);
    driveMotor.setInverted(moduleConfiguration.driveMotorInverted);
    driveMotor.setMotorBrake(true);

    driveMotor.burnFlash();
    angleMotor.burnFlash();

    drivePositionCache = new Cache<>(driveMotor::getPosition, 20);
    driveVelocityCache = new Cache<>(driveMotor::getVelocity, 20);

    // Force a cache update on init.
    driveVelocityCache.update();
    drivePositionCache.update();
    absolutePositionCache.update();

    // Save the current state.
    lastState = getState();

    noEncoderWarning = new Alert("Motors",
                                 "There is no Absolute Encoder on module #" +
                                 moduleNumber,
                                 AlertType.kWarning);
    encoderOffsetWarning = new Alert("Motors",
                                     "Pushing the Absolute Encoder offset to the encoder failed on module #" +
                                     moduleNumber,
                                     AlertType.kWarning);

    rawAbsoluteAnglePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
        "swerve/modules/" + configuration.name + "/Raw Absolute Encoder").publish();
    adjAbsoluteAnglePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
        "swerve/modules/" + configuration.name + "/Adjusted Absolute Encoder").publish();
    absoluteEncoderIssuePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getBooleanTopic(
        "swerve/modules/" + configuration.name + "/Absolute Encoder Read Issue").publish();
    rawAnglePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
        "swerve/modules/" + configuration.name + "/Raw Angle Encoder").publish();
    rawDriveEncoderPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
        "swerve/modules/" + configuration.name + "/Raw Drive Encoder").publish();
    rawDriveVelocityPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
        "swerve/modules/" + configuration.name + "/Raw Drive Velocity").publish();
    speedSetpointPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
        "swerve/modules/" + configuration.name + "/Speed Setpoint").publish();
    angleSetpointPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
        "swerve/modules/" + configuration.name + "/Angle Setpoint").publish();
  }

  @Override
  public void close()
  {
    angleMotor.close();
    driveMotor.close();
    absoluteEncoder.close();
  }

  /**
   * Get the default {@link SimpleMotorFeedforward} for the swerve module drive motor.
   *
   * @return {@link SimpleMotorFeedforward} using motor details.
   */
  public SimpleMotorFeedforward getDefaultFeedforward()
  {
    double nominalVoltage   = driveMotor.getSimMotor().nominalVoltageVolts;
    double maxDriveSpeedMPS = getMaxVelocity().in(MetersPerSecond);
    return SwerveMath.createDriveFeedforward(nominalVoltage,
                                             maxDriveSpeedMPS,
                                             configuration.physicalCharacteristics.wheelGripCoefficientOfFriction);
  }

  /**
   * Set utilization of {@link SwerveModuleState#optimize(Rotation2d)} which should be disabled for some debugging.
   *
   * @param optimizationState Optimization enabled.
   */
  public void setModuleStateOptimization(boolean optimizationState)
  {
    optimizeSwerveModuleState = optimizationState;
    if (!optimizeSwerveModuleState)
    {
      angleMotor.disablePIDWrapping();
      angleMotor.burnFlash();
    }
  }

  /**
   * Check if the module state optimization used by {@link SwerveModuleState#optimize(Rotation2d)} is enabled.
   *
   * @return optimization state.
   */
  public boolean getModuleStateOptimization()
  {
    return optimizeSwerveModuleState;
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param optimalVoltage Nominal voltage for operation to output to.
   */
  public void setAngleMotorVoltageCompensation(double optimalVoltage)
  {
    angleMotor.setVoltageCompensation(optimalVoltage);
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param optimalVoltage Nominal voltage for operation to output to.
   */
  public void setDriveMotorVoltageCompensation(double optimalVoltage)
  {
    driveMotor.setVoltageCompensation(optimalVoltage);
  }


  /**
   * Queue synchronization of the integrated angle encoder with the absolute encoder.
   */
  public void queueSynchronizeEncoders()
  {
    if (absoluteEncoder != null && synchronizeEncoderEnabled)
    {
      synchronizeEncoderQueued = true;
    }
  }

  /**
   * Enable auto synchronization for encoders during a match. This will only occur when the modules are not moving for a
   * few seconds.
   *
   * @param enabled  Enable state
   * @param deadband Deadband in degrees, default is 3 degrees.
   */
  public void setEncoderAutoSynchronize(boolean enabled, double deadband)
  {
    synchronizeEncoderEnabled = enabled;
    synchronizeEncoderDeadband = deadband;
  }

  /**
   * Enable auto synchronization for encoders during a match. This will only occur when the modules are not moving for a
   * few seconds.
   *
   * @param enabled Enable state
   */
  public void setEncoderAutoSynchronize(boolean enabled)
  {
    synchronizeEncoderEnabled = enabled;
  }

  /**
   * Set the antiJitter functionality, if true the modules will NOT auto center. Pushes the offsets to the angle motor
   * controllers as well.
   *
   * @param antiJitter Anti-Jitter state desired.
   */
  public void setAntiJitter(boolean antiJitter)
  {
    this.antiJitterEnabled = antiJitter;
    if (antiJitter)
    {
      pushOffsetsToEncoders();
    } else
    {
      restoreInternalOffset();
    }
  }

  /**
   * Set the feedforward attributes to the given parameters.
   *
   * @param drive Drive motor feedforward for the module.
   */
  public void setFeedforward(SimpleMotorFeedforward drive)
  {
    this.driveMotorFeedforward = drive;
  }

  /**
   * Get the current drive motor PIDF values.
   *
   * @return {@link PIDFConfig} of the drive motor.
   */
  public PIDFConfig getDrivePIDF()
  {
    return configuration.velocityPIDF;
  }

  /**
   * Set the drive PIDF values.
   *
   * @param config {@link PIDFConfig} of that should be set.
   */
  public void setDrivePIDF(PIDFConfig config)
  {
    configuration.velocityPIDF = config;
    driveMotor.configurePIDF(config);
  }

  /**
   * Get the current angle/azimuth/steering motor PIDF values.
   *
   * @return {@link PIDFConfig} of the angle motor.
   */
  public PIDFConfig getAnglePIDF()
  {
    return configuration.anglePIDF;
  }

  /**
   * Set the angle/azimuth/steering motor PID
   *
   * @param config {@link PIDFConfig} of that should be set.
   */
  public void setAnglePIDF(PIDFConfig config)
  {
    configuration.anglePIDF = config;
    angleMotor.configurePIDF(config);
  }

  /**
   * Set the desired state of the swerve module. <br /><b>WARNING: If you are not using one of the functions from
   * {@link SwerveDrive} you may screw up {@link SwerveDrive#kinematics}</b>
   *
   * @param desiredState Desired swerve module state.
   * @param isOpenLoop   Whether to use open loop (direct percent) or direct velocity control.
   * @param force        Disables optimizations that prevent movement in the angle motor and forces the desired state
   *                     onto the swerve module.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean force)
  {
    applyStateOptimizations(desiredState);
    applyAntiJitter(desiredState, force);

    // Cosine compensation.
    double nextVelocityMetersPerSecond = configuration.useCosineCompensator
                                         ? getCosineCompensatedVelocity(desiredState)
                                         : desiredState.speedMetersPerSecond;
    double curVelocityMetersPerSecond = lastState.speedMetersPerSecond;
    desiredState.speedMetersPerSecond = nextVelocityMetersPerSecond;

    setDesiredState(desiredState,
                    isOpenLoop,
                    driveMotorFeedforward.calculateWithVelocities(curVelocityMetersPerSecond,
                                                                  nextVelocityMetersPerSecond));
  }

  /**
   * Set the desired state of the swerve module. <br /><b>WARNING: If you are not using one of the functions from
   * {@link SwerveDrive} you may screw up {@link SwerveDrive#kinematics}</b>
   *
   * @param desiredState            Desired swerve module state.
   * @param isOpenLoop              Whether to use open loop (direct percent) or direct velocity control.
   * @param driveFeedforwardVoltage Drive motor controller feedforward as a voltage.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop,
                              double driveFeedforwardVoltage)
  {
    if (isOpenLoop)
    {
      double percentOutput = desiredState.speedMetersPerSecond / maxDriveVelocity.in(MetersPerSecond);
      driveMotor.setVoltage(percentOutput * 12);
    } else
    {
      driveMotor.setReference(desiredState.speedMetersPerSecond, driveFeedforwardVoltage);
    }

    // Prevent module rotation if angle is the same as the previous angle.
    // Synchronize encoders if queued and send in the current position as the value from the absolute encoder.
    if (absoluteEncoder != null && synchronizeEncoderQueued && synchronizeEncoderEnabled)
    {
      double absoluteEncoderPosition = getAbsolutePosition();
      if (Math.abs(angleMotor.getPosition() - absoluteEncoderPosition) >= synchronizeEncoderDeadband)
      {
        angleMotor.setPosition(absoluteEncoderPosition);
      }
      angleMotor.setReference(desiredState.angle.getDegrees(), 0, absoluteEncoderPosition);
      synchronizeEncoderQueued = false;
    } else
    {
      angleMotor.setReference(desiredState.angle.getDegrees(), 0);
    }

    lastState = desiredState;

    if (SwerveDriveTelemetry.isSimulation)
    {
      simModule.updateStateAndPosition(desiredState);
    }

    // TODO: Change and move to SwerveDriveTelemetry
    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.INFO.ordinal())
    {
      SwerveDriveTelemetry.desiredStatesObj[moduleNumber] = desiredState;
    }

    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
    {
      speedSetpointPublisher.set(desiredState.speedMetersPerSecond);
      angleSetpointPublisher.set(desiredState.angle.getDegrees());
    }

    if (moduleNumber == SwerveDriveTelemetry.moduleCount - 1)
    {
      SwerveDriveTelemetry.endCtrlCycle();
    }
  }

  /**
   * Get the cosine compensated velocity to set the swerve module to.
   *
   * @param desiredState Desired {@link SwerveModuleState} to use.
   * @return Cosine compensated velocity in meters/second.
   */
  private double getCosineCompensatedVelocity(SwerveModuleState desiredState)
  {
    double cosineScalar = 1.0;
    // Taken from the CTRE SwerveModule class.
    // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.46
    /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
    /* To reduce the "skew" that occurs when changing direction */
    /* If error is close to 0 rotations, we're already there, so apply full power */
    /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
    cosineScalar = Rotation2d.fromDegrees(desiredState.angle.getDegrees())
                             .minus(Rotation2d.fromDegrees(getAbsolutePosition()))
                             .getCos(); // TODO: Investigate angle modulus by 180.
    /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
    if (cosineScalar < 0.0)
    {
      cosineScalar = 1;
    }

    return desiredState.speedMetersPerSecond * cosineScalar;
  }

  /**
   * Apply the {@link SwerveModuleState#optimize(Rotation2d)} function if the module state optimization is enabled while
   * debugging.
   *
   * @param desiredState The desired state to apply the optimization to.
   */
  public void applyStateOptimizations(SwerveModuleState desiredState)
  {
    // SwerveModuleState optimization might be desired to be disabled while debugging.
    if (optimizeSwerveModuleState)
    {
      desiredState.optimize(Rotation2d.fromDegrees(getAbsolutePosition()));
    }
  }

  /**
   * Apply anti-jitter to the desired state. This will prevent the module from rotating if the speed requested is too
   * low. If force is true, the anti-jitter will not be applied.
   *
   * @param desiredState The desired state to apply the anti-jitter to.
   * @param force        Whether to ignore the {@link SwerveModule#antiJitterEnabled} state and apply the anti-jitter
   *                     anyway.
   */
  public void applyAntiJitter(SwerveModuleState desiredState, boolean force)
  {
    if (!force && antiJitterEnabled)
    {
      // Prevents module rotation if speed is less than 1%
      SwerveMath.antiJitter(desiredState, lastState, Math.min(maxDriveVelocityMetersPerSecond, 4));
    }
  }

  /**
   * Set the angle for the module.
   *
   * @param angle Angle in degrees.
   */
  public void setAngle(double angle)
  {
    angleMotor.setReference(angle, 0);
    lastState.angle = Rotation2d.fromDegrees(angle);
  }

  /**
   * Get the Swerve Module state.
   *
   * @return Current SwerveModule state.
   */
  public SwerveModuleState getState()
  {
    double     velocity;
    Rotation2d azimuth;
    if (!SwerveDriveTelemetry.isSimulation)
    {
      velocity = driveVelocityCache.getValue();
      azimuth = Rotation2d.fromDegrees(getAbsolutePosition());
    } else
    {
      return simModule.getState();
    }
    return new SwerveModuleState(velocity, azimuth);
  }

  /**
   * Get the position of the swerve module.
   *
   * @return {@link SwerveModulePosition} of the swerve module.
   */
  public SwerveModulePosition getPosition()
  {
    double     position;
    Rotation2d azimuth;
    if (!SwerveDriveTelemetry.isSimulation)
    {
      position = drivePositionCache.getValue();
      azimuth = Rotation2d.fromDegrees(getAbsolutePosition());
    } else
    {
      return simModule.getPosition();
    }
    return new SwerveModulePosition(position, azimuth);
  }

  /**
   * Get the absolute position. Falls back to relative position on reading failure.
   *
   * @return Absolute encoder angle in degrees in the range [0, 360).
   */
  public double getAbsolutePosition()
  {
    return absolutePositionCache.getValue();
  }

  /**
   * Get the absolute position. Falls back to relative position on reading failure.
   *
   * @return Absolute encoder angle in degrees in the range [0, 360).
   */
  public double getRawAbsolutePosition()
  {
    /* During simulation, when no absolute encoders are available, we return the state from the simulation module instead. */
    if (SwerveDriveTelemetry.isSimulation)
    {
      Rotation2d absolutePosition = simModule.getState().angle;
      return absolutePosition.getDegrees();
    }

    double angle;
    if (absoluteEncoder != null)
    {
      angle = absoluteEncoder.getAbsolutePosition() - angleOffset;
      if (absoluteEncoder.readingError)
      {
        angle = getRelativePosition();
      }
    } else
    {
      angle = getRelativePosition();
    }
    if (optimizeSwerveModuleState)
    {
      angle %= 360;
      if (angle < 0.0)
      {
        angle += 360;
      }
    }

    return angle;
  }

  /**
   * Get the relative angle in degrees.
   *
   * @return Angle in degrees.
   */
  public double getRelativePosition()
  {
    return angleMotor.getPosition();
  }

  /**
   * Set the brake mode.
   *
   * @param brake Set the brake mode.
   */
  public void setMotorBrake(boolean brake)
  {
    driveMotor.setMotorBrake(brake);
  }

  /**
   * Set the conversion factor for the angle/azimuth motor controller.
   *
   * @param conversionFactor Angle motor conversion factor for PID, should be generated from
   *                         {@link SwerveMath#calculateDegreesPerSteeringRotation(double, double)} or calculated.
   */
  public void setAngleMotorConversionFactor(double conversionFactor)
  {
    angleMotor.configureIntegratedEncoder(conversionFactor);
  }

  /**
   * Set the conversion factor for the drive motor controller.
   *
   * @param conversionFactor Drive motor conversion factor for PID, should be generated from
   *                         {@link SwerveMath#calculateMetersPerRotation(double, double, double)} or calculated.
   */
  public void setDriveMotorConversionFactor(double conversionFactor)
  {
    driveMotor.configureIntegratedEncoder(conversionFactor);
  }

  /**
   * Get the angle {@link SwerveMotor} for the {@link SwerveModule}.
   *
   * @return {@link SwerveMotor} for the angle/steering motor of the module.
   */
  public SwerveMotor getAngleMotor()
  {
    return angleMotor;
  }

  /**
   * Get the drive {@link SwerveMotor} for the {@link SwerveModule}.
   *
   * @return {@link SwerveMotor} for the drive motor of the module.
   */
  public SwerveMotor getDriveMotor()
  {
    return driveMotor;
  }

  /**
   * Get the {@link SwerveAbsoluteEncoder} for the {@link SwerveModule}.
   *
   * @return {@link SwerveAbsoluteEncoder} for the swerve module.
   */
  public SwerveAbsoluteEncoder getAbsoluteEncoder()
  {
    return absoluteEncoder;
  }

  /**
   * Fetch the {@link SwerveModuleConfiguration} for the {@link SwerveModule} with the parsed configurations.
   *
   * @return {@link SwerveModuleConfiguration} for the {@link SwerveModule}.
   */
  public SwerveModuleConfiguration getConfiguration()
  {
    return configuration;
  }


  /**
   * Use external sensors for the feedback of the angle/azimuth/steer controller.
   */
  public void useExternalFeedbackSensor()
  {
    if (absoluteEncoder == null)
    {
      externalSensorIsNull.set(true);
      return;
    }
    if (angleOffset == 0)
    {
      internalOffsetIsZero.set(true);
    }
    if (absoluteEncoder.setAbsoluteEncoderOffset(configuration.angleOffset))
    {
      angleMotor.setAbsoluteEncoder(absoluteEncoder);
      if (angleMotor.usingExternalFeedbackSensor())
      {
        angleOffset = 0;
      } else
      {
        externalFeedbackIncompatible.set(true);
        angleMotor.setAbsoluteEncoder(null);
        absoluteEncoder.setAbsoluteEncoderOffset(0);
      }

    } else
    {
      externalOffsetIncompatible.set(true);
      absoluteEncoder.setAbsoluteEncoderOffset(0);
    }
  }

  /**
   * Use external sensors for the feedback of the angle/azimuth/steer controller.
   */
  public void useInternalFeedbackSensor()
  {
    if (absoluteEncoder == null)
    {
      externalSensorIsNull.set(true);
      return;
    }
    if (angleOffset == 0)
    {
      internalOffsetIsZero.set(true);
    }
    angleMotor.setAbsoluteEncoder(null);
    absoluteEncoder.setAbsoluteEncoderOffset(0);
    angleOffset = configuration.angleOffset;
  }

  /**
   * Push absolute encoder offset in the memory of the encoder or controller. Also removes the internal angle offset.
   */
  @Deprecated
  public void pushOffsetsToEncoders()
  {
    if (absoluteEncoder != null && angleOffset == configuration.angleOffset)
    {
      // If the absolute encoder is attached.
      if (angleMotor instanceof SparkMaxSwerve || angleMotor instanceof SparkMaxBrushedMotorSwerve)
      {
        if (absoluteEncoder instanceof SparkMaxEncoderSwerve)
        {
          angleMotor.setAbsoluteEncoder(absoluteEncoder);
          if (absoluteEncoder.setAbsoluteEncoderOffset(angleOffset))
          {
            angleOffset = 0;
          } else
          {
            angleMotor.setAbsoluteEncoder(null);
            encoderOffsetWarning.set(true);
          }
        }
      }

    } else
    {
      noEncoderWarning.set(true);
    }
  }

  /**
   * Restore internal offset in YAGSL and either sets absolute encoder offset to 0 or restores old value.
   */
  public void restoreInternalOffset()
  {
    angleMotor.setAbsoluteEncoder(null);
    absoluteEncoder.setAbsoluteEncoderOffset(0);
    angleOffset = configuration.angleOffset;
  }

  /**
   * Get if the last Absolute Encoder had a read issue, such as it does not exist.
   *
   * @return If the last Absolute Encoder had a read issue, or absolute encoder does not exist.
   */
  public boolean getAbsoluteEncoderReadIssue()
  {
    if (absoluteEncoder == null)
    {
      return true;
    } else
    {
      return absoluteEncoder.readingError;
    }
  }


  /**
   * Get the maximum module velocity as a {@link LinearVelocity} based on the RPM and gear ratio.
   *
   * @return {@link LinearVelocity} max velocity of the drive wheel.
   */
  public LinearVelocity getMaxVelocity()
  {
    getMaxDriveVelocityMetersPerSecond();
    return maxDriveVelocity;
  }

  /**
   * Get the maximum drive velocity of the module in Meters Per Second.
   *
   * @return Maximum drive motor velocity in Meters Per Second.
   */
  public double getMaxDriveVelocityMetersPerSecond()
  {
    if (maxDriveVelocity == null)
    {
      maxDriveVelocity = InchesPerSecond.of(
          (driveMotor.getSimMotor().freeSpeedRadPerSec /
           configuration.conversionFactors.drive.gearRatio) *
          configuration.conversionFactors.drive.diameter / 2.0);
      maxDriveVelocityMetersPerSecond = maxDriveVelocity.in(MetersPerSecond);
    }
    return maxDriveVelocityMetersPerSecond;
  }

  /**
   * Get the maximum module angular velocity as a {@link AngularVelocity} based on the RPM and gear ratio.
   *
   * @return {@link AngularVelocity} max velocity of the angle/azimuth.
   */
  public AngularVelocity getMaxAngularVelocity()
  {
    if (maxAngularVelocity == null)
    {
      maxAngularVelocity = RotationsPerSecond.of(
          RadiansPerSecond.of(angleMotor.getSimMotor().freeSpeedRadPerSec).in(RotationsPerSecond) /
          configuration.conversionFactors.angle.gearRatio);
    }
    return maxAngularVelocity;
  }

  /**
   * Update data sent to {@link SmartDashboard}.
   */
  public void updateTelemetry()
  {
    if (absoluteEncoder != null)
    {
      rawAbsoluteAnglePublisher.set(absoluteEncoder.getAbsolutePosition());
    }
    if (SwerveDriveTelemetry.isSimulation && SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
    {
      SwerveModulePosition pos   = simModule.getPosition();
      SwerveModuleState    state = simModule.getState();
      rawAnglePublisher.set(pos.angle.getDegrees());
      rawDriveEncoderPublisher.set(pos.distanceMeters);
      rawDriveVelocityPublisher.set(state.speedMetersPerSecond);
      // For code coverage
      angleMotor.getPosition();
      drivePositionCache.getValue();
      driveVelocityCache.getValue();
    } else
    {
      rawAnglePublisher.set(angleMotor.getPosition());
      rawDriveEncoderPublisher.set(drivePositionCache.getValue());
      rawDriveVelocityPublisher.set(driveVelocityCache.getValue());
    }
    adjAbsoluteAnglePublisher.set(getAbsolutePosition());
    absoluteEncoderIssuePublisher.set(getAbsoluteEncoderReadIssue());

  }

  /**
   * Invalidate the {@link Cache} objects used by {@link SwerveModule}.
   */
  public void invalidateCache()
  {
    absolutePositionCache.update();
    drivePositionCache.update();
    driveVelocityCache.update();
  }

  /**
   * Obtains the {@link SwerveModuleSimulation} used in simulation.
   *
   * @return the module simulation, <b>null</b> if this method is called on a real robot
   */
  public SwerveModuleSimulation getSimModule()
  {
    return simModule;
  }

  /**
   * Configure the {@link SwerveModule#simModule} with the MapleSim
   * {@link org.ironmaple.simulation.drivesims.SwerveModuleSimulation}
   *
   * @param swerveModuleSimulation  MapleSim {@link org.ironmaple.simulation.drivesims.SwerveModuleSimulation} to
   *                                configure with.
   * @param physicalCharacteristics {@link SwerveModulePhysicalCharacteristics} that represent the swerve drive.
   */
  public void configureModuleSimulation(
      org.ironmaple.simulation.drivesims.SwerveModuleSimulation swerveModuleSimulation,
      SwerveModulePhysicalCharacteristics physicalCharacteristics)
  {
    this.simModule.configureSimModule(swerveModuleSimulation, physicalCharacteristics);
  }
}
