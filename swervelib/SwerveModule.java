package swervelib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.math.SwerveMath;
import swervelib.motors.SwerveMotor;
import swervelib.parser.Cache;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.simulation.SwerveModuleSimulation;
import swervelib.telemetry.Alert;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The Swerve Module class which represents and controls Swerve Modules for the swerve drive.
 */
public class SwerveModule
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
  public final int moduleNumber;
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
   * NT3 Raw Absolute Angle publisher for the absolute encoder.
   */
  private final String                 rawAbsoluteAngleName;
  /**
   * NT3 Adjusted Absolute angle publisher for the absolute encoder.
   */
  private final String                 adjAbsoluteAngleName;
  /**
   * NT3 Absolute encoder read issue.
   */
  private final String                 absoluteEncoderIssueName;
  /**
   * NT3 raw angle motor.
   */
  private final String                 rawAngleName;
  /**
   * NT3 Raw drive motor.
   */
  private final String                 rawDriveName;
  /**
   * NT3 Raw drive motor.
   */
  private final String                 rawDriveVelName;
  /**
   * Maximum speed of the drive motors in meters per second.
   */
  public        double                 maxSpeed;
  /**
   * Feedforward for the drive motor during closed loop control.
   */
  private       SimpleMotorFeedforward driveMotorFeedforward;
  /**
   * Anti-Jitter AKA auto-centering disabled.
   */
  private       boolean                antiJitterEnabled        = true;
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
   * Encoder synchronization queued.
   */
  private       boolean                synchronizeEncoderQueued = false;


  /**
   * Construct the swerve module and initialize the swerve module motors and absolute encoder.
   *
   * @param moduleNumber        Module number for kinematics.
   * @param moduleConfiguration Module constants containing CAN ID's and offsets.
   * @param driveFeedforward    Drive motor feedforward created by
   *                            {@link SwerveMath#createDriveFeedforward(double, double, double)}.
   */
  public SwerveModule(int moduleNumber, SwerveModuleConfiguration moduleConfiguration,
                      SimpleMotorFeedforward driveFeedforward)
  {
    //    angle = 0;
    //    speed = 0;
    //    omega = 0;
    //    fakePos = 0;
    this.moduleNumber = moduleNumber;
    configuration = moduleConfiguration;
    angleOffset = moduleConfiguration.angleOffset;

    // Initialize Feedforwards.
    driveMotorFeedforward = driveFeedforward;

    // Create motors from configuration and reset them to defaults.
    angleMotor = moduleConfiguration.angleMotor;
    driveMotor = moduleConfiguration.driveMotor;
    angleMotor.factoryDefaults();
    driveMotor.factoryDefaults();

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

    // Setup the cache for the absolute encoder position.
    absolutePositionCache = new Cache<>(this::getRawAbsolutePosition, 15);

    // Config angle motor/controller
    angleMotor.configureIntegratedEncoder(moduleConfiguration.conversionFactors.angle);
    angleMotor.configurePIDF(moduleConfiguration.anglePIDF);
    angleMotor.configurePIDWrapping(0, 180);
    angleMotor.setInverted(moduleConfiguration.angleMotorInverted);
    angleMotor.setMotorBrake(false);

    // Set the position AFTER settings the conversion factor.
    if (absoluteEncoder != null)
    {
      angleMotor.setPosition(getAbsolutePosition());
    }

    // Config drive motor/controller
    driveMotor.configureIntegratedEncoder(moduleConfiguration.conversionFactors.drive);
    driveMotor.configurePIDF(moduleConfiguration.velocityPIDF);
    driveMotor.setInverted(moduleConfiguration.driveMotorInverted);
    driveMotor.setMotorBrake(true);

    driveMotor.burnFlash();
    angleMotor.burnFlash();

    drivePositionCache = new Cache<>(driveMotor::getPosition, 15);
    driveVelocityCache = new Cache<>(driveMotor::getVelocity, 15);

    if (SwerveDriveTelemetry.isSimulation)
    {
      simModule = new SwerveModuleSimulation();
    }

    // Force a cache update on init.
    driveVelocityCache.update();
    drivePositionCache.update();
    absolutePositionCache.update();

    // Save the current state.
    lastState = getState();

    noEncoderWarning = new Alert("Motors",
                                 "There is no Absolute Encoder on module #" +
                                 moduleNumber,
                                 Alert.AlertType.WARNING);
    encoderOffsetWarning = new Alert("Motors",
                                     "Pushing the Absolute Encoder offset to the encoder failed on module #" +
                                     moduleNumber,
                                     Alert.AlertType.WARNING);

    rawAbsoluteAngleName = "swerve/modules/" + configuration.name + "/Raw Absolute Encoder";
    adjAbsoluteAngleName = "swerve/modules/" + configuration.name + "/Adjusted Absolute Encoder";
    absoluteEncoderIssueName = "swerve/modules/" + configuration.name + "/Absolute Encoder Read Issue";
    rawAngleName = "swerve/modules/" + configuration.name + "/Raw Angle Encoder";
    rawDriveName = "swerve/modules/" + configuration.name + "/Raw Drive Encoder";
    rawDriveVelName = "swerve/modules/" + configuration.name + "/Raw Drive Velocity";
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
    if (absoluteEncoder != null)
    {
      synchronizeEncoderQueued = true;
    }
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
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAbsolutePosition()));

    // If we are forcing the angle
    if (!force && antiJitterEnabled)
    {
      // Prevents module rotation if speed is less than 1%
      SwerveMath.antiJitter(desiredState, lastState, Math.min(maxSpeed, 4));
    }

    // Cosine compensation.
    double velocity = configuration.useCosineCompensator
                      ? getCosineCompensatedVelocity(desiredState)
                      : desiredState.speedMetersPerSecond;

    if (isOpenLoop)
    {
      double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
      driveMotor.set(percentOutput);
    } else
    {
      driveMotor.setReference(velocity, driveMotorFeedforward.calculate(velocity));
      desiredState.speedMetersPerSecond = velocity;
    }

    // Prevent module rotation if angle is the same as the previous angle.
    // Synchronize encoders if queued and send in the current position as the value from the absolute encoder.
    if (absoluteEncoder != null && synchronizeEncoderQueued)
    {
      double absoluteEncoderPosition = getAbsolutePosition();
      angleMotor.setPosition(absoluteEncoderPosition);
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

    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.INFO.ordinal())
    {
      SwerveDriveTelemetry.desiredStates[moduleNumber * 2] = desiredState.angle.getDegrees();
      SwerveDriveTelemetry.desiredStates[(moduleNumber * 2) + 1] = velocity;
    }

    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
    {
      SmartDashboard.putNumber("swerve/modules/" + configuration.name + "/Speed Setpoint",
                               desiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("swerve/modules/" + configuration.name + "/Angle Setpoint",
                               desiredState.angle.getDegrees());
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

    return desiredState.speedMetersPerSecond * (cosineScalar);
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
    angle %= 360;
    if (angle < 0.0)
    {
      angle += 360;
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
   * Push absolute encoder offset in the memory of the encoder or controller. Also removes the internal angle offset.
   */
  public void pushOffsetsToEncoders()
  {
    if (absoluteEncoder != null && angleOffset == configuration.angleOffset)
    {
      // If the absolute encoder is attached.
      if (angleMotor.getMotor() instanceof CANSparkMax)
      {
        if (absoluteEncoder.getAbsoluteEncoder() instanceof MotorFeedbackSensor)
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
   * Update data sent to {@link SmartDashboard}.
   */
  public void updateTelemetry()
  {
    if (absoluteEncoder != null)
    {
      SmartDashboard.putNumber(rawAbsoluteAngleName, absoluteEncoder.getAbsolutePosition());
    }
    SmartDashboard.putNumber(rawAngleName, angleMotor.getPosition());
    SmartDashboard.putNumber(rawDriveName, driveMotor.getPosition());
    SmartDashboard.putNumber(rawDriveVelName, driveMotor.getVelocity());
    SmartDashboard.putNumber(adjAbsoluteAngleName, getAbsolutePosition());
    SmartDashboard.putNumber(absoluteEncoderIssueName, getAbsoluteEncoderReadIssue() ? 1 : 0);
  }
}
