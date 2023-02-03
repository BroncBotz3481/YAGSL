package frc.robot.subsystems.swervedrive.swerve;

import static java.util.Objects.requireNonNull;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.swerve.SwerveMotor.ModuleMotorType;
import frc.robot.subsystems.swervedrive.swerve.kinematics.SwerveModuleState2;
import java.io.Closeable;
import java.util.HashMap;
import java.util.Map;

/**
 * Swerve module for representing a single swerve module of the robot.
 *
 * @param <DriveMotorType>      Main motor type that drives the wheel.
 * @param <AngleMotorType>      Motor that controls the angle of the wheel.
 * @param <AbsoluteEncoderType> Absolute encoder for the swerve drive module.
 */
public class SwerveModule<DriveMotorType extends MotorController, AngleMotorType extends MotorController,
    AbsoluteEncoderType>
    implements MotorController, Sendable, AutoCloseable
{

  /**
   * Swerve Module location object relative to the center of the robot.
   */
  public final  Translation2d                 swerveModuleLocation;
  /**
   * Motor Controllers for drive motor of the swerve module.
   */
  public final  SwerveMotor                   driveMotor;
  /***
   * Motor Controller for the turning motor of the swerve drive module.
   */
  public final  SwerveMotor                   turningMotor;
  /**
   * Enum representing the swerve module's location on the robot, assuming square.
   */
  private final SwerveModuleLocation          swerveLocation;
  /**
   * Absolute encoder for the swerve module.
   */
  private final SwerveEncoder<?>              absoluteEncoder;
  /**
   * The Distance between centers of right and left wheels in meters.
   */
  private final double                        driveTrainWidth;
  /**
   * The Distance between front and back wheels of the robot in meters.
   */
  private final double                        wheelBase;
  /**
   * kV for steering feedforward.
   */
  private final double                        steeringKV;
  private final ShuffleboardTab               moduleTab;
  private final HashMap<String, SimpleWidget> NT4Entries               = new HashMap<>();
  /**
   * Drive feedforward for PID when driving by velocity.
   */
  public        SimpleMotorFeedforward        driveFeedforward;
  /**
   * Angle offset of the CANCoder at initialization.
   */
  public        double                        angleOffset              = 0;
  /**
   * Maximum speed in meters per second, used to eliminate unnecessary movement of the module.
   */
  public        double                        maxDriveSpeedMPS;
  /**
   * Inverted drive motor.
   */
  private       boolean                       invertedDrive            = false;
  /**
   * Inverted turning motor.
   */
  private       boolean                       invertedTurn             = false;
  /**
   * Power to drive motor from -1 to 1.
   */
  private       double                        drivePower               = 0;
  /**
   * Store the last angle for optimization.
   */
  private       double                        targetAngle;
  /**
   * Angular velocity in radians per second.
   */
  private       double                        targetAngularVelocityRPS = 0;
  /**
   * Target velocity for the swerve module.
   */
  private       double                        targetVelocity           = 0;

  ///////////////////////////// CONFIGURATION FUNCTIONS SECTION ///////////////////////////////////////////////////

  /**
   * Swerve module constructor. Both motors <b>MUST</b> be a {@link MotorController} class. It is recommended to create
   * a command to reset the encoders when triggered and
   *
   * @param mainMotor                 Main drive motor. Must be a {@link MotorController} type.
   * @param angleMotor                Angle motor for controlling the angle of the swerve module.
   * @param encoder                   Absolute encoder for the swerve module.
   * @param driveGearRatio            Drive gear ratio in form of (rotation:1 AKA rotations/1) to get the encoder ticks
   *                                  per rotation.
   * @param steerGearRatio            Steering motor gear ratio (usually 12.8:1 for MK4 in form of rotations:1 or
   *                                  rotations/1), only applied if using Neo's.
   * @param swervePosition            Swerve Module position on the robot.
   * @param steeringOffsetDegrees     The current offset of the absolute encoder from 0 in degrees.
   * @param wheelDiameterMeters       The wheel diameter of the swerve drive module in meters.
   * @param wheelBaseMeters           The Distance between front and back wheels of the robot in meters.
   * @param driveTrainWidthMeters     The Distance between centers of right and left wheels in meters.
   * @param steeringMotorFreeSpeedRPM The RPM free speed of the steering motor.
   * @param maxSpeedMPS               The maximum drive speed in meters per second.
   * @param maxDriveAcceleration      The maximum drive acceleration in meters^2 per second.
   * @param drivingPowerLimit         The power limit for the closed loop PID of the driver motor.
   * @param steeringPowerLimit        The power limit for the closed loop PID of the steering motor.
   * @param steeringInverted          The steering motor is inverted.
   * @param drivingInverted           The driving motor is inverted.
   * @throws RuntimeException if an assertion fails or invalid swerve module location is given.
   */
  public SwerveModule(DriveMotorType mainMotor, AngleMotorType angleMotor, AbsoluteEncoderType encoder,
                      SwerveModuleLocation swervePosition, double driveGearRatio, double steerGearRatio,
                      double steeringOffsetDegrees, double wheelDiameterMeters, double wheelBaseMeters,
                      double driveTrainWidthMeters, double steeringMotorFreeSpeedRPM, double maxSpeedMPS,
                      double maxDriveAcceleration, double drivingPowerLimit, double steeringPowerLimit,
                      boolean steeringInverted, boolean drivingInverted)
  {
    // Steps to configure swerve drive are as follows
    // 1.  Set Current limit of turning motor to 20 amps
    // 2.  Enable voltage compensation with optimal battery voltage
    // 3.  Configure CANCoder
    // 4.  Set inverted motors.
    // 5.  Configure status frames
    // 6.  Set all motors to brake mode.
    // 7.  Set velocity and position conversion factors on drive motor encoder.
    // 8.  Set PIDF with integral zone on drive motor controller PID.
    // 9.  Set velocity and position conversion factors on turning motor controller. (Usually P=0.01,I=0,D=0,F=0,IZ=1)
    // 10. Set PIDF with integral zone on turning motor controller.
    // 11. Check all CAN devices are active.
    // 12. Reset the angle on the internal encoder to the absolute encoder.
    requireNonNull(mainMotor);
    requireNonNull(angleMotor);
    requireNonNull(encoder);

    this.wheelBase = wheelBaseMeters;
    this.driveTrainWidth = driveTrainWidthMeters;

    assert mainMotor instanceof CANSparkMax || mainMotor instanceof WPI_TalonFX;
    assert angleMotor instanceof CANSparkMax || angleMotor instanceof WPI_TalonFX;
    assert encoder instanceof DutyCycleEncoder || encoder instanceof AnalogEncoder || encoder instanceof CANCoder ||
           encoder instanceof AbsoluteEncoder;

    absoluteEncoder = SwerveEncoder.fromEncoder(encoder);
    assert absoluteEncoder != null;

    absoluteEncoder.factoryDefault();

    driveMotor = SwerveMotor.fromMotor(mainMotor,
                                       absoluteEncoder,
                                       ModuleMotorType.DRIVE,
                                       driveGearRatio,
                                       wheelDiameterMeters,
                                       0,
                                       drivingPowerLimit);

    turningMotor = SwerveMotor.fromMotor(angleMotor,
                                         absoluteEncoder,
                                         ModuleMotorType.TURNING,
                                         steerGearRatio,
                                         wheelDiameterMeters,
                                         steeringMotorFreeSpeedRPM,
                                         steeringPowerLimit);

    swerveLocation = swervePosition;
    swerveModuleLocation = getSwerveModulePosition(swervePosition);

    assert driveMotor != null;
    assert turningMotor != null;

    // Set the maximum speed for each swerve module for use when trying to optimize movements.
    // Drive feedforward gains
    //        public static final double KS = 0;
    //        public static final double KV = 12 / MAX_SPEED; // Volt-seconds per meter (max voltage divided by max
    //        speed)
    //        public static final double KA = 12 / MAX_ACCELERATION; // Volt-seconds^2 per meter (max voltage divided
    //        by max accel)
    maxDriveSpeedMPS = maxSpeedMPS;
    driveFeedforward = new SimpleMotorFeedforward(0, 12 / maxDriveSpeedMPS, 12 / maxDriveAcceleration);
    steeringKV = (12 * 60) / (steeringMotorFreeSpeedRPM * Math.toRadians(360 / steerGearRatio));

    driveMotor.setInverted(drivingInverted);
    invertedDrive = drivingInverted;
    turningMotor.setInverted(steeringInverted);
    invertedTurn = steeringInverted;

    absoluteEncoder.configure();

    setAngleOffset(steeringOffsetDegrees);
    resetEncoders();

    // Convert CANCoder to read data as unsigned 0 to 360 for synchronization purposes.
    absoluteEncoder.configure();

    assert activeCAN();

    resetEncoders();
    synchronizeSteeringEncoder();

    driveMotor.saveConfig();
    turningMotor.saveConfig();

    // targetAngle = getState().angle.getDegrees();
    targetAngle = 0;

    if (!remoteIntegratedEncoder() && Robot.isReal())
    {
      Robot.getInstance().addPeriodic(this::synchronizeSteeringEncoder, 0.5);
    }

    // Shuffleboard Data
    moduleTab = Shuffleboard.getTab(SwerveModule.SwerveModuleLocationToString(swerveLocation));
    publish(Verbosity.SETUP);

  }

  /**
   * Convert {@link SwerveModuleLocation} to {@link String} representation.
   *
   * @param swerveLocation Swerve position to convert.
   * @return {@link String} name of the {@link SwerveModuleLocation} enum.
   */
  public static String SwerveModuleLocationToString(SwerveModuleLocation swerveLocation)
  {
    switch (swerveLocation)
    {
      case FrontLeft:
        return "Front Left Module";
      case BackLeft:
        return "Back Left Module";
      case FrontRight:
        return "Front Right Module";
      case BackRight:
        return "Back Right Module";
      default:
        return "Unknown";
    }
  }

  /**
   * Reset the REV encoders onboard the SparkMax's to 0, and set's the drive motor to position to 0 and synchronizes the
   * internal steering encoders with the absolute encoder.
   */
  public void resetEncoders()
  {
    driveMotor.setEncoder(0);

    if (!remoteIntegratedEncoder())
    {
      turningMotor.setEncoder(0);
      synchronizeSteeringEncoder();
    }
  }

  /**
   * Synchronizes the internal encoder for the steering motor with the value from the absolute encoder.
   */
  public void synchronizeSteeringEncoder()
  {
    if (!remoteIntegratedEncoder())
    {
      if (absoluteEncoder.getMagnetFieldStrength() != MagnetFieldStrength.Good_GreenLED)
      {
        System.err.println("CANCoder magnetic field strength is unacceptable, will not synchronize encoders.");
        return;
      }
      turningMotor.setEncoder(absoluteEncoder.getAbsolutePosition() - angleOffset);
    }
  }

  /////////////////////// END OF CONFIGURATION FUNCTIONS SECTION //////////////////////////

  ////////////////////////////// STATUS FUNCTIONS SECTION //////////////////////////////////////////////////////

  /**
   * Set the PIDF coefficients for the closed loop PID onboard the motor controller. Tuning the PID
   * <p>
   * <b>P</b> = .5 and increase it by .1 until oscillations occur, then decrease by .05 then .005 until oscillations
   * stop and angle is perfect or near perfect.
   * </p>
   * <p>
   * <b>I</b> = 0 tune this if your PID never quite reaches the target, after tuning <b>D</b>. Increase this by
   * <b>P</b>*.01 each time and adjust accordingly.
   * </p>
   * <p>
   * <b>D</b> = 0 tune this if the PID accelerates too fast, it will smooth the motion. Increase this by <b>P</b>*10
   * and adjust accordingly.
   * </p>
   * <p>
   * <b>F</b> = 0 tune this if the PID is being used for velocity, the <b>F</b> is multiplied by the target and added
   * to the voltage output. Increase this by 0.01 until the PIDF reaches the desired state in a fast enough manner.
   * </p>
   * Documentation for this is best described by CTRE <a
   * href="https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#position-closed-loop-control-mode">here</a>.
   *
   * @param p               Proportional gain for closed loop. This is multiplied by closed loop error in sensor units.
   * @param i               Integral gain for closed loop. This is multiplied by closed loop error in sensor units every
   *                        PID Loop.
   * @param d               Derivative gain for closed loop. This is multiplied by derivative error (sensor units per
   *                        PID loop).
   * @param f               Feed Fwd gain for Closed loop.
   * @param integralZone    Integral Zone can be used to auto clear the integral accumulator if the sensor pos is too
   *                        far from the target. This prevents unstable oscillation if the kI is too large. Value is in
   *                        sensor units.
   * @param moduleMotorType Swerve drive motor type.
   */
  public void setPIDF(double p, double i, double d, double f, double integralZone,
                      ModuleMotorType moduleMotorType)
  {
    if ((moduleMotorType == ModuleMotorType.DRIVE))
    {
      driveMotor.setPIDF(p, i, d, f, integralZone);
    } else
    {
      turningMotor.setPIDF(p, i, d, f, integralZone);
    }
  }

  //////////////////////////// END OF STATUS FUNCTIONS SECTION ////////////////////////////////////////////////

  //////////////////////////// ODOMETRY AND STATE FUNCTIONS SECTION ///////////////////////////////////////////

  /**
   * Configure the magnetic offset in the CANCoder.
   *
   * @param offset Magnetic offset in degrees.
   */
  public void setAngleOffset(double offset)
  {
    // System.out.println(offset);
    angleOffset = offset;
    absoluteEncoder.setOffset(0);
  }

  /**
   * Check that the link is good on the swerve module and CAN bus is able to retrieve data.
   *
   * @return true on all devices are accessible over CAN.
   */
  public boolean activeCAN()
  {
    boolean drive = driveMotor.reachable(), turn = turningMotor.reachable(), encoder = absoluteEncoder.reachable();

    return drive && turn && encoder;
  }

  /**
   * Checks if the absolute encoder is able to be read directly by the motor controller instead of using the
   * synchronizeEncoder() function periodically.
   *
   * @return If the absolute sensor in the steering motor is directly accessed by the motor controller.
   */
  public boolean remoteIntegratedEncoder()
  {
    return turningMotor.remoteIntegratedEncoder();
  }

  /**
   * Set the angle of the swerve module.
   *
   * @param angle       Angle in degrees.
   * @param feedforward The feedforward for the PID.
   */
  public void setAngle(double angle, double feedforward)
  {
    turningMotor.setTarget(Math.round(angle) % 180, feedforward);
  }

  /**
   * Set the angle of the swerve module without feedforward.
   *
   * @param angle Angle in degrees.
   */
  private void setAngle(double angle)
  {
    turningMotor.setTarget(angle, 0);
  }

  /**
   * Set the drive motor velocity in MPS.
   *
   * @param velocity Velocity in MPS.
   */
  public void setVelocity(double velocity)
  {
    driveMotor.setTarget(velocity, driveFeedforward.calculate(velocity));
  }

  /**
   * Get the module state.
   *
   * @return SwerveModuleState with the encoder inputs.
   * @throws RuntimeException Exception if CANCoder doesnt exist
   */
  public SwerveModuleState2 getState()
  {
    double     mps = driveMotor.get();
    double     angularVelocityRPS;
    Rotation2d angle;
    angle = Rotation2d.fromDegrees(Math.round(
        Robot.isReal() ? absoluteEncoder.getAbsolutePosition() - angleOffset : targetAngle));
    angularVelocityRPS = Robot.isReal() ? Math.toRadians(absoluteEncoder.getVelocity()) : targetAngularVelocityRPS;
    //^ Convert degrees per second to radians per second.
    return new SwerveModuleState2(mps, angle, angularVelocityRPS);
  }

  /////////////////// END OF ODOMETRY AND STATE FUNCTIONS SECTION ////////////////////////////////////////

  /////////////////// DIAGNOSTIC AND TUNING FUNCTIONS SECTION ////////////////////////////////////////////

  /**
   * Set the module speed and angle based off the module state.
   *
   * @param state Module state.
   */
  public void setState(SwerveModuleState2 state)
  {
    // state.angle = state.angle.minus(Rotation2d.fromDegrees(angleOffset));
    // inspired by https://github.com/first95/FRC2022/blob/1f57d6837e04d8c8a89f4d83d71b5d2172f41a0e/SwervyBot/src/main/java/frc/robot/SwerveModule.java#L22
    Rotation2d currentAngle = getState().angle;
//    SwerveModuleState optimizedState = SwerveModuleState.optimize(new SwerveModuleState(state.speedMetersPerSecond,
//                                                                                        state.angle), currentAngle);
    SwerveModuleState2 optimizedState = SwerveModuleState2.optimize(state, currentAngle);
    double             angle          = optimizedState.angle.getDegrees(); // getDegrees returns in the range of -180
    // to 180 we want 0 to
    // 360.
    double velocity = optimizedState.speedMetersPerSecond;
    // turn motor code
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angle = (Math.abs(optimizedState.speedMetersPerSecond) <= (maxDriveSpeedMPS * 0.01)) ? targetAngle : angle;
    setAngle(angle, optimizedState.angularVelocityRadPerSecond * steeringKV);
    setVelocity(velocity);
    targetAngle = angle;
    targetVelocity = velocity;
    targetAngularVelocityRPS = optimizedState.angularVelocityRadPerSecond;
  }

  /**
   * Get the swerve module position based off the sensors.
   *
   * @return Swerve Module position, with the angle as what the angle is supposed to be, not what it actually is.
   */
  public SwerveModulePosition getPosition()
  {
    return new SwerveModulePosition(driveMotor.getPosition(), Rotation2d.fromDegrees(targetAngle));
  }

  //////////////////////////// END OF DIAGNOSTIC AND TUNING FUNCTIONS SECTION /////////////////////////

  /**
   * Subscribe from data within smart dashboard to make changes to the swerve module.
   */
  public void subscribe()
  {

  }

  //////////////////////////// ENUMS SECTION //////////////////////////////////////////////////////////

  /**
   * Create a widget and add the entry to the hashmap of entries for network tables.
   *
   * @param name         Key to display on shuffleboard.
   * @param defaultValue Default value.
   * @return Widget that can be modified.
   */
  private SimpleWidget addEntry(String name, Object defaultValue)
  {
    if (NT4Entries.containsKey(name))
    {
      GenericEntry entry = NT4Entries.get(name).getEntry();
      if (defaultValue instanceof Integer)
      {
        entry.setInteger((int) defaultValue);
      } else if (defaultValue instanceof Double)
      {
        entry.setDouble((double) defaultValue);
      } else if (defaultValue instanceof Boolean)
      {
        entry.setBoolean((boolean) defaultValue);
      } else if (defaultValue instanceof Double[])
      {
        entry.setDoubleArray((Double[]) defaultValue);
      } else
      {
        throw new RuntimeException("Cannot publish value for " + name);
      }
      return NT4Entries.get(name);
    }
    try
    {
      SimpleWidget widget = moduleTab.add(name, defaultValue);
      NT4Entries.put(name, widget);
      return widget;
    } catch (Exception e)
    {
      System.err.println(e);
    }
    throw new RuntimeException("Error creating entry");
  }

  /**
   * Publish data to the smart dashboard relating to this swerve module.
   *
   * @param level Verbosity level, affects the CAN utilization, on HIGH it will enable the update button.
   */
  public void publish(Verbosity level)
  {
    boolean goodLaptop = false;
    switch (level)
    {
      case SETUP:
        // PID
        addEntry("drive/pid/kP", driveMotor.kP);
        addEntry("drive/pid/kI", driveMotor.kI);
        addEntry("drive/pid/kD", driveMotor.kD);
        addEntry("drive/pid/kF", driveMotor.kF);
        addEntry("drive/pid/kIZ", driveMotor.kIZ);

        addEntry("steer/pid/kP", turningMotor.kP);
        addEntry("steer/pid/kI", turningMotor.kI);
        addEntry("steer/pid/kD", turningMotor.kD);
        addEntry("steer/pid/kF", turningMotor.kF);
        addEntry("steer/pid/kIZ", turningMotor.kIZ);

        // Pretty Widget Setup
        if (goodLaptop)
        {
          addEntry("Angle (Degrees)",
                   new Double[]{targetAngle, turningMotor.get(), absoluteEncoder.getAbsolutePosition()})
              .withPosition(0, 1)
              .withSize(4, 4)
              .withProperties(Map.of("visible time", 5))
              .withWidget(BuiltInWidgets.kGraph);
          addEntry("Absolute Angle", absoluteEncoder.getAbsolutePosition())
              .withPosition(0, 0)
              .withSize(2, 1);
          addEntry("Integrated Angle", turningMotor.get())
              .withPosition(2, 0)
              .withSize(2, 1);
          addEntry("Target Angle", targetAngle)
              .withPosition(8, 4)
              .withSize(2, 2);

          addEntry("Velocity (Meters Per Second)", new Double[]{targetVelocity, driveMotor.get()})
              .withPosition(4, 1)
              .withSize(4, 4)
              .withProperties(Map.of("visible time", 5))
              .withWidget(BuiltInWidgets.kGraph);
          addEntry("Motor Velocity", driveMotor.get())
              .withPosition(4, 0)
              .withSize(4, 1);
          addEntry("Target Velocity", targetVelocity)
              .withPosition(8, 2)
              .withSize(2, 2);

          // Inverted Motors.
          addEntry("Steer Motor Inverted", invertedTurn)
              .withPosition(0, 5)
              .withSize(4, 1);
          addEntry("Drive Motor inverted", invertedDrive)
              .withPosition(4, 5)
              .withSize(4, 1);

          addEntry("Absolute Encoder Magnetic Field", absoluteEncoder.getMagnetFieldStrength().value)
              .withPosition(10, 2)
              .withSize(2, 1);
          addEntry("Drive Motor Current", driveMotor.getAmps())
              .withPosition(10, 5)
              .withSize(2, 1);

          // Angle Constants
          addEntry("Absolute Encoder Offset", angleOffset)
              .withProperties(Map.of("min", -180, "max", 180))
              .withPosition(10, 3)
              .withSize(2, 2)
              .withWidget(BuiltInWidgets.kDial);
        } else
        {
          addEntry("Target Angle", targetAngle)
              .withProperties(Map.of("min", -180, "max", 180))
              .withWidget(BuiltInWidgets.kDial)
              .withPosition(0, 0)
              .withSize(4, 2);
          addEntry("Integrated Angle", turningMotor.get())
              .withProperties(Map.of("min", 0, "max", 360))
              .withWidget(BuiltInWidgets.kDial)
              .withPosition(0, 2)
              .withSize(2, 2);
          addEntry("Absolute Angle", absoluteEncoder.getAbsolutePosition())
              .withProperties(Map.of("min", 0, "max", 360))
              .withWidget(BuiltInWidgets.kDial)
              .withPosition(2, 2)
              .withSize(2, 2);

          addEntry("Target Velocity", targetVelocity)
              .withPosition(4, 0)
              .withProperties(Map.of("min", -maxDriveSpeedMPS, "max", maxDriveSpeedMPS))
              .withSize(4, 2)
              .withWidget(BuiltInWidgets.kNumberBar);
          addEntry("Motor Velocity", driveMotor.get())
              .withPosition(4, 2)
              .withProperties(Map.of("min", -maxDriveSpeedMPS, "max", maxDriveSpeedMPS))
              .withSize(4, 2)
              .withWidget(BuiltInWidgets.kNumberBar);

          // Inverted Motors.
          addEntry("Steer Motor Inverted", invertedTurn)
              .withPosition(0, 4)
              .withSize(4, 1);
          addEntry("Drive Motor inverted", invertedDrive)
              .withPosition(4, 4)
              .withSize(4, 1);

          addEntry("Absolute Encoder Magnetic Field", absoluteEncoder.getMagnetFieldStrength().value)
              .withPosition(8, 2)
              .withSize(2, 1);

          addEntry("Drive Motor Current", driveMotor.getAmps())
              .withPosition(10, 2)
              .withSize(2, 1);

          // Angle Constants
          addEntry("Absolute Encoder Offset", angleOffset)
              .withProperties(Map.of("min", -180, "max", 180))
              .withPosition(8, 3)
              .withSize(2, 2)
              .withWidget(BuiltInWidgets.kDial);
        }

        addEntry("CAN Connection", activeCAN())
            .withPosition(8, 0)
            .withSize(4, 2);

      case HIGH:
        // Update if button is set.
        // The higher the better, 2 and 3 are what we want.
        addEntry("Absolute Encoder Magnetic Field", absoluteEncoder.getMagnetFieldStrength().value);
      case NORMAL:
        // Steering Encoder Values
        if (goodLaptop)
        {
          addEntry("Angle (Degrees)",
                   new Double[]{targetAngle, turningMotor.get(), absoluteEncoder.getAbsolutePosition()});
        }
        addEntry("Absolute Angle", absoluteEncoder.getAbsolutePosition());
        addEntry("Integrated Angle", turningMotor.get());

        // Driving Encoder Values
        if (goodLaptop)
        {
          addEntry("Velocity (Meters Per Second)", new Double[]{targetVelocity, driveMotor.get()});
        }
        addEntry("Motor Velocity", driveMotor.get());
        addEntry("Drive Motor Current", driveMotor.getAmps());

        // CAN Bus is accessible
        addEntry("CAN Connection", activeCAN());

      case LOW:
        // PID
        addEntry("Target Velocity", targetVelocity);
        addEntry("Target Angle", targetAngle);

    }
  }

  /**
   * Initializes this {@link Sendable} object.
   *
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType(SwerveModuleLocationToString(swerveLocation) + " SwerveDriveModule");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
  }

  //////////////////////////////////// END OF ENUMS SECTION //////////////////////////////////////////////

  ////////////////////////////////// OVERRIDES SECTION ///////////////////////////////////////////////////

  /**
   * Closes this resource, relinquishing any underlying resources. This method is invoked automatically on objects
   * managed by the {@code try}-with-resources statement.
   *
   * <p>While this interface method is declared to throw {@code
   * Exception}, implementers are <em>strongly</em> encouraged to declare concrete implementations of the {@code close}
   * method to throw more specific exceptions, or to throw no exception at all if the close operation cannot fail.
   *
   * <p> Cases where the close operation may fail require careful
   * attention by implementers. It is strongly advised to relinquish the underlying resources and to internally
   * <em>mark</em> the resource as closed, prior to throwing the exception. The {@code close} method is unlikely to be
   * invoked more than once and so this ensures that the resources are released in a timely manner. Furthermore it
   * reduces problems that could arise when the resource wraps, or is wrapped, by another resource.
   *
   * <p><em>Implementers of this interface are also strongly advised
   * to not have the {@code close} method throw {@link InterruptedException}.</em>
   * <p>
   * This exception interacts with a thread's interrupted status, and runtime misbehavior is likely to occur if an
   * {@code InterruptedException} is {@linkplain Throwable#addSuppressed suppressed}.
   * <p>
   * More generally, if it would cause problems for an exception to be suppressed, the {@code AutoCloseable.close}
   * method should not throw it.
   *
   * <p>Note that unlike the {@link Closeable#close close}
   * method of {@link Closeable}, this {@code close} method is <em>not</em> required to be idempotent.  In other words,
   * calling this {@code close} method more than once may have some visible side effect, unlike {@code Closeable.close}
   * which is required to have no effect if called more than once.
   * <p>
   * However, implementers of this interface are strongly encouraged to make their {@code close} methods idempotent.
   */
  @Override
  public void close()
  {
    SendableRegistry.remove(this);
  }

  /**
   * Disable the motor controller.
   */
  @Override
  public void disable()
  {
    stopMotor();
  }

  /**
   * Stops motor movement. Motor can be moved again by calling set without having to re-enable the motor.
   */
  @Override
  public void stopMotor()
  {
//    turningMotor.stop();
    driveMotor.stop();
  }

  /**
   * Common interface for setting the speed of a motor controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  @Override
  public void set(double speed)
  {
    drivePower = speed;
    driveMotor.set(speed);
  }

  /**
   * Common interface for getting the current set speed of a motor controller.
   *
   * @return The current set speed. Value is between -1.0 and 1.0.
   */
  @Override
  public double get()
  {
    return drivePower;
  }

  /**
   * Get the swerve module position in {@link Translation2d} from the enum passed.
   *
   * @param swerveLocation Swerve module location enum.
   * @return Location as {@link Translation2d}.
   * @throws RuntimeException If Enum value is not defined.
   */
  private Translation2d getSwerveModulePosition(SwerveModuleLocation swerveLocation)
  {
    // Modeling off of https://github.com/Stampede3630/2022-Code/blob/master/src/main/java/frc/robot/SwerveDrive.java
    switch (swerveLocation)
    {
      case FrontLeft:
        return new Translation2d(wheelBase / 2, driveTrainWidth / 2);
      case BackLeft:
        return new Translation2d(-wheelBase / 2, driveTrainWidth / 2);
      case FrontRight:
        return new Translation2d(wheelBase / 2, -driveTrainWidth / 2);
      case BackRight:
        return new Translation2d(-wheelBase / 2, -driveTrainWidth / 2);
      default:
        throw new RuntimeException("Invalid location given");
    }
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  public void setVoltageCompensation(double nominalVoltage)
  {
    driveMotor.setVoltageCompensation(nominalVoltage);
    turningMotor.setVoltageCompensation(nominalVoltage);
  }

  //////////////////////////////////// END OF OVERRIDES SECTION ////////////////////////////////////////////

  //////////////////////////////////// STATIC FUNCTIONS SECTION ////////////////////////////////////////////

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   * @param type         Swerve Drive Motor type to configure.
   */
  public void setCurrentLimit(int currentLimit, ModuleMotorType type)
  {
    if (type == ModuleMotorType.DRIVE)
    {
      driveMotor.setCurrentLimit(currentLimit);
    } else
    {
      turningMotor.setCurrentLimit(currentLimit);
    }
  }

  /////////////////////////////////// END OF STATIC FUNCTIONS SECTION //////////////////////////////////////////

  /////////////////////////////////// EXTRA FUNCTIONS THAT PROBABLY WONT MATTER ////////////////////////////////

  /**
   * Set the steering motor to be inverted.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public void setInvertedTurn(boolean isInverted)
  {
    invertedTurn = isInverted;
    turningMotor.setInverted(isInverted);
  }

  /**
   * Invert the absolute encoder.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public void setInvertedAbsoluteEncoder(boolean isInverted)
  {
    absoluteEncoder.setInverted(isInverted);
  }

  ////////////// CUSTOM INVERSION FUNCTIONS SECTION //////////////////////////

  /**
   * Common interface for returning if a motor controller is in the inverted state or not.
   *
   * @return isInverted The state of the inversion true is inverted.
   */
  @Override
  public boolean getInverted()
  {
    return invertedDrive;
  }

  /**
   * Common interface for inverting direction of a motor controller.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  @Override
  public void setInverted(boolean isInverted)
  {
    invertedDrive = isInverted;
    driveMotor.setInverted(isInverted);
  }

  /**
   * Swerve Module location on the robot.
   */
  public enum SwerveModuleLocation
  {
    FrontLeft,
    BackLeft,
    FrontRight,
    BackRight
  }

  /**
   * Verbosity levels for data publishing,
   */
  public enum Verbosity
  {
    /**
     * The bare minimum and not utilize the CAN bus when reporting data. Only posts data from attributes.
     */
    LOW,
    /**
     * Utilize the CAN bus minimally.
     */
    NORMAL,
    /**
     * Extensively use the CAN bus to fetch data and report back.
     */
    HIGH,
    /**
     * Creates every field for the module.
     */
    SETUP
  }

  ////////////////////////////// END OF CUSTOM INVERSION FUNCTIONS SECTION /////////////////////////////////////

}