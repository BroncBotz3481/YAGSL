package swervelib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import swervelib.encoders.CANCoderSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;
import swervelib.motors.TalonFXSwerve;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.simulation.SwerveIMUSimulation;
import swervelib.telemetry.Alert;
import swervelib.telemetry.Alert.AlertType;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Swerve Drive class representing and controlling the swerve drive.
 */
public class SwerveDrive
{

  /**
   * Swerve Kinematics object.
   */
  public final  SwerveDriveKinematics    kinematics;
  /**
   * Swerve drive configuration.
   */
  public final  SwerveDriveConfiguration swerveDriveConfiguration;
  /**
   * Swerve odometry.
   */
  public final  SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  /**
   * Swerve modules.
   */
  private final SwerveModule[]           swerveModules;
  /**
   * WPILib {@link Notifier} to keep odometry up to date.
   */
  private final Notifier                 odometryThread;
  /**
   * Odometry lock to ensure thread safety.
   */
  private final Lock                     odometryLock                                    = new ReentrantLock();
  /**
   * Alert to recommend Tuner X if the configuration is compatible.
   */
  private final Alert                    tunerXRecommendation                            = new Alert("Swerve Drive",
                                                                                                     "Your Swerve Drive is compatible with Tuner X swerve generator, please consider using that instead of YAGSL. More information here!\n" +
                                                                                                     "https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html",
                                                                                                     AlertType.WARNING);
  /**
   * Field object.
   */
  public        Field2d                  field                                           = new Field2d();
  /**
   * Swerve controller for controlling heading of the robot.
   */
  public        SwerveController         swerveController;
  /**
   * Correct chassis velocity in {@link SwerveDrive#drive(Translation2d, double, boolean, boolean)} using 254's
   * correction.
   */
  public        boolean                  chassisVelocityCorrection                       = true;
  /**
   * Whether to correct heading when driving translationally. Set to true to enable.
   */
  public        boolean                  headingCorrection                               = false;
  /**
   * Deadband for speeds in heading correction.
   */
  private       double                   HEADING_CORRECTION_DEADBAND                     = 0.01;
  /**
   * Whether heading correction PID is currently active.
   */
  private       boolean                  correctionEnabled                               = false;
  /**
   * Swerve IMU device for sensing the heading of the robot.
   */
  private       SwerveIMU                imu;
  /**
   * Simulation of the swerve drive.
   */
  private       SwerveIMUSimulation      simIMU;
  /**
   * Counter to synchronize the modules relative encoder with absolute encoder when not moving.
   */
  private       int                      moduleSynchronizationCounter                    = 0;
  /**
   * The last heading set in radians.
   */
  private       double                   lastHeadingRadians                              = 0;
  /**
   * The absolute max speed that your robot can reach while translating in meters per second.
   */
  private       double                   attainableMaxTranslationalSpeedMetersPerSecond  = 0;
  /**
   * The absolute max speed the robot can reach while rotating radians per second.
   */
  private       double                   attainableMaxRotationalVelocityRadiansPerSecond = 0;
  /**
   * Maximum speed of the robot in meters per second.
   */
  private       double                   maxSpeedMPS;

  /**
   * Creates a new swerve drivebase subsystem. Robot is controlled via the {@link SwerveDrive#drive} method, or via the
   * {@link SwerveDrive#setRawModuleStates} method. The {@link SwerveDrive#drive} method incorporates kinematics-- it
   * takes a translation and rotation, as well as parameters for field-centric and closed-loop velocity control.
   * {@link SwerveDrive#setRawModuleStates} takes a list of SwerveModuleStates and directly passes them to the modules.
   * This subsystem also handles odometry.
   *
   * @param config           The {@link SwerveDriveConfiguration} configuration to base the swerve drive off of.
   * @param controllerConfig The {@link SwerveControllerConfiguration} to use when creating the
   *                         {@link SwerveController}.
   * @param maxSpeedMPS      Maximum speed in meters per second, remember to use {@link Units#feetToMeters(double)} if
   *                         you have feet per second!
   */
  public SwerveDrive(
      SwerveDriveConfiguration config, SwerveControllerConfiguration controllerConfig, double maxSpeedMPS)
  {
    this.maxSpeedMPS = maxSpeedMPS;
    swerveDriveConfiguration = config;
    swerveController = new SwerveController(controllerConfig);
    // Create Kinematics from swerve module locations.
    kinematics = new SwerveDriveKinematics(config.moduleLocationsMeters);
    odometryThread = new Notifier(this::updateOdometry);

    // Create an integrator for angle if the robot is being simulated to emulate an IMU
    // If the robot is real, instantiate the IMU instead.
    if (SwerveDriveTelemetry.isSimulation)
    {
      simIMU = new SwerveIMUSimulation();
    } else
    {
      imu = config.imu;
      imu.factoryDefault();
    }

    this.swerveModules = config.modules;

    //    odometry = new SwerveDriveOdometry(kinematics, getYaw(), getModulePositions());
    swerveDrivePoseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(new Translation2d(0, 0),
                       Rotation2d.fromDegrees(0))); // x,y,heading in radians; Vision measurement std dev, higher=less weight

    zeroGyro();

    // Initialize Telemetry
    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.LOW.ordinal())
    {
      SmartDashboard.putData("Field", field);
    }

    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
    {
      SwerveDriveTelemetry.maxSpeed = maxSpeedMPS;
      SwerveDriveTelemetry.maxAngularVelocity = swerveController.config.maxAngularVelocity;
      SwerveDriveTelemetry.moduleCount = swerveModules.length;
      SwerveDriveTelemetry.sizeFrontBack = Units.metersToInches(SwerveMath.getSwerveModule(swerveModules, true,
                                                                                           false).moduleLocation.getX() +
                                                                SwerveMath.getSwerveModule(swerveModules,
                                                                                           false,
                                                                                           false).moduleLocation.getX());
      SwerveDriveTelemetry.sizeLeftRight = Units.metersToInches(SwerveMath.getSwerveModule(swerveModules, false,
                                                                                           true).moduleLocation.getY() +
                                                                SwerveMath.getSwerveModule(swerveModules,
                                                                                           false,
                                                                                           false).moduleLocation.getY());
      SwerveDriveTelemetry.wheelLocations = new double[SwerveDriveTelemetry.moduleCount * 2];
      for (SwerveModule module : swerveModules)
      {
        SwerveDriveTelemetry.wheelLocations[module.moduleNumber * 2] = Units.metersToInches(
            module.configuration.moduleLocation.getX());
        SwerveDriveTelemetry.wheelLocations[(module.moduleNumber * 2) + 1] = Units.metersToInches(
            module.configuration.moduleLocation.getY());
      }
      SwerveDriveTelemetry.measuredStates = new double[SwerveDriveTelemetry.moduleCount * 2];
      SwerveDriveTelemetry.desiredStates = new double[SwerveDriveTelemetry.moduleCount * 2];
    }

    odometryThread.startPeriodic(SwerveDriveTelemetry.isSimulation ? 0.01 : 0.02);

    checkIfTunerXCompatible();
  }

  /**
   * Check all components to ensure that Tuner X Swerve Generator is recommended instead.
   */
  private void checkIfTunerXCompatible()
  {
    boolean compatible = imu instanceof Pigeon2Swerve;
    for (SwerveModule module : swerveModules)
    {
      compatible = compatible && module.getDriveMotor() instanceof TalonFXSwerve &&
                   module.getAngleMotor() instanceof TalonFXSwerve &&
                   module.getAbsoluteEncoder() instanceof CANCoderSwerve;
      if (!compatible)
      {
        break;
      }
    }
    if (compatible)
    {
      tunerXRecommendation.set(true);
    }

  }

  /**
   * Set the odometry update period in seconds.
   *
   * @param period period in seconds.
   */
  public void setOdometryPeriod(double period)
  {
    odometryThread.stop();
    odometryThread.startPeriodic(period);
  }

  /**
   * Stop the odometry thread in favor of manually updating odometry.
   */
  public void stopOdometryThread()
  {
    odometryThread.stop();
  }

  /**
   * Set the conversion factor for the angle/azimuth motor controller.
   *
   * @param conversionFactor Angle motor conversion factor for PID, should be generated from
   *                         {@link SwerveMath#calculateDegreesPerSteeringRotation(double, double)} or calculated.
   */
  public void setAngleMotorConversionFactor(double conversionFactor)
  {
    for (SwerveModule module : swerveModules)
    {
      module.setAngleMotorConversionFactor(conversionFactor);
    }
  }

  /**
   * Set the conversion factor for the drive motor controller.
   *
   * @param conversionFactor Drive motor conversion factor for PID, should be generated from
   *                         {@link SwerveMath#calculateMetersPerRotation(double, double, double)} or calculated.
   */
  public void setDriveMotorConversionFactor(double conversionFactor)
  {
    for (SwerveModule module : swerveModules)
    {
      module.setDriveMotorConversionFactor(conversionFactor);
    }
  }

  /**
   * Fetch the latest odometry heading, should be trusted over {@link SwerveDrive#getYaw()}.
   *
   * @return {@link Rotation2d} of the robot heading.
   */
  public Rotation2d getOdometryHeading()
  {
    return swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Set the heading correction capabilities of YAGSL.
   *
   * @param state {@link SwerveDrive#headingCorrection} state.
   */
  public void setHeadingCorrection(boolean state)
  {
    setHeadingCorrection(state, HEADING_CORRECTION_DEADBAND);
  }

  /**
   * Set the heading correction capabilities of YAGSL.
   *
   * @param state    {@link SwerveDrive#headingCorrection} state.
   * @param deadband {@link SwerveDrive#HEADING_CORRECTION_DEADBAND} deadband.
   */
  public void setHeadingCorrection(boolean state, double deadband)
  {
    headingCorrection = state;
    HEADING_CORRECTION_DEADBAND = deadband;
  }

  /**
   * Secondary method of controlling the drive base given velocity and adjusting it for field oriented use.
   *
   * @param velocity Velocity of the robot desired.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    ChassisSpeeds fieldOrientedVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, getOdometryHeading());
    drive(fieldOrientedVelocity);
  }

  /**
   * Secondary method of controlling the drive base given velocity and adjusting it for field oriented use.
   *
   * @param velocity               Velocity of the robot desired.
   * @param centerOfRotationMeters The center of rotation in meters, 0 is the center of the robot.
   */
  public void driveFieldOriented(ChassisSpeeds velocity, Translation2d centerOfRotationMeters)
  {
    ChassisSpeeds fieldOrientedVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, getOdometryHeading());
    drive(fieldOrientedVelocity, centerOfRotationMeters);
  }

  /**
   * Secondary method for controlling the drivebase. Given a simple {@link ChassisSpeeds} set the swerve module states,
   * to achieve the goal.
   *
   * @param velocity The desired robot-oriented {@link ChassisSpeeds} for the robot to achieve.
   */
  public void drive(ChassisSpeeds velocity)
  {
    drive(velocity, false, new Translation2d());
  }

  /**
   * Secondary method for controlling the drivebase. Given a simple {@link ChassisSpeeds} set the swerve module states,
   * to achieve the goal.
   *
   * @param velocity               The desired robot-oriented {@link ChassisSpeeds} for the robot to achieve.
   * @param centerOfRotationMeters The center of rotation in meters, 0 is the center of the robot.
   */
  public void drive(ChassisSpeeds velocity, Translation2d centerOfRotationMeters)
  {
    drive(velocity, false, centerOfRotationMeters);
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation rate, and calculates
   * and commands module states accordingly. Can use either open-loop or closed-loop velocity control for the wheel
   * velocities. Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation            {@link Translation2d} that is the commanded linear velocity of the robot, in meters
   *                               per second. In robot-relative mode, positive x is torwards the bow (front) and
   *                               positive y is torwards port (left). In field-relative mode, positive x is away from
   *                               the alliance wall (field North) and positive y is torwards the left wall when looking
   *                               through the driver station glass (field West).
   * @param rotation               Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot
   *                               relativity.
   * @param fieldRelative          Drive mode. True for field-relative, false for robot-relative.
   * @param isOpenLoop             Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   * @param centerOfRotationMeters The center of rotation in meters, 0 is the center of the robot.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
      Translation2d centerOfRotationMeters)
  {
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if
    // necessary.
    ChassisSpeeds velocity =
        fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getOdometryHeading())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    drive(velocity, isOpenLoop, centerOfRotationMeters);
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation rate, and calculates
   * and commands module states accordingly. Can use either open-loop or closed-loop velocity control for the wheel
   * velocities. Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x is away from the alliance wall (field
   *                      North) and positive y is torwards the left wall when looking through the driver station glass
   *                      (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  {
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if
    // necessary.
    ChassisSpeeds velocity =
        fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getOdometryHeading())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    drive(velocity, isOpenLoop, new Translation2d());
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link ChassisSpeeds}, and calculates and commands module
   * states accordingly. Can use either open-loop or closed-loop velocity control for the wheel velocities. Also has
   * field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param velocity               The chassis speeds to set the robot to achieve.
   * @param isOpenLoop             Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   * @param centerOfRotationMeters The center of rotation in meters, 0 is the center of the robot.
   */
  public void drive(ChassisSpeeds velocity, boolean isOpenLoop, Translation2d centerOfRotationMeters)
  {

    // Thank you to Jared Russell FRC254 for Open Loop Compensation Code
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
    if (chassisVelocityCorrection)
    {
      velocity = ChassisSpeeds.discretize(velocity, 0.02);
    }

    // Heading Angular Velocity Deadband, might make a configuration option later.
    // Originally made by Team 1466 Webb Robotics.
    // Modified by Team 7525 Pioneers and BoiledBurntBagel of 6036
    if (headingCorrection)
    {
      if (Math.abs(velocity.omegaRadiansPerSecond) < HEADING_CORRECTION_DEADBAND
          && (Math.abs(velocity.vxMetersPerSecond) > HEADING_CORRECTION_DEADBAND
              || Math.abs(velocity.vyMetersPerSecond) > HEADING_CORRECTION_DEADBAND))
      {
        if (!correctionEnabled)
        {
          lastHeadingRadians = getOdometryHeading().getRadians();
          correctionEnabled = true;
        }
        velocity.omegaRadiansPerSecond =
            swerveController.headingCalculate(lastHeadingRadians, getOdometryHeading().getRadians());
      } else
      {
        correctionEnabled = false;
      }
    }

    // Display commanded speed for testing
    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
    {
      SmartDashboard.putString("RobotVelocity", velocity.toString());
    }
    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
    {
      SwerveDriveTelemetry.desiredChassisSpeeds[1] = velocity.vyMetersPerSecond;
      SwerveDriveTelemetry.desiredChassisSpeeds[0] = velocity.vxMetersPerSecond;
      SwerveDriveTelemetry.desiredChassisSpeeds[2] = Math.toDegrees(velocity.omegaRadiansPerSecond);
    }

    // Calculate required module states via kinematics
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(velocity, centerOfRotationMeters);

    setRawModuleStates(swerveModuleStates, isOpenLoop);
  }


  /**
   * Set the maximum speeds for desaturation.
   *
   * @param attainableMaxModuleSpeedMetersPerSecond         The absolute max speed that a module can reach in meters per
   *                                                        second.
   * @param attainableMaxTranslationalSpeedMetersPerSecond  The absolute max speed that your robot can reach while
   *                                                        translating in meters per second.
   * @param attainableMaxRotationalVelocityRadiansPerSecond The absolute max speed the robot can reach while rotating in
   *                                                        radians per second.
   */
  public void setMaximumSpeeds(
      double attainableMaxModuleSpeedMetersPerSecond,
      double attainableMaxTranslationalSpeedMetersPerSecond,
      double attainableMaxRotationalVelocityRadiansPerSecond)
  {
    setMaximumSpeed(attainableMaxModuleSpeedMetersPerSecond);
    this.attainableMaxTranslationalSpeedMetersPerSecond = attainableMaxTranslationalSpeedMetersPerSecond;
    this.attainableMaxRotationalVelocityRadiansPerSecond = attainableMaxRotationalVelocityRadiansPerSecond;
    this.swerveController.config.maxAngularVelocity = attainableMaxRotationalVelocityRadiansPerSecond;
  }

  /**
   * Get the maximum velocity from {@link SwerveDrive#attainableMaxTranslationalSpeedMetersPerSecond} or
   * {@link SwerveDrive#maxSpeedMPS} whichever is higher.
   *
   * @return Maximum speed in meters/second.
   */
  public double getMaximumVelocity()
  {
    return Math.max(this.attainableMaxTranslationalSpeedMetersPerSecond, maxSpeedMPS);
  }

  /**
   * Get the maximum angular velocity, either {@link SwerveDrive#attainableMaxRotationalVelocityRadiansPerSecond} or
   * {@link SwerveControllerConfiguration#maxAngularVelocity}.
   *
   * @return Maximum angular velocity in radians per second.
   */
  public double getMaximumAngularVelocity()
  {
    return Math.max(this.attainableMaxRotationalVelocityRadiansPerSecond, swerveController.config.maxAngularVelocity);
  }

  /**
   * Set the module states (azimuth and velocity) directly. Used primarily for auto pathing.
   *
   * @param desiredStates A list of SwerveModuleStates to send to the modules.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  private void setRawModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop)
  {
    // Desaturates wheel speeds
    if (attainableMaxTranslationalSpeedMetersPerSecond != 0 || attainableMaxRotationalVelocityRadiansPerSecond != 0)
    {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, getRobotVelocity(),
                                                  maxSpeedMPS,
                                                  attainableMaxTranslationalSpeedMetersPerSecond,
                                                  attainableMaxRotationalVelocityRadiansPerSecond);
    } else
    {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeedMPS);
    }

    // Sets states
    for (SwerveModule module : swerveModules)
    {
      module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop, false);

      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
      {
        SwerveDriveTelemetry.desiredStates[module.moduleNumber *
                                           2] = module.lastState.angle.getDegrees();
        SwerveDriveTelemetry.desiredStates[(module.moduleNumber * 2) +
                                           1] = module.lastState.speedMetersPerSecond;
      }
    }
  }

  /**
   * Set the module states (azimuth and velocity) directly. Used primarily for auto paths.
   *
   * @param desiredStates A list of SwerveModuleStates to send to the modules.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop)
  {
    setRawModuleStates(kinematics.toSwerveModuleStates(kinematics.toChassisSpeeds(desiredStates)),
                       isOpenLoop);
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    SwerveDriveTelemetry.desiredChassisSpeeds[1] = chassisSpeeds.vyMetersPerSecond;
    SwerveDriveTelemetry.desiredChassisSpeeds[0] = chassisSpeeds.vxMetersPerSecond;
    SwerveDriveTelemetry.desiredChassisSpeeds[2] = Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond);

    setRawModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds), false);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {

    odometryLock.lock();
    Pose2d poseEstimation = swerveDrivePoseEstimator.getEstimatedPosition();
    odometryLock.unlock();
    return poseEstimation;
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the
    // angle
    // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        kinematics.toChassisSpeeds(getStates()), getOdometryHeading().unaryMinus());
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return kinematics.toChassisSpeeds(getStates());
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param pose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d pose)
  {
    odometryLock.lock();
    swerveDrivePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    odometryLock.unlock();
    kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, pose.getRotation()));
  }

  /**
   * Post the trajectory to the field
   *
   * @param trajectory the trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.LOW.ordinal())
    {
      field.getObject("Trajectory").setTrajectory(trajectory);
    }
  }

  /**
   * Gets the current module states (azimuth and velocity)
   *
   * @return A list of SwerveModuleStates containing the current module states
   */
  public SwerveModuleState[] getStates()
  {
    SwerveModuleState[] states = new SwerveModuleState[swerveDriveConfiguration.moduleCount];
    for (SwerveModule module : swerveModules)
    {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * Gets the current module positions (azimuth and wheel position (meters)).
   *
   * @return A list of SwerveModulePositions containg the current module positions
   */
  public SwerveModulePosition[] getModulePositions()
  {
    SwerveModulePosition[] positions =
        new SwerveModulePosition[swerveDriveConfiguration.moduleCount];
    for (SwerveModule module : swerveModules)
    {
      positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    // Resets the real gyro or the angle accumulator, depending on whether the robot is being
    // simulated
    if (!SwerveDriveTelemetry.isSimulation)
    {
      imu.setOffset(imu.getRawRotation3d());
    } else
    {
      simIMU.setAngle(0);
    }
    swerveController.lastAngleScalar = 0;
    lastHeadingRadians = 0;
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw as a {@link Rotation2d} angle
   */
  public Rotation2d getYaw()
  {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    if (!SwerveDriveTelemetry.isSimulation)
    {
      return Rotation2d.fromRadians(imu.getRotation3d().getZ());
    } else
    {
      return simIMU.getYaw();
    }
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    if (!SwerveDriveTelemetry.isSimulation)
    {
      return Rotation2d.fromRadians(imu.getRotation3d().getY());
    } else
    {
      return simIMU.getPitch();
    }
  }

  /**
   * Gets the current roll angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getRoll()
  {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    if (!SwerveDriveTelemetry.isSimulation)
    {
      return Rotation2d.fromRadians(imu.getRotation3d().getX());
    } else
    {
      return simIMU.getRoll();
    }
  }

  /**
   * Gets the current gyro {@link Rotation3d} of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation3d} angle
   */
  public Rotation3d getGyroRotation3d()
  {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    if (!SwerveDriveTelemetry.isSimulation)
    {
      return imu.getRotation3d();
    } else
    {
      return simIMU.getGyroRotation3d();
    }
  }

  /**
   * Gets current acceleration of the robot in m/s/s. If gyro unsupported returns empty.
   *
   * @return acceleration of the robot as a {@link Translation3d}
   */
  public Optional<Translation3d> getAccel()
  {
    if (!SwerveDriveTelemetry.isSimulation)
    {
      return imu.getAccel();
    } else
    {
      return simIMU.getAccel();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorIdleMode(boolean brake)
  {
    for (SwerveModule swerveModule : swerveModules)
    {
      swerveModule.setMotorBrake(brake);
    }
  }

  /**
   * Set the maximum speed of the drive motors, modified {@link SwerveDrive#maxSpeedMPS} which is used for the
   * {@link SwerveDrive#setRawModuleStates(SwerveModuleState[], boolean)} function and
   * {@link SwerveController#getTargetSpeeds(double, double, double, double, double)} functions. This function overrides
   * what was placed in the JSON and could damage your motor/robot if set too high or unachievable rates.
   *
   * @param maximumSpeed            Maximum speed for the drive motors in meters / second.
   * @param updateModuleFeedforward Update the swerve module feedforward to account for the new maximum speed. This
   *                                should be true unless you have replaced the drive motor feedforward with
   *                                {@link SwerveDrive#replaceSwerveModuleFeedforward(SimpleMotorFeedforward)}
   * @param optimalVoltage          Optimal voltage to use for the feedforward.
   */
  public void setMaximumSpeed(double maximumSpeed, boolean updateModuleFeedforward, double optimalVoltage)
  {
    maxSpeedMPS = maximumSpeed;
    swerveDriveConfiguration.physicalCharacteristics.optimalVoltage = optimalVoltage;
    for (SwerveModule module : swerveModules)
    {
      if (updateModuleFeedforward)
      {
        module.feedforward = SwerveMath.createDriveFeedforward(optimalVoltage,
                                                               maximumSpeed,
                                                               swerveDriveConfiguration.physicalCharacteristics.wheelGripCoefficientOfFriction);
      }
    }
  }

  /**
   * Set the maximum speed of the drive motors, modified {@link SwerveDrive#maxSpeedMPS} which is used for the
   * {@link SwerveDrive#setRawModuleStates(SwerveModuleState[], boolean)} function and
   * {@link SwerveController#getTargetSpeeds(double, double, double, double, double)} functions. This function overrides
   * what was placed in the JSON and could damage your motor/robot if set too high or unachievable rates. Overwrites the
   * {@link SwerveModule#feedforward}.
   *
   * @param maximumSpeed Maximum speed for the drive motors in meters / second.
   */
  public void setMaximumSpeed(double maximumSpeed)
  {
    setMaximumSpeed(maximumSpeed, true, swerveDriveConfiguration.physicalCharacteristics.optimalVoltage);
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move. Forcing the robot to keep
   * the current pose.
   */
  public void lockPose()
  {
    // Sets states
    for (SwerveModule swerveModule : swerveModules)
    {
      SwerveModuleState desiredState =
          new SwerveModuleState(0, swerveModule.configuration.moduleLocation.getAngle());
      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
      {
        SwerveDriveTelemetry.desiredStates[swerveModule.moduleNumber * 2] =
            desiredState.angle.getDegrees();
        SwerveDriveTelemetry.desiredStates[(swerveModule.moduleNumber * 2) + 1] =
            desiredState.speedMetersPerSecond;
      }
      swerveModule.setDesiredState(desiredState, false, true);

    }

    // Update kinematics because we are not using setModuleStates
    kinematics.toSwerveModuleStates(new ChassisSpeeds());
  }

  /**
   * Get the swerve module poses and on the field relative to the robot.
   *
   * @param robotPose Robot pose.
   * @return Swerve module poses.
   */
  public Pose2d[] getSwerveModulePoses(Pose2d robotPose)
  {
    Pose2d[]     poseArr = new Pose2d[swerveDriveConfiguration.moduleCount];
    List<Pose2d> poses   = new ArrayList<>();
    for (SwerveModule module : swerveModules)
    {
      poses.add(
          robotPose.plus(
              new Transform2d(module.configuration.moduleLocation, module.getState().angle)));
    }
    return poses.toArray(poseArr);
  }

  /**
   * Setup the swerve module feedforward.
   *
   * @param feedforward Feedforward for the drive motor on swerve modules.
   */
  public void replaceSwerveModuleFeedforward(SimpleMotorFeedforward feedforward)
  {
    for (SwerveModule swerveModule : swerveModules)
    {
      swerveModule.feedforward = feedforward;
    }
  }

  /**
   * Update odometry should be run every loop. Synchronizes module absolute encoders with relative encoders
   * periodically. In simulation mode will also post the pose of each module. Updates SmartDashboard with module encoder
   * readings and states.
   */
  public void updateOdometry()
  {
    odometryLock.lock();
    try
    {
      // Update odometry
      swerveDrivePoseEstimator.update(getYaw(), getModulePositions());

      // Update angle accumulator if the robot is simulated
      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
      {
        Pose2d[] modulePoses = getSwerveModulePoses(swerveDrivePoseEstimator.getEstimatedPosition());
        if (SwerveDriveTelemetry.isSimulation)
        {
          simIMU.updateOdometry(
              kinematics,
              getStates(),
              modulePoses,
              field);
        }

        ChassisSpeeds measuredChassisSpeeds = getRobotVelocity();
        SwerveDriveTelemetry.measuredChassisSpeeds[1] = measuredChassisSpeeds.vyMetersPerSecond;
        SwerveDriveTelemetry.measuredChassisSpeeds[0] = measuredChassisSpeeds.vxMetersPerSecond;
        SwerveDriveTelemetry.measuredChassisSpeeds[2] = Math.toDegrees(measuredChassisSpeeds.omegaRadiansPerSecond);
        SwerveDriveTelemetry.robotRotation = getOdometryHeading().getDegrees();
      }

      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.LOW.ordinal())
      {
        field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
      }

      double sumVelocity = 0;
      for (SwerveModule module : swerveModules)
      {
        SwerveModuleState moduleState = module.getState();
        sumVelocity += Math.abs(moduleState.speedMetersPerSecond);
        if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
        {
          module.updateTelemetry();
          SmartDashboard.putNumber("Raw IMU Yaw", getYaw().getDegrees());
          SmartDashboard.putNumber("Adjusted IMU Yaw", getOdometryHeading().getDegrees());
        }
        if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
        {
          SwerveDriveTelemetry.measuredStates[module.moduleNumber * 2] = moduleState.angle.getDegrees();
          SwerveDriveTelemetry.measuredStates[(module.moduleNumber * 2) + 1] = moduleState.speedMetersPerSecond;
        }
      }

      // If the robot isn't moving synchronize the encoders every 100ms (Inspired by democrat's SDS
      // lib)
      // To ensure that everytime we initialize it works.
      if (sumVelocity <= .01 && ++moduleSynchronizationCounter > 5)
      {
        synchronizeModuleEncoders();
        moduleSynchronizationCounter = 0;
      }

      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
      {
        SwerveDriveTelemetry.updateData();
      }
    } catch (Exception e)
    {
      odometryLock.unlock();
      throw e;
    }
    odometryLock.unlock();
  }

  /**
   * Synchronize angle motor integrated encoders with data from absolute encoders.
   */
  public void synchronizeModuleEncoders()
  {
    for (SwerveModule module : swerveModules)
    {
      module.queueSynchronizeEncoders();
    }
  }

  /**
   * Set the gyro scope offset to a desired known rotation. Unlike {@link SwerveDrive#setGyro(Rotation3d)} it DOES NOT
   * take the current rotation into account.
   *
   * @param offset {@link Rotation3d} known offset of the robot for gyroscope to use.
   */
  public void setGyroOffset(Rotation3d offset)
  {
    if (SwerveDriveTelemetry.isSimulation)
    {
      simIMU.setAngle(offset.getZ());
    } else
    {
      imu.setOffset(offset);
    }
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the {@link SwerveIMU} gyro reading with
   * the given timestamp of the vision measurement.
   *
   * @param robotPose                Robot {@link Pose2d} as measured by vision.
   * @param timestamp                Timestamp the measurement was taken as time since startup, should be taken from
   *                                 {@link Timer#getFPGATimestamp()} or similar sources.
   * @param visionMeasurementStdDevs Vision measurement standard deviation that will be sent to the
   *                                 {@link SwerveDrivePoseEstimator}.The standard deviation of the vision measurement,
   *                                 for best accuracy calculate the standard deviation at 2 or more  points and fit a
   *                                 line to it with the calculated optimal standard deviation. (Units should be meters
   *                                 per pixel). By optimizing this you can get * vision accurate to inches instead of
   *                                 feet.
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp,
                                   Matrix<N3, N1> visionMeasurementStdDevs)
  {
    odometryLock.lock();
    swerveDrivePoseEstimator.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
    odometryLock.unlock();
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the {@link SwerveIMU} gyro reading with
   * the given timestamp of the vision measurement. <br /> <b>IT IS HIGHLY RECOMMENDED TO UPDATE YOUR GYROSCOPE OFFSET
   * AFTER USING THIS FUNCTION!</b> <br /> To update your gyroscope readings directly use
   * {@link SwerveDrive#setGyroOffset(Rotation3d)}.
   *
   * @param robotPose Robot {@link Pose2d} as measured by vision.
   * @param timestamp Timestamp the measurement was taken as time since startup, should be taken from
   *                  {@link Timer#getFPGATimestamp()} or similar sources.
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp)
  {
    odometryLock.lock();
    swerveDrivePoseEstimator.addVisionMeasurement(robotPose, timestamp);
//    Pose2d newOdometry = new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(),
//                                    robotPose.getRotation());
    odometryLock.unlock();

//    setGyroOffset(new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
//    resetOdometry(newOdometry);
  }


  /**
   * Set the expected gyroscope angle using a {@link Rotation3d} object. To reset gyro, set to a new {@link Rotation3d}
   * subtracted from the current gyroscopic readings {@link SwerveIMU#getRotation3d()}.
   *
   * @param gyro expected gyroscope angle as {@link Rotation3d}.
   */
  public void setGyro(Rotation3d gyro)
  {
    if (SwerveDriveTelemetry.isSimulation)
    {
      setGyroOffset(simIMU.getGyroRotation3d().minus(gyro));
    } else
    {
      setGyroOffset(imu.getRawRotation3d().minus(gyro));
    }
  }

  /**
   * Helper function to get the {@link SwerveDrive#swerveController} for the {@link SwerveDrive} which can be used to
   * generate {@link ChassisSpeeds} for the robot to orient it correctly given axis or angles, and apply
   * {@link edu.wpi.first.math.filter.SlewRateLimiter} to given inputs. Important functions to look at are
   * {@link SwerveController#getTargetSpeeds(double, double, double, double, double)},
   * {@link SwerveController#addSlewRateLimiters(SlewRateLimiter, SlewRateLimiter, SlewRateLimiter)},
   * {@link SwerveController#getRawTargetSpeeds(double, double, double)}.
   *
   * @return {@link SwerveController} for the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveController;
  }

  /**
   * Get the {@link SwerveModule}s associated with the {@link SwerveDrive}.
   *
   * @return {@link SwerveModule} array specified by configurations.
   */
  public SwerveModule[] getModules()
  {
    return swerveDriveConfiguration.modules;
  }

  /**
   * Reset the drive encoders on the robot, useful when manually resetting the robot without a reboot, like in
   * autonomous.
   */
  public void resetDriveEncoders()
  {
    for (SwerveModule module : swerveModules)
    {
      module.configuration.driveMotor.setPosition(0);
    }
  }

  /**
   * Pushes the Absolute Encoder offsets to the Encoder or Motor Controller, depending on type. Also removes the
   * internal offsets to prevent double offsetting.
   */
  public void pushOffsetsToControllers()
  {
    for (SwerveModule module : swerveModules)
    {
      module.pushOffsetsToControllers();
    }
  }

  /**
   * Restores Internal YAGSL Encoder offsets and sets the Encoder stored offset back to 0
   */
  public void restoreInternalOffset()
  {
    for (SwerveModule module : swerveModules)
    {
      module.restoreInternalOffset();
    }
  }

}
