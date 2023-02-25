package swervelib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveKinematics2;
import swervelib.math.SwerveMath;
import swervelib.math.SwerveModuleState2;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.simulation.SwerveIMUSimulation;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Swerve Drive class representing and controlling the swerve drive.
 */
public class SwerveDrive
{

  /**
   * Swerve Kinematics object utilizing second order kinematics.
   */
  public final  SwerveKinematics2        kinematics;
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
   * Field object.
   */
  public        Field2d                  field                        = new Field2d();
  /**
   * Swerve controller for controlling heading of the robot.
   */
  public        SwerveController         swerveController;
  /**
   * Trustworthiness of the internal model of how motors should be moving Measured in expected standard deviation
   * (meters of position and degrees of rotation)
   */
  public        Matrix<N3, N1>           stateStdDevs                 = VecBuilder.fill(0.1, 0.1, 0.1);
  /**
   * Trustworthiness of the vision system Measured in expected standard deviation (meters of position and degrees of
   * rotation)
   */
  public        Matrix<N3, N1>           visionMeasurementStdDevs     = VecBuilder.fill(0.9, 0.9, 0.9);
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
  private       int                      moduleSynchronizationCounter = 0;
  /**
   * The last heading set in radians.
   */
  private       double                   lastHeadingRadians           = 0;

  /**
   * Creates a new swerve drivebase subsystem. Robot is controlled via the {@link SwerveDrive#drive} method, or via the
   * {@link SwerveDrive#setModuleStates} method. The {@link SwerveDrive#drive} method incorporates kinematics-- it takes
   * a translation and rotation, as well as parameters for field-centric and closed-loop velocity control.
   * {@link SwerveDrive#setModuleStates} takes a list of SwerveModuleStates and directly passes them to the modules.
   * This subsystem also handles odometry.
   *
   * @param config           The {@link SwerveDriveConfiguration} configuration to base the swerve drive off of.
   * @param controllerConfig The {@link SwerveControllerConfiguration} to use when creating the
   *                         {@link SwerveController}.
   */
  public SwerveDrive(
      SwerveDriveConfiguration config, SwerveControllerConfiguration controllerConfig)
  {
    swerveDriveConfiguration = config;
    swerveController = new SwerveController(controllerConfig);
    // Create Kinematics from swerve module locations.
    kinematics = new SwerveKinematics2(config.moduleLocationsMeters);

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
            new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
            stateStdDevs,
            visionMeasurementStdDevs); // x,y,heading in radians; Vision measurement std dev, higher=less weight

    zeroGyro();

    // Initialize Telemetry
    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.LOW.ordinal())
    {
      SmartDashboard.putData("Field", field);
    }

    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
    {
      SwerveDriveTelemetry.maxSpeed = swerveDriveConfiguration.maxSpeed;
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
  }

  /**
   * The primary method for controlling the drivebase. Takes a Translation2d and a rotation rate, and calculates and
   * commands module states accordingly. Can use either open-loop or closed-loop velocity control for the wheel
   * velocities. Also has field- and robot-relative modes, which affect how the translation vector is used. This method
   * defaults to no heading correction.
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
    drive(translation, rotation, fieldRelative, isOpenLoop, false);
  }

  /**
   * The primary method for controlling the drivebase. Takes a Translation2d and a rotation rate, and calculates and
   * commands module states accordingly. Can use either open-loop or closed-loop velocity control for the wheel
   * velocities. Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation       {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                          second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                          torwards port (left). In field-relative mode, positive x is away from the alliance wall
   *                          (field North) and positive y is torwards the left wall when looking through the driver
   *                          station glass (field West).
   * @param rotation          Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot
   *                          relativity.
   * @param fieldRelative     Drive mode. True for field-relative, false for robot-relative.
   * @param isOpenLoop        Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   * @param headingCorrection Whether to correct heading when driving translationally. Set to true to enable.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean headingCorrection)
  {
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if
    // necessary.
    ChassisSpeeds velocity =
        fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getYaw())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    // Heading Angular Velocity Deadband, might make a configuration option later.
    // Originally made by Team 1466 Webb Robotics.
    if (Math.abs(rotation) < 0.01 && headingCorrection)
    {
      velocity.omegaRadiansPerSecond =
          swerveController.headingCalculate(lastHeadingRadians, getYaw().getRadians());
    } else
    {
      lastHeadingRadians = getYaw().getRadians();
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
    SwerveModuleState2[] swerveModuleStates = kinematics.toSwerveModuleStates(velocity);

    setModuleStates(swerveModuleStates, isOpenLoop);
  }

  /**
   * Set the module states (azimuth and velocity) directly. Used primarily for auto pathing.
   *
   * @param desiredStates A list of SwerveModuleStates to send to the modules.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  public void setModuleStates(SwerveModuleState2[] desiredStates, boolean isOpenLoop)
  {
    // Desaturates wheel speeds
    SwerveKinematics2.desaturateWheelSpeeds(desiredStates, swerveDriveConfiguration.maxSpeed);

    // Sets states
    for (SwerveModule module : swerveModules)
    {
      module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop);

      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
      {
        SwerveModuleState2 moduleState = module.getState();
        SwerveDriveTelemetry.desiredStates[module.moduleNumber * 2] = moduleState.angle.getDegrees();
        SwerveDriveTelemetry.desiredStates[(module.moduleNumber * 2) + 1] = moduleState.speedMetersPerSecond;
      }
      if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
      {
        SmartDashboard.putNumber(
            "Module " + module.moduleNumber + " Speed Setpoint: ",
            desiredStates[module.moduleNumber].speedMetersPerSecond);
        SmartDashboard.putNumber(
            "Module " + module.moduleNumber + " Angle Setpoint: ",
            desiredStates[module.moduleNumber].angle.getDegrees());
      }
    }
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

    setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds), false);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrivePoseEstimator.getEstimatedPosition();
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
        kinematics.toChassisSpeeds(getStates()), getYaw().unaryMinus());
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
    swerveDrivePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
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
  public SwerveModuleState2[] getStates()
  {
    SwerveModuleState2[] states = new SwerveModuleState2[swerveDriveConfiguration.moduleCount];
    for (SwerveModule module : swerveModules)
    {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * Gets the current module positions (azimuth and wheel position (meters))
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
      imu.setYaw(0);
    } else
    {
      simIMU.setAngle(0);
    }
    swerveController.lastAngleScalar = 0;
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
      double[] ypr = new double[3];
      imu.getYawPitchRoll(ypr);
      return Rotation2d.fromDegrees(swerveDriveConfiguration.invertedIMU ? 360 - ypr[0] : ypr[0]);
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
      double[] ypr = new double[3];
      imu.getYawPitchRoll(ypr);
      return Rotation2d.fromDegrees(swerveDriveConfiguration.invertedIMU ? 360 - ypr[1] : ypr[1]);
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
      double[] ypr = new double[3];
      imu.getYawPitchRoll(ypr);
      return Rotation2d.fromDegrees(swerveDriveConfiguration.invertedIMU ? 360 - ypr[2] : ypr[2]);
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
      double[] ypr = new double[3];
      imu.getYawPitchRoll(ypr);
      return new Rotation3d(
          Math.toRadians(swerveDriveConfiguration.invertedIMU ? 360 - ypr[2] : ypr[2]),
          Math.toRadians(swerveDriveConfiguration.invertedIMU ? 360 - ypr[1] : ypr[1]),
          Math.toRadians(swerveDriveConfiguration.invertedIMU ? 360 - ypr[0] : ypr[0]));
    } else
    {
      return simIMU.getGyroRotation3d();
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
   * Point all modules toward the robot center, thus making the robot very difficult to move. Forcing the robot to keep
   * the current pose.
   */
  public void lockPose()
  {
    // Sets states
    for (SwerveModule swerveModule : swerveModules)
    {
      SwerveModuleState2 desiredState =
          new SwerveModuleState2(0, swerveModule.configuration.moduleLocation.getAngle(), 0);
      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
      {
        SwerveDriveTelemetry.desiredStates[swerveModule.moduleNumber * 2] =
            desiredState.angle.getDegrees();
        SwerveDriveTelemetry.desiredStates[(swerveModule.moduleNumber * 2) + 1] =
            desiredState.speedMetersPerSecond;
      }
      swerveModule.setDesiredState(desiredState, false);

    }
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
      SwerveDriveTelemetry.robotRotation = getYaw().getDegrees();
    }

    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.LOW.ordinal())
    {
      field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
    }

    double sumOmega = 0;
    for (SwerveModule module : swerveModules)
    {
      SwerveModuleState2 moduleState = module.getState();
      sumOmega += Math.abs(moduleState.omegaRadPerSecond);
      if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
      {
        SmartDashboard.putNumber(
            "Module" + module.moduleNumber + "Relative Encoder", module.getRelativePosition());
        SmartDashboard.putNumber(
            "Module" + module.moduleNumber + "Absolute Encoder", module.getAbsolutePosition());
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
    if (sumOmega <= .01 && ++moduleSynchronizationCounter > 5)
    {
      synchronizeModuleEncoders();
      moduleSynchronizationCounter = 0;
    }

    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
    {
      SwerveDriveTelemetry.updateData();
    }
  }

  /**
   * Synchronize angle motor integrated encoders with data from absolute encoders.
   */
  public void synchronizeModuleEncoders()
  {
    for (SwerveModule module : swerveModules)
    {
      module.synchronizeEncoders();
    }
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the {@link SwerveIMU} gyro reading with
   * the given timestamp of the vision measurement. <b>THIS WILL BREAK IF UPDATED TOO OFTEN.</b>
   *
   * @param robotPose       Robot {@link Pose2d} as measured by vision.
   * @param timestamp       Timestamp the measurement was taken as time since startup, should be taken from
   *                        {@link Timer#getFPGATimestamp()} or similar sources.
   * @param soft            Add vision estimate using the
   *                        {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)} function, or hard
   *                        reset odometry with the given position with
   *                        {@link edu.wpi.first.math.kinematics.SwerveDriveOdometry#resetPosition(Rotation2d,
   *                        SwerveModulePosition[], Pose2d)}.
   * @param trustWorthiness Trust level of vision reading when using a soft measurement, used to multiply the standard
   *                        deviation. Set to 1 for full trust.
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp, boolean soft, double trustWorthiness)
  {
    if (soft)
    {
      swerveDrivePoseEstimator.addVisionMeasurement(robotPose, timestamp,
                                                    visionMeasurementStdDevs.times(1.0 / trustWorthiness));
    } else
    {
      swerveDrivePoseEstimator.resetPosition(
          robotPose.getRotation(), getModulePositions(), robotPose);
    }

    if (!SwerveDriveTelemetry.isSimulation)
    {
      imu.setYaw(swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
      // Yaw reset recommended by Team 1622
    } else
    {
      simIMU.setAngle(swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians());
    }
  }
}
