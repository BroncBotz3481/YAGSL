package swervelib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
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
import java.util.Optional;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveKinematics2;
import swervelib.math.SwerveMath;
import swervelib.math.SwerveModuleState2;
import swervelib.math.SwervePoseEstimator2;
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
  public final  SwervePoseEstimator2     swerveDrivePoseEstimator;
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
   * Invert odometry readings of drive motor positions, used as a patch for debugging currently.
   */
  public  boolean             invertOdometry               = false;
  /**
   * Correct chassis velocity in {@link SwerveDrive#drive(Translation2d, double, boolean, boolean)} using 254's
   * correction.
   */
  public  boolean             chassisVelocityCorrection    = true;
  /**
   * Swerve IMU device for sensing the heading of the robot.
   */
  private SwerveIMU           imu;
  /**
   * Simulation of the swerve drive.
   */
  private SwerveIMUSimulation simIMU;
  /**
   * Counter to synchronize the modules relative encoder with absolute encoder when not moving.
   */
  private int                 moduleSynchronizationCounter = 0;
  /**
   * The last heading set in radians.
   */
  private double              lastHeadingRadians           = 0;

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
        new SwervePoseEstimator2(
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

    // Thank you to Jared Russell FRC254 for Open Loop Compensation Code
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
    if (chassisVelocityCorrection)
    {
      double dtConstant = 0.009;
      Pose2d robotPoseVel = new Pose2d(velocity.vxMetersPerSecond * dtConstant,
                                       velocity.vyMetersPerSecond * dtConstant,
                                       Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * dtConstant));
      Twist2d twistVel = SwerveMath.PoseLog(robotPoseVel);

      velocity = new ChassisSpeeds(twistVel.dx / dtConstant, twistVel.dy / dtConstant,
                                   twistVel.dtheta / dtConstant);
    }

    // Heading Angular Velocity Deadband, might make a configuration option later.
    // Originally made by Team 1466 Webb Robotics.
    if (headingCorrection)
    {
      if (Math.abs(rotation) < 0.01)
      {
        velocity.omegaRadiansPerSecond =
            swerveController.headingCalculate(lastHeadingRadians, getYaw().getRadians());
      } else
      {
        lastHeadingRadians = getYaw().getRadians();
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
    SwerveModuleState2[] swerveModuleStates = kinematics.toSwerveModuleStates(velocity);

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
    swerveDriveConfiguration.attainableMaxTranslationalSpeedMetersPerSecond =
        attainableMaxTranslationalSpeedMetersPerSecond;
    swerveDriveConfiguration.attainableMaxRotationalVelocityRadiansPerSecond =
        attainableMaxRotationalVelocityRadiansPerSecond;
  }

  /**
   * Set the module states (azimuth and velocity) directly. Used primarily for auto pathing.
   *
   * @param desiredStates A list of SwerveModuleStates to send to the modules.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  private void setRawModuleStates(SwerveModuleState2[] desiredStates, boolean isOpenLoop)
  {
    // Desaturates wheel speeds
    if (swerveDriveConfiguration.attainableMaxTranslationalSpeedMetersPerSecond != 0 ||
        swerveDriveConfiguration.attainableMaxRotationalVelocityRadiansPerSecond != 0)
    {
      SwerveKinematics2.desaturateWheelSpeeds(desiredStates, getRobotVelocity(),
                                              swerveDriveConfiguration.maxSpeed,
                                              swerveDriveConfiguration.attainableMaxTranslationalSpeedMetersPerSecond,
                                              swerveDriveConfiguration.attainableMaxRotationalVelocityRadiansPerSecond);
    } else
    {
      SwerveKinematics2.desaturateWheelSpeeds(desiredStates, swerveDriveConfiguration.maxSpeed);
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
      if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
      {
        SmartDashboard.putNumber(
            "Module[" + module.configuration.name + "] Speed Setpoint: ",
            module.lastState.speedMetersPerSecond);
        SmartDashboard.putNumber(
            "Module[" + module.configuration.name + "] Angle Setpoint: ",
            module.lastState.angle.getDegrees());
      }
    }
  }

  /**
   * Set the module states (azimuth and velocity) directly. Used primarily for auto paths.
   *
   * @param desiredStates A list of SwerveModuleStates to send to the modules.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  public void setModuleStates(SwerveModuleState2[] desiredStates, boolean isOpenLoop)
  {
    setRawModuleStates(kinematics.toSwerveModuleStates(kinematics.toChassisSpeeds(desiredStates)),
                       isOpenLoop);
  }

  /**
   * Set chassis speeds with closed-loop velocity control and second order kinematics.
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
   * Gets the current module positions (azimuth and wheel position (meters)). Inverts the distance from each module if
   * {@link #invertOdometry} is true.
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
      if (invertOdometry)
      {
        positions[module.moduleNumber].distanceMeters *= -1;
      }
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
      return swerveDriveConfiguration.invertedIMU
             ? Rotation2d.fromRadians(imu.getRotation3d().unaryMinus().getZ())
             : Rotation2d.fromRadians(imu.getRotation3d().getZ());
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
      return swerveDriveConfiguration.invertedIMU
             ? Rotation2d.fromRadians(imu.getRotation3d().unaryMinus().getY())
             : Rotation2d.fromRadians(imu.getRotation3d().getY());
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
      return swerveDriveConfiguration.invertedIMU
             ? Rotation2d.fromRadians(imu.getRotation3d().unaryMinus().getX())
             : Rotation2d.fromRadians(imu.getRotation3d().getX());
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
      return swerveDriveConfiguration.invertedIMU
             ? imu.getRotation3d().unaryMinus()
             : imu.getRotation3d();
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
   * Set the maximum speed of the drive motors, modified {@link SwerveControllerConfiguration#maxSpeed} and
   * {@link SwerveDriveConfiguration#maxSpeed} which is used for the
   * {@link SwerveDrive#setRawModuleStates(SwerveModuleState2[], boolean)} function and
   * {@link SwerveController#getTargetSpeeds(double, double, double, double, double)} functions. This function overrides
   * what was placed in the JSON and could damage your motor/robot if set too high or unachievable rates.
   *
   * @param maximumSpeed            Maximum speed for the drive motors in meters / second.
   * @param updateModuleFeedforward Update the swerve module feedforward to account for the new maximum speed. This
   *                                should be true unless you have replaced the drive motor feedforward with
   *                                {@link SwerveDrive#replaceSwerveModuleFeedforward(SimpleMotorFeedforward)}
   */
  public void setMaximumSpeed(double maximumSpeed, boolean updateModuleFeedforward)
  {
    swerveDriveConfiguration.maxSpeed = maximumSpeed;
    swerveController.config.maxSpeed = maximumSpeed;
    for (SwerveModule module : swerveModules)
    {
      module.configuration.maxSpeed = maximumSpeed;
      if (updateModuleFeedforward)
      {
        module.feedforward = module.configuration.createDriveFeedforward();
      }
    }
  }

  /**
   * Set the maximum speed of the drive motors, modified {@link SwerveControllerConfiguration#maxSpeed} and
   * {@link SwerveDriveConfiguration#maxSpeed} which is used for the
   * {@link SwerveDrive#setRawModuleStates(SwerveModuleState2[], boolean)} function and
   * {@link SwerveController#getTargetSpeeds(double, double, double, double, double)} functions. This function overrides
   * what was placed in the JSON and could damage your motor/robot if set too high or unachievable rates. Overwrites the
   * {@link SwerveModule#feedforward}.
   *
   * @param maximumSpeed Maximum speed for the drive motors in meters / second.
   */
  public void setMaximumSpeed(double maximumSpeed)
  {
    setMaximumSpeed(maximumSpeed, true);
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
    // Update odometry
    swerveDrivePoseEstimator.update(getYaw(), getPitch(), getRoll(), getModulePositions());

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
            "Module[" + module.configuration.name + "] Relative Encoder", module.getRelativePosition());
        SmartDashboard.putNumber(
            "Module[" + module.configuration.name + "] Absolute Encoder", module.getAbsolutePosition());
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
      module.queueSynchronizeEncoders();
    }
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the {@link SwerveIMU} gyro reading with
   * the given timestamp of the vision measurement.
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
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the {@link SwerveIMU} gyro reading with
   * the given timestamp of the vision measurement.
   *
   * @param robotPose                Robot {@link Pose2d} as measured by vision.
   * @param timestamp                Timestamp the measurement was taken as time since startup, should be taken from
   *                                 {@link Timer#getFPGATimestamp()} or similar sources.
   * @param soft                     Add vision estimate using the
   *                                 {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)} function, or
   *                                 hard reset odometry with the given position with
   *                                 {@link edu.wpi.first.math.kinematics.SwerveDriveOdometry#resetPosition(Rotation2d,
   *                                 SwerveModulePosition[], Pose2d)}.
   * @param visionMeasurementStdDevs Vision measurement standard deviation that will be sent to the
   *                                 {@link SwerveDrivePoseEstimator}.
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp, boolean soft,
                                   Matrix<N3, N1> visionMeasurementStdDevs)
  {
    this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    addVisionMeasurement(robotPose, timestamp, soft, 1);
  }


  /**
   * Set the expected gyroscope angle using a {@link Rotation3d} object. To reset gyro, set to a new
   * {@link Rotation3d}.
   *
   * @param gyro expected gyroscope angle.
   */
  public void setGyro(Rotation3d gyro)
  {
    imu.setOffset(imu.getRawRotation3d().minus(gyro));
  }

  /**
   * Helper function to get the {@link SwerveDrive#swerveController} for the {@link SwerveDrive} which can be used to
   * generate {@link ChassisSpeeds} for the robot to orient it correctly given axis or angles, and apply
   * {@link edu.wpi.first.math.filter.SlewRateLimiter} to given inputs. Important functions to look at are
   * {@link SwerveController#getTargetSpeeds(double, double, double, double)},
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
  public void resetEncoders()
  {
    for (SwerveModule module : swerveModules)
    {
      module.configuration.driveMotor.setPosition(0);
    }
  }

  /**
   * Enable second order kinematics for simulation and modifying the feedforward. Second order kinematics could increase
   * accuracy in odometry.
   *
   * @param moduleFeedforward Module feedforward to apply should be between [-1, 1] excluding 0.
   */
  public void enableSecondOrderKinematics(double moduleFeedforward)
  {
    for (SwerveModule module : swerveModules)
    {
      module.configuration.moduleSteerFFCL = moduleFeedforward;
    }
  }

  /**
   * Enable second order kinematics for tracking purposes but completely untuned.
   */
  public void enableSecondOrderKinematics()
  {
    enableSecondOrderKinematics(-0.00000000000000001);
  }

  /**
   * Disable second order kinematics.
   */
  public void disableSecondOrderKinematics()
  {
    enableSecondOrderKinematics(0);
  }

}
