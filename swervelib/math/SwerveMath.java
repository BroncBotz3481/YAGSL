package swervelib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveController;
import swervelib.SwerveModule;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;

/**
 * Mathematical functions which pertain to swerve drive.
 */
public class SwerveMath
{

  /**
   * Calculate the angle kV which will be multiplied by the radians per second for the feedforward. Volt * seconds /
   * degree == (maxVolts) / (maxSpeed)
   *
   * @param optimalVoltage    Optimal voltage to use when calculating the angle kV.
   * @param motorFreeSpeedRPM Motor free speed in Rotations per Minute.
   * @param angleGearRatio    Angle gear ratio, the amount of times the motor as to turn for the wheel rotation.
   * @return angle kV for feedforward.
   */
  public static double calculateAngleKV(
      double optimalVoltage, double motorFreeSpeedRPM, double angleGearRatio)
  {
    double maxAngularVelocity = 360 * (motorFreeSpeedRPM / angleGearRatio) / 60; // deg/s
    return optimalVoltage / maxAngularVelocity;
  }

  /**
   * Calculate the meters per rotation for the integrated encoder. Calculation: 4in diameter wheels * pi [circumfrence]
   * / gear ratio.
   *
   * @param wheelDiameter    Wheel diameter in meters.
   * @param driveGearRatio   The gear ratio of the drive motor.
   * @param pulsePerRotation The number of encoder pulses per rotation. 1 if using an integrated encoder.
   * @return Meters per rotation for the drive motor.
   */
  public static double calculateMetersPerRotation(
      double wheelDiameter, double driveGearRatio, double pulsePerRotation)
  {
    return (Math.PI * wheelDiameter) / (driveGearRatio * pulsePerRotation);
  }

  /**
   * Normalize an angle to be within 0 to 360.
   *
   * @param angle Angle in degrees.
   * @return Normalized angle in degrees.
   */
  public static double normalizeAngle(double angle)
  {
    Rotation2d angleRotation = Rotation2d.fromDegrees(angle);
    return new Rotation2d(angleRotation.getCos(), angleRotation.getSin()).getDegrees();
  }

  /**
   * Algebraically apply a deadband using a piece wise function.
   *
   * @param value    value to apply deadband to.
   * @param scaled   Use algebra to determine deadband by starting the value at 0 past deadband.
   * @param deadband The deadbnad to apply.
   * @return Value with deadband applied.
   */
  public static double applyDeadband(double value, boolean scaled, double deadband)
  {
    value = Math.abs(value) > deadband ? value : 0;
    return scaled
           ? ((1 / (1 - deadband)) * (Math.abs(value) - deadband)) * Math.signum(value)
           : value;
  }

  /**
   * Calculate the degrees per steering rotation for the integrated encoder. Encoder conversion values. Drive converts
   * motor rotations to linear wheel distance and steering converts motor rotations to module azimuth.
   *
   * @param angleGearRatio   The gear ratio of the steering motor.
   * @param pulsePerRotation The number of pulses in a complete rotation for the encoder, 1 if integrated.
   * @return Degrees per steering rotation for the angle motor.
   */
  public static double calculateDegreesPerSteeringRotation(
      double angleGearRatio, double pulsePerRotation)
  {
    return 360 / (angleGearRatio * pulsePerRotation);
  }

  /**
   * Calculate the maximum angular velocity.
   *
   * @param maxSpeed        Max speed of the robot in meters per second.
   * @param furthestModuleX X of the furthest module in meters.
   * @param furthestModuleY Y of the furthest module in meters.
   * @return Maximum angular velocity in rad/s.
   */
  public static double calculateMaxAngularVelocity(
      double maxSpeed, double furthestModuleX, double furthestModuleY)
  {
    return maxSpeed / Math.hypot(furthestModuleX, furthestModuleY);
  }

  /**
   * Calculate the practical maximum acceleration of the robot using the wheel coefficient of friction.
   *
   * @param cof Coefficient of Friction of the wheel grip tape.
   * @return Practical maximum acceleration in m/s/s.
   */
  public static double calculateMaxAcceleration(double cof)
  {
    return cof * 9.81;
  }

  /**
   * Calculate the maximum theoretical acceleration without friction.
   *
   * @param stallTorqueNm Stall torque of driving motor in nM.
   * @param gearRatio     Gear ratio for driving motor number of motor rotations until one wheel rotation.
   * @param moduleCount   Number of swerve modules.
   * @param wheelDiameter Wheel diameter in meters.
   * @param robotMass     Mass of the robot in kg.
   * @return Theoretical maximum acceleration in m/s/s.
   */
  public static double calculateMaxAcceleration(
      double stallTorqueNm,
      double gearRatio,
      double moduleCount,
      double wheelDiameter,
      double robotMass)
  {
    return (stallTorqueNm * gearRatio * moduleCount) / ((wheelDiameter / 2) * robotMass);
  }

  /**
   * Calculates the maximum acceleration allowed in a direction without tipping the robot. Reads arm position from
   * NetworkTables and is passed the direction in question.
   *
   * @param angle                  The direction in which to calculate max acceleration, as a Rotation2d. Note that this
   *                               is robot-relative.
   * @param chassisMass            Chassis mass in kg. (The weight of just the chassis not anything else)
   * @param robotMass              The weight of the robot in kg. (Including manipulators, etc).
   * @param chassisCenterOfGravity Chassis center of gravity.
   * @param config                 The swerve drive configuration.
   * @return Maximum acceleration allowed in the robot direction.
   */
  private static double calcMaxAccel(
      Rotation2d angle,
      double chassisMass,
      double robotMass,
      Translation3d chassisCenterOfGravity,
      SwerveDriveConfiguration config)
  {
    double xMoment = (chassisCenterOfGravity.getX() * chassisMass);
    double yMoment = (chassisCenterOfGravity.getY() * chassisMass);
    // Calculate the vertical mass moment using the floor as the datum.  This will be used later to
    // calculate max
    // acceleration
    double        zMoment      = (chassisCenterOfGravity.getZ() * (chassisMass));
    Translation3d robotCG      = new Translation3d(xMoment, yMoment, zMoment).div(robotMass);
    Translation2d horizontalCG = robotCG.toTranslation2d();

    Translation2d projectedHorizontalCg =
        new Translation2d(
            (angle.getSin() * angle.getCos() * horizontalCG.getY())
            + (Math.pow(angle.getCos(), 2) * horizontalCG.getX()),
            (angle.getSin() * angle.getCos() * horizontalCG.getX())
            + (Math.pow(angle.getSin(), 2) * horizontalCG.getY()));

    // Projects the edge of the wheelbase onto the direction line.  Assumes the wheelbase is
    // rectangular.
    // Because a line is being projected, rather than a point, one of the coordinates of the
    // projected point is
    // already known.
    Translation2d projectedWheelbaseEdge;
    double        angDeg = angle.getDegrees();
    if (angDeg <= 45 && angDeg >= -45)
    {
      SwerveModuleConfiguration conf = getSwerveModule(config.modules, true, true);
      projectedWheelbaseEdge =
          new Translation2d(
              conf.moduleLocation.getX(), conf.moduleLocation.getX() * angle.getTan());
    } else if (135 >= angDeg && angDeg > 45)
    {
      SwerveModuleConfiguration conf = getSwerveModule(config.modules, true, true);
      projectedWheelbaseEdge =
          new Translation2d(
              conf.moduleLocation.getY() / angle.getTan(), conf.moduleLocation.getY());
    } else if (-135 <= angDeg && angDeg < -45)
    {
      SwerveModuleConfiguration conf = getSwerveModule(config.modules, true, false);
      projectedWheelbaseEdge =
          new Translation2d(
              conf.moduleLocation.getY() / angle.getTan(), conf.moduleLocation.getY());
    } else
    {
      SwerveModuleConfiguration conf = getSwerveModule(config.modules, false, true);
      projectedWheelbaseEdge =
          new Translation2d(
              conf.moduleLocation.getX(), conf.moduleLocation.getX() * angle.getTan());
    }

    double horizontalDistance = projectedHorizontalCg.plus(projectedWheelbaseEdge).getNorm();
    double maxAccel           = 9.81 * horizontalDistance / robotCG.getZ();

    SmartDashboard.putNumber("calcMaxAccel", maxAccel);
    return maxAccel;
  }

  /**
   * Limits a commanded velocity to prevent exceeding the maximum acceleration given by
   * {@link SwerveMath#calcMaxAccel(Rotation2d, double, double, Translation3d, SwerveDriveConfiguration)}. Note that
   * this takes and returns field-relative velocities.
   *
   * @param commandedVelocity      The desired velocity
   * @param fieldVelocity          The velocity of the robot within a field relative state.
   * @param robotPose              The current pose of the robot.
   * @param loopTime               The time it takes to update the velocity in seconds. <b>Note: this should include the
   *                               100ms that it takes for a SparkMax velocity to update.</b>
   * @param chassisMass            Chassis mass in kg. (The weight of just the chassis not anything else)
   * @param robotMass              The weight of the robot in kg. (Including manipulators, etc).
   * @param chassisCenterOfGravity Chassis center of gravity.
   * @param config                 The swerve drive configuration.
   * @return The limited velocity. This is either the commanded velocity, if attainable, or the closest attainable
   * velocity.
   */
  public static Translation2d limitVelocity(
      Translation2d commandedVelocity,
      ChassisSpeeds fieldVelocity,
      Pose2d robotPose,
      double loopTime,
      double chassisMass,
      double robotMass,
      Translation3d chassisCenterOfGravity,
      SwerveDriveConfiguration config)
  {
    // Get the robot's current field-relative velocity
    Translation2d currentVelocity = SwerveController.getTranslation2d(fieldVelocity);
    SmartDashboard.putNumber("currentVelocity", currentVelocity.getX());

    // Calculate the commanded change in velocity by subtracting current velocity
    // from commanded velocity
    Translation2d deltaV = commandedVelocity.minus(currentVelocity);
    SmartDashboard.putNumber("deltaV", deltaV.getX());

    // Creates an acceleration vector with the direction of delta V and a magnitude
    // of the maximum allowed acceleration in that direction
    Translation2d maxAccel =
        new Translation2d(
            calcMaxAccel(
                deltaV
                    // Rotates the velocity vector to convert from field-relative to robot-relative
                    .rotateBy(robotPose.getRotation().unaryMinus())
                    .getAngle(),
                chassisMass,
                robotMass,
                chassisCenterOfGravity,
                config),
            deltaV.getAngle());

    // Calculate the maximum achievable velocity by the next loop cycle.
    // delta V = Vf - Vi = at
    Translation2d maxAchievableDeltaVelocity = maxAccel.times(loopTime);

    if (deltaV.getNorm() > maxAchievableDeltaVelocity.getNorm())
    {
      return maxAchievableDeltaVelocity.plus(currentVelocity);
    } else
    {
      // If the commanded velocity is attainable, use that.
      return commandedVelocity;
    }
  }

  /**
   * Get the fruthest module from center based on the module locations.
   *
   * @param modules Swerve module list.
   * @param front   True = furthest front, False = furthest back.
   * @param left    True = furthest left, False = furthest right.
   * @return Module location which is the furthest from center and abides by parameters.
   */
  public static SwerveModuleConfiguration getSwerveModule(
      SwerveModule[] modules, boolean front, boolean left)
  {
    Translation2d             target        = modules[0].configuration.moduleLocation, current, temp;
    SwerveModuleConfiguration configuration = modules[0].configuration;
    for (SwerveModule module : modules)
    {
      current = module.configuration.moduleLocation;
      temp =
          front
          ? (target.getY() >= current.getY() ? current : target)
          : (target.getY() <= current.getY() ? current : target);
      target =
          left
          ? (target.getX() >= temp.getX() ? temp : target)
          : (target.getX() <= temp.getX() ? temp : target);
      configuration = current.equals(target) ? module.configuration : configuration;
    }
    return configuration;
  }
}
