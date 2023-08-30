package swervelib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import swervelib.SwerveController;
import swervelib.SwerveModule;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Mathematical functions which pertain to swerve drive.
 */
public class SwerveMath
{

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
    return maxSpeed / new Rotation2d(furthestModuleX, furthestModuleY).getRadians();
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
   * @param angle     The direction in which to calculate max acceleration, as a Rotation2d. Note that this is
   *                  robot-relative.
   * @param matter    Matter that the robot is composed of in kg. (Includes chassis)
   * @param robotMass The weight of the robot in kg. (Including manipulators, etc).
   * @param config    The swerve drive configuration.
   * @return Maximum acceleration allowed in the robot direction.
   */
  private static double calcMaxAccel(
      Rotation2d angle,
      List<Matter> matter,
      double robotMass,
      SwerveDriveConfiguration config)
  {
    // Calculate the vertical mass moment using the floor as the datum.  This will be used later to
    // calculate max acceleration
    Translation3d centerMass = new Translation3d();
    for (Matter object : matter)
    {
      centerMass = centerMass.plus(object.massMoment());
    }
    Translation3d robotCG      = centerMass.div(robotMass);
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
    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
    {
      SmartDashboard.putNumber("calcMaxAccel", maxAccel);
    }
    return maxAccel;
  }

  /**
   * Logical inverse of the Pose exponential from 254. Taken from team 3181.
   *
   * @param transform Pose to perform the log on.
   * @return {@link Twist2d} of the transformed pose.
   */
  public static Twist2d PoseLog(final Pose2d transform)
  {

    final double kEps          = 1E-9;
    final double dtheta        = transform.getRotation().getRadians();
    final double half_dtheta   = 0.5 * dtheta;
    final double cos_minus_one = transform.getRotation().getCos() - 1.0;
    double       halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps)
    {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else
    {
      halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
    }
    final Translation2d translation_part = transform.getTranslation()
                                                    .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta,
                                                                             -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  /**
   * Limits a commanded velocity to prevent exceeding the maximum acceleration given by {@link SwerveMath#calcMaxAccel}.
   * Note that this takes and returns field-relative velocities.
   *
   * @param commandedVelocity The desired velocity
   * @param fieldVelocity     The velocity of the robot within a field relative state.
   * @param robotPose         The current pose of the robot.
   * @param loopTime          The time it takes to update the velocity in seconds. <b>Note: this should include the
   *                          100ms that it takes for a SparkMax velocity to update.</b>
   * @param matter            Matter that the robot is composed of with position in meters and mass in kg.
   * @param robotMass         The weight of the robot in kg. (Including manipulators, etc).
   * @param config            The swerve drive configuration.
   * @return The limited velocity. This is either the commanded velocity, if attainable, or the closest attainable
   * velocity.
   */
  public static Translation2d limitVelocity(
      Translation2d commandedVelocity,
      ChassisSpeeds fieldVelocity,
      Pose2d robotPose,
      double loopTime,
      double robotMass,
      List<Matter> matter,
      SwerveDriveConfiguration config)
  {
    // Get the robot's current field-relative velocity
    Translation2d currentVelocity = SwerveController.getTranslation2d(fieldVelocity);
    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
    {
      SmartDashboard.putNumber("currentVelocity", currentVelocity.getX());
    }

    // Calculate the commanded change in velocity by subtracting current velocity
    // from commanded velocity
    Translation2d deltaV = commandedVelocity.minus(currentVelocity);
    if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
    {
      SmartDashboard.putNumber("deltaV", deltaV.getX());
    }

    // Creates an acceleration vector with the direction of delta V and a magnitude
    // of the maximum allowed acceleration in that direction
    Translation2d maxAccel =
        new Translation2d(
            calcMaxAccel(
                deltaV
                    // Rotates the velocity vector to convert from field-relative to robot-relative
                    .rotateBy(robotPose.getRotation().unaryMinus())
                    .getAngle(),
                matter,
                robotMass,
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

  /**
   * Put an angle within the 360 deg scope of a reference. For example, given a scope reference of 756 degrees, assumes
   * the full scope is (720-1080), and places an angle of 22 degrees into it, returning 742 deg.
   *
   * @param scopeReference Current Angle (deg)
   * @param newAngle       Target Angle (deg)
   * @return Closest angle within scope (deg)
   */
  public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle)
  {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0)
    {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else
    {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound)
    {
      newAngle += 360;
    }
    while (newAngle > upperBound)
    {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180)
    {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180)
    {
      newAngle += 360;
    }
    return newAngle;
  }

  /**
   * Perform anti-jitter within modules if the speed requested is too low.
   *
   * @param moduleState     Current {@link SwerveModuleState} requested.
   * @param lastModuleState Previous {@link SwerveModuleState} used.
   * @param maxSpeed        Maximum speed of the modules, should be in {@link SwerveDriveConfiguration#maxSpeed}.
   */
  public static void antiJitter(SwerveModuleState moduleState, SwerveModuleState lastModuleState, double maxSpeed)
  {
    if (Math.abs(moduleState.speedMetersPerSecond) <= (maxSpeed * 0.01))
    {
      moduleState.angle = lastModuleState.angle;
    }
  }
}
