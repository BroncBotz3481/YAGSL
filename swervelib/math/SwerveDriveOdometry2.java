package swervelib.math;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ejml.simple.SimpleMatrix;

/**
 * Clone of {@link SwerveDriveOdometry} except uses gyro pitch and roll to achieve a more accurate estimation.
 * Originally made by Team 1466.
 */
public class SwerveDriveOdometry2 extends SwerveDriveOdometry
{

  /**
   * Swerve drive kinematics.
   */
  private final SwerveDriveKinematics  m_kinematics;
  /**
   * Number of swerve modules.
   */
  private final int                    m_numModules;
  /**
   * Previous swerve module positions.
   */
  private final SwerveModulePosition[] m_previousModulePositions;
  /**
   * Zero module states.
   */
  private final SwerveModuleState[]    m_zeroModuleStates;
  /**
   * Estimated pose.
   */
  private       Pose2d                 m_poseMeters;
  /**
   * Gyro offset.
   */
  private       Rotation2d             m_gyroOffset;
  /**
   * Previous gyroscope angle.
   */
  private       Rotation2d             m_previousAngle;

  /**
   * Constructs a SwerveDriveOdometry object.
   *
   * @param kinematics      The swerve drive kinematics for your drivetrain.
   * @param gyroAngle       The angle reported by the gyroscope.
   * @param modulePositions The wheel positions reported by each module.
   * @param initialPose     The starting position of the robot on the field.
   */
  public SwerveDriveOdometry2(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPose)
  {
    super(kinematics, gyroAngle, modulePositions, initialPose);
    m_kinematics = kinematics;
    m_poseMeters = initialPose;
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = initialPose.getRotation();
    m_numModules = modulePositions.length;

    m_previousModulePositions = new SwerveModulePosition[m_numModules];
    for (int index = 0; index < m_numModules; index++)
    {
      m_previousModulePositions[index] =
          new SwerveModulePosition(
              modulePositions[index].distanceMeters, modulePositions[index].angle);
    }
    m_zeroModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(1, 0, 0));

    MathSharedStore.reportUsage(MathUsageId.kOdometry_SwerveDrive, 1);
  }

  /**
   * Constructs a SwerveDriveOdometry object with the default pose at the origin.
   *
   * @param kinematics      The swerve drive kinematics for your drivetrain.
   * @param gyroAngle       The angle reported by the gyroscope.
   * @param modulePositions The wheel positions reported by each module.
   */
  public SwerveDriveOdometry2(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions)
  {
    this(kinematics, gyroAngle, modulePositions, new Pose2d());
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * <p>Similarly, module positions do not need to be reset in user code.
   *
   * @param gyroAngle       The angle reported by the gyroscope.
   * @param modulePositions The wheel positions reported by each module.,
   * @param pose            The position on the field that your robot is at.
   */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose)
  {
    super.resetPosition(gyroAngle, modulePositions, pose);
    if (modulePositions.length != m_numModules)
    {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
          + "constructor");
    }

    m_poseMeters = pose;
    m_previousAngle = pose.getRotation();
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    for (int index = 0; index < m_numModules; index++)
    {
      m_previousModulePositions[index] =
          new SwerveModulePosition(
              modulePositions[index].distanceMeters, modulePositions[index].angle);
    }
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose2d getPoseMeters()
  {
    return m_poseMeters;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose over time. This
   * method automatically calculates the current time to calculate period (difference between two timestamps). The
   * period is used to calculate the change in distance from a velocity. This also takes in an angle parameter which is
   * used instead of the angular rate that is calculated from forward kinematics. This also takes in pitch and roll to
   * allow for more accurate pose estimation on angled surfaces using a rotation matrix.
   *
   * @param gyroYaw         The yaw reported by the gyro.
   * @param pitch           The pitch reported by the gyro.
   * @param roll            The roll reported by the gyro.
   * @param modulePositions The current position of all swerve modules. Please provide the positions in the same order
   *                        in which you instantiated your SwerveDriveKinematics.
   * @return The new pose of the robot.
   */
  public Pose2d update(
      Rotation2d gyroYaw,
      Rotation2d pitch,
      Rotation2d roll,
      SwerveModulePosition[] modulePositions)
  {
    super.update(gyroYaw, modulePositions);
    if (modulePositions.length != m_numModules)
    {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
          + "constructor");
    }

    var moduleDeltas = new SwerveModulePosition[m_numModules];
    var yaw          = gyroYaw.plus(m_gyroOffset);

    var rotationMatrix = new SimpleMatrix(3, 3);
    var gyroZero       = new Rotation2d(); // doing dist calcs robot relative
    rotationMatrix.setRow(
        0,
        0,
        gyroZero.getCos() * pitch.getCos(),
        gyroZero.getCos() * pitch.getSin() * roll.getSin() - gyroZero.getSin() * roll.getCos(),
        gyroZero.getCos() * pitch.getSin() * roll.getCos() + gyroZero.getSin() * roll.getSin());

    rotationMatrix.setRow(
        1,
        0,
        gyroZero.getSin() * pitch.getCos(),
        gyroZero.getSin() * pitch.getSin() * roll.getSin() + gyroZero.getCos() * roll.getCos(),
        gyroZero.getSin() * pitch.getSin() * roll.getCos() - gyroZero.getCos() * roll.getSin());

    rotationMatrix.setRow(
        2, 0, -pitch.getSin(), pitch.getCos() * roll.getSin(), pitch.getCos() * roll.getCos());

    for (int index = 0; index < m_numModules; index++)
    {
      var current  = modulePositions[index];
      var previous = m_previousModulePositions[index];

      var deltaDistanceInitial = current.distanceMeters - previous.distanceMeters;
      var changedAngle =
          current.angle.minus(
              m_zeroModuleStates[index]
                  .angle); // does this return (-pi, pi)? shoudln't matter since Twist2d does sin
      // cos

      var deltaMatrix = new SimpleMatrix(3, 1);
      deltaMatrix.setColumn(
          0,
          0,
          changedAngle.getCos() * deltaDistanceInitial,
          changedAngle.getSin() * deltaDistanceInitial,
          0);

      var rotatedDeltaMatrix = rotationMatrix.mult(deltaMatrix);
      var finalDeltaDistance =
          deltaDistanceInitial >= 0
          ? Math.sqrt(
              Math.pow(rotatedDeltaMatrix.get(0, 0), 2)
              + Math.pow(rotatedDeltaMatrix.get(1, 0), 2))
          : -Math.sqrt(
              Math.pow(rotatedDeltaMatrix.get(0, 0), 2)
              + Math.pow(rotatedDeltaMatrix.get(1, 0), 2));

      var deltaDistance =
          (roll.getDegrees() > 10 || pitch.getDegrees() > 10)
          ? finalDeltaDistance
          : deltaDistanceInitial;
      var updatedAngle =
          (roll.getDegrees() > 10 || pitch.getDegrees() > 10)
          ? Rotation2d.fromRadians(
              Math.atan2(rotatedDeltaMatrix.get(0, 0), rotatedDeltaMatrix.get(1, 0)))
          : current.angle;

      moduleDeltas[index] = new SwerveModulePosition(deltaDistance, updatedAngle);
      previous.distanceMeters = current.distanceMeters;
    }

    var twist = m_kinematics.toTwist2d(moduleDeltas);
    twist.dtheta = yaw.minus(m_previousAngle).getRadians();

    var newPose = m_poseMeters.exp(twist);

    m_previousAngle = yaw;
    m_poseMeters = new Pose2d(newPose.getTranslation(), yaw);

    return m_poseMeters;
  }
}
