package swervelib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import java.util.Arrays;
import java.util.Map;
import java.util.Objects;

/**
 * Clone of {@link SwerveDrivePoseEstimator} which takes into account gyroscope pitch and roll to achieve a more
 * accurate estimation, originally made by Team 1466.
 */
public class SwervePoseEstimator2 // extends SwerveDrivePoseEstimator
{

  /**
   * Swerve drive kinematics.
   */
  private final SwerveDriveKinematics                         m_kinematics;
  /**
   * Enhanced swerve drive odometry.
   */
  private final SwerveDriveOdometry2                          m_odometry;
  /**
   * Matrix quotient.
   */
  private final Matrix<N3, N1>                                m_q          = new Matrix<>(Nat.N3(), Nat.N1());
  /**
   * Number of swerve modules.
   */
  private final int                                           m_numModules;
  /**
   * Interpolation buffer.
   */
  private final TimeInterpolatableBuffer<InterpolationRecord> m_poseBuffer = TimeInterpolatableBuffer.createBuffer(1.5);
  /**
   * Vision standard deviations.
   */
  private final Matrix<N3, N3>                                m_visionK    = new Matrix<>(Nat.N3(), Nat.N3());

  /**
   * Constructs a SwerveDrivePoseEstimator with default standard deviations for the model and vision measurements.
   *
   * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for y,
   * and 0.1 radians for heading. The default standard deviations of the vision measurements are 0.9 meters for x, 0.9
   * meters for y, and 0.9 radians for heading.
   *
   * @param kinematics        A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle         The current gyro angle.
   * @param modulePositions   The current distance measurements and rotations of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   */
  public SwervePoseEstimator2(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters)
  {
    this(
        kinematics,
        gyroAngle,
        modulePositions,
        initialPoseMeters,
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.9, 0.9, 0.9));
  }

  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param kinematics               A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle                The current gyro angle.
   * @param modulePositions          The current distance and rotation measurements of the swerve modules.
   * @param initialPoseMeters        The starting pose estimate.
   * @param stateStdDevs             Standard deviations of the pose estimate (x position in meters, y position in
   *                                 meters, and heading in radians). Increase these numbers to trust your state
   *                                 estimate less.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position in meters, y
   *                                 position in meters, and heading in radians). Increase these numbers to trust the
   *                                 vision pose measurement less.
   */
  public SwervePoseEstimator2(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs)
  {
//    super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    m_kinematics = kinematics;
    m_odometry =
        new SwerveDriveOdometry2(kinematics, gyroAngle, modulePositions, initialPoseMeters);

    for (int i = 0; i < 3; ++i)
    {
      m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }

    m_numModules = modulePositions.length;

    setVisionMeasurementStdDevs(visionMeasurementStdDevs);
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in vision measurements
   * after the autonomous period, or to change trust as distance to a vision target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these numbers to trust
   *                                 global measurements from vision less. This matrix is in the form [x, y, theta]^T,
   *                                 with units in meters and radians.
   */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs)
  {
//    super.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    var r = new double[3];
    for (int i = 0; i < 3; ++i)
    {
      r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    for (int row = 0; row < 3; ++row)
    {
      if (m_q.get(row, 0) == 0.0)
      {
        m_visionK.set(row, row, 0.0);
      } else
      {
        m_visionK.set(
            row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
      }
    }
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle       The angle reported by the gyroscope.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param poseMeters      The position on the field that your robot is at.
   */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters)
  {
//    super.resetPosition(gyroAngle, modulePositions, poseMeters);
    // Reset state estimate and error covariance
    m_odometry.resetPosition(gyroAngle, modulePositions, poseMeters);
    m_poseBuffer.clear();
  }

  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition()
  {
    return m_odometry.getPoseMeters();
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting
   * for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the current pose estimate.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds      The timestamp of the vision measurement in seconds. Note that if you don't use your
   *                              own time source by calling
   *                              {@link SwerveDrivePoseEstimator#updateWithTime(double, Rotation2d,
   *                              SwerveModulePosition[])} then you must use a timestamp with an epoch since FPGA
   *                              startup (i.e., the epoch of this timestamp is the same epoch as
   *                              {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.) This means that you should
   *                              use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source or sync
   *                              the epochs.
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds)
  {
//    super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    // Step 1: Get the pose odometry measured at the moment the vision measurement was made.
    var sample = m_poseBuffer.getSample(timestampSeconds);

    if (sample.isEmpty())
    {
      return;
    }

    // Step 2: Measure the twist between the odometry pose and the vision pose.
    var twist = sample.get().poseMeters.log(visionRobotPoseMeters);

    // Step 3: We should not trust the twist entirely, so instead we scale this twist by a Kalman
    // gain matrix representing how much we trust vision measurements compared to our current pose.
    var k_times_twist = m_visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

    // Step 4: Convert back to Twist2d.
    var scaledTwist =
        new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));

    // Step 5: Reset Odometry to state at sample with vision adjustment.
    m_odometry.resetPosition(
        sample.get().gyroAngle,
        sample.get().modulePositions,
        sample.get().poseMeters.exp(scaledTwist));

    // Step 6: Replay odometry inputs between sample time and latest recorded sample to update the
    // pose buffer and correct odometry.
    for (Map.Entry<Double, InterpolationRecord> entry :
        m_poseBuffer.getInternalBuffer().tailMap(timestampSeconds).entrySet())
    {
      updateWithTime(
          entry.getKey(),
          entry.getValue().gyroAngle,
          entry.getValue().gyroPitch,
          entry.getValue().gyroRoll,
          entry.getValue().modulePositions);
    }
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting
   * for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to
   * {@link SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters    The pose of the robot as measured by the vision camera.
   * @param timestampSeconds         The timestamp of the vision measurement in seconds. Note that if you don't use your
   *                                 own time source by calling
   *                                 {@link SwerveDrivePoseEstimator#updateWithTime(double, Rotation2d,
   *                                 SwerveModulePosition[])}, then you must use a timestamp with an epoch since FPGA
   *                                 startup (i.e., the epoch of this timestamp is the same epoch as
   *                                 {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}). This means that you should
   *                                 use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source in
   *                                 this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position in meters, y
   *                                 position in meters, and heading in radians). Increase these numbers to trust the
   *                                 vision pose measurement less.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs)
  {
    setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every loop.
   *
   * @param gyroAngle       The current gyro angle.
   * @param gyroPitch       The current gyro pitch.
   * @param gyroRoll        The current gyro roll.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d update(
      Rotation2d gyroAngle,
      Rotation2d gyroPitch,
      Rotation2d gyroRoll,
      SwerveModulePosition[] modulePositions)
  {
    return updateWithTime(
        WPIUtilJNI.now() * 1.0e-6, gyroAngle, gyroPitch, gyroRoll, modulePositions);
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every loop.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle          The current gyro angle.
   * @param gyroPitch          The current gyro pitch.
   * @param gyroRoll           The current gyro roll.
   * @param modulePositions    The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d updateWithTime(
      double currentTimeSeconds,
      Rotation2d gyroAngle,
      Rotation2d gyroPitch,
      Rotation2d gyroRoll,
      SwerveModulePosition[] modulePositions)
  {
//    super.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
    if (modulePositions.length != m_numModules)
    {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
          + "constructor");
    }

    var internalModulePositions = new SwerveModulePosition[m_numModules];

    for (int i = 0; i < m_numModules; i++)
    {
      internalModulePositions[i] =
          new SwerveModulePosition(modulePositions[i].distanceMeters, modulePositions[i].angle);
    }

    m_odometry.update(gyroAngle, gyroPitch, gyroRoll, internalModulePositions);

    m_poseBuffer.addSample(
        currentTimeSeconds,
        new InterpolationRecord(
            getEstimatedPosition(), gyroAngle, gyroPitch, gyroRoll, internalModulePositions));

    return getEstimatedPosition();
  }

  /**
   * Represents an odometry record. The record contains the inputs provided as well as the pose that was observed based
   * on these inputs, as well as the previous record and its inputs.
   */
  private class InterpolationRecord implements Interpolatable<InterpolationRecord>
  {

    // The pose observed given the current sensor inputs and the previous pose.
    private final Pose2d poseMeters;

    // The current gyro angle.
    private final Rotation2d gyroAngle;

    // The current gyro pitch.
    private final Rotation2d gyroPitch;

    // The current gyro roll.
    private final Rotation2d gyroRoll;

    // The distances and rotations measured at each module.
    private final SwerveModulePosition[] modulePositions;

    /**
     * Constructs an Interpolation Record with the specified parameters.
     *
     * @param poseMeters      The pose observed given the current sensor inputs and the previous pose.
     * @param gyro            The current gyro angle.
     * @param gyroPitch       The current gyro pitch.
     * @param gyroRoll        The current gyro roll.
     * @param modulePositions The distances and rotations measured at each wheel.
     */
    private InterpolationRecord(
        Pose2d poseMeters,
        Rotation2d gyro,
        Rotation2d gyroPitch,
        Rotation2d gyroRoll,
        SwerveModulePosition[] modulePositions)
    {
      this.poseMeters = poseMeters;
      this.gyroAngle = gyro;
      this.gyroPitch = gyroPitch;
      this.gyroRoll = gyroRoll;
      this.modulePositions = modulePositions;
    }

    /**
     * Return the interpolated record. This object is assumed to be the starting position, or lower bound.
     *
     * @param endValue The upper bound, or end.
     * @param t        How far between the lower and upper bound we are. This should be bounded in [0, 1].
     * @return The interpolated value.
     */
    @Override
    public InterpolationRecord interpolate(InterpolationRecord endValue, double t)
    {
      if (t < 0)
      {
        return this;
      } else if (t >= 1)
      {
        return endValue;
      } else
      {
        // Find the new wheel distances.
        var modulePositions = new SwerveModulePosition[m_numModules];

        // Find the distance travelled between this measurement and the interpolated measurement.
        var moduleDeltas = new SwerveModulePosition[m_numModules];

        for (int i = 0; i < m_numModules; i++)
        {
          double ds =
              MathUtil.interpolate(
                  this.modulePositions[i].distanceMeters,
                  endValue.modulePositions[i].distanceMeters,
                  t);
          Rotation2d theta =
              this.modulePositions[i].angle.interpolate(endValue.modulePositions[i].angle, t);
          modulePositions[i] = new SwerveModulePosition(ds, theta);
          moduleDeltas[i] =
              new SwerveModulePosition(ds - this.modulePositions[i].distanceMeters, theta);
        }

        // Find the new gyro angle.
        var gyro_lerp      = gyroAngle.interpolate(endValue.gyroAngle, t);
        var gyroPitch_lerp = gyroPitch.interpolate(endValue.gyroPitch, t);
        var gyroRoll_lerp  = gyroRoll.interpolate(endValue.gyroRoll, t);

        // Create a twist to represent this change based on the interpolated sensor inputs.
        Twist2d twist = m_kinematics.toTwist2d(moduleDeltas);
        twist.dtheta = gyro_lerp.minus(gyroAngle).getRadians();

        return new InterpolationRecord(
            poseMeters.exp(twist), gyro_lerp, gyroPitch_lerp, gyroRoll_lerp, modulePositions);
      }
    }

    @Override
    public boolean equals(Object obj)
    {
      if (this == obj)
      {
        return true;
      }
      if (!(obj instanceof InterpolationRecord))
      {
        return false;
      }
      InterpolationRecord record = (InterpolationRecord) obj;
      return Objects.equals(gyroAngle, record.gyroAngle)
             && Arrays.equals(modulePositions, record.modulePositions)
             && Objects.equals(poseMeters, record.poseMeters);
    }

    @Override
    public int hashCode()
    {
      return Objects.hash(gyroAngle, Arrays.hashCode(modulePositions), poseMeters);
    }
  }
}
