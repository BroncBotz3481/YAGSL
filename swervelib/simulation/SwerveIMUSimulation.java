package swervelib.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Optional;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * Simulation for {@link swervelib.SwerveDrive} IMU.
 */
public class SwerveIMUSimulation
{

  private final GyroSimulation gyroSimulation;

  /**
   * Create the swerve drive IMU simulation.
   *
   * @param gyroSimulation Gyro simulation from MapleSim.
   */
  public SwerveIMUSimulation(GyroSimulation gyroSimulation)
  {
    this.gyroSimulation = gyroSimulation;
  }

  /**
   * Get the estimated angle of the robot.
   *
   * @return {@link Rotation2d} estimation of the robot.
   */
  public Rotation2d getYaw()
  {
    return gyroSimulation.getGyroReading();
  }

  /**
   * Pitch is not simulated currently, always returns 0.
   *
   * @return Pitch of the robot as {@link Rotation2d}.
   */
  public Rotation2d getPitch()
  {
    return new Rotation2d();
  }

  /**
   * Roll is not simulated currently, always returns 0.
   *
   * @return Roll of the robot as {@link Rotation2d}.
   */
  public Rotation2d getRoll()
  {
    return new Rotation2d();
  }

  /**
   * Gets the estimated gyro {@link Rotation3d} of the robot.
   *
   * @return The heading as a {@link Rotation3d} angle
   */
  public Rotation3d getGyroRotation3d()
  {
    return new Rotation3d(0, 0, getYaw().getRadians());
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in m/s/s. If acceleration isn't supported returns empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  public Optional<Translation3d> getAccel()
  {
    return Optional.empty();
  }

  /**
   * Update the odometry of the simulated {@link swervelib.SwerveDrive} and post the {@link swervelib.SwerveModule}
   * states to the {@link Field2d}.
   *
   * @param kinematics  {@link SwerveDriveKinematics} of the swerve drive.
   * @param states      {@link SwerveModuleState} array of the module states.
   * @param modulePoses {@link Pose2d} representing the swerve modules.
   * @param field       {@link Field2d} to update.
   */
  public void updateOdometry(
      SwerveDriveKinematics kinematics,
      SwerveModuleState[] states,
      Pose2d[] modulePoses,
      Field2d field)
  {
    field.getObject("XModules").setPoses(modulePoses);
  }

  /**
   * Set the heading of the robot.
   *
   * @param angle Angle of the robot in radians.
   */
  public void setAngle(double angle)
  {
    this.gyroSimulation.setRotation(Rotation2d.fromRadians(angle));
  }
}
