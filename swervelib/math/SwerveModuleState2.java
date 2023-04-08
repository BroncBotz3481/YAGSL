package swervelib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/**
 * Second order kinematics swerve module state.
 */
public class SwerveModuleState2 extends SwerveModuleState
{

  /**
   * Rad per sec
   */
  public double omegaRadPerSecond = 0;

  /**
   * Constructs a SwerveModuleState with zeros for speed and angle.
   */
  public SwerveModuleState2()
  {
    super();
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle                The angle of the module.
   * @param omegaRadPerSecond    The angular velocity of the module.
   */
  public SwerveModuleState2(
      double speedMetersPerSecond, Rotation2d angle, double omegaRadPerSecond)
  {
    super(speedMetersPerSecond, angle);
    this.omegaRadPerSecond = omegaRadPerSecond;
  }

  /**
   * Create a {@link SwerveModuleState2} based on the {@link SwerveModuleState} with the radians per second defined.
   *
   * @param state             First order kinematic module state.
   * @param omegaRadPerSecond Module wheel angular rotation in radians per second.
   */
  public SwerveModuleState2(SwerveModuleState state, double omegaRadPerSecond)
  {
    super(state.speedMetersPerSecond, state.angle);
    this.omegaRadPerSecond = omegaRadPerSecond;
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially reversing the direction
   * the wheel spins. If this is used with the PIDController class's continuous input functionality, the furthest a
   * wheel will ever rotate is 90 degrees.
   *
   * @param desiredState                     The desired state.
   * @param currentAngle                     The current module angle.
   * @param lastState                        The last state of the module.
   * @param moduleSteerFeedForwardClosedLoop The module feed forward closed loop for the angle motor.
   * @return Optimized swerve module state.
   */
  public static SwerveModuleState2 optimize(SwerveModuleState2 desiredState, Rotation2d currentAngle,
                                            SwerveModuleState2 lastState, double moduleSteerFeedForwardClosedLoop)
  {
    if (moduleSteerFeedForwardClosedLoop == 0)
    {
//    desiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(
//        lastState.omegaRadPerSecond * moduleSteerFeedForwardClosedLoop * 0.065));
//      return new SwerveModuleState2(SwerveModuleState.optimize(desiredState, currentAngle),
//                                    desiredState.omegaRadPerSecond);
      return new SwerveModuleState2(SwerveModuleState.optimize(desiredState, currentAngle), 0);
//      return desiredState;
    } else
    {
      double targetAngle = SwerveMath.placeInAppropriate0To360Scope(currentAngle.getDegrees(),
                                                                    desiredState.angle.getDegrees() +
                                                                    Units.radiansToDegrees(lastState.omegaRadPerSecond *
                                                                                           moduleSteerFeedForwardClosedLoop *
                                                                                           0.065));
      double targetSpeed = desiredState.speedMetersPerSecond;
      double delta       = targetAngle - currentAngle.getDegrees();
      if (Math.abs(delta) > 90)
      {
        targetSpeed = -targetSpeed;
        if (delta > 90)
        {
          targetAngle -= 180;
        } else
        {
          targetAngle += 180;
        }
      }
      return new SwerveModuleState2(targetSpeed, Rotation2d.fromDegrees(targetAngle), desiredState.omegaRadPerSecond);
    }
  }

  /**
   * Convert to a {@link SwerveModuleState}.
   *
   * @return {@link SwerveModuleState} with the same angle and speed.
   */
  public SwerveModuleState toSwerveModuleState()
  {
    return new SwerveModuleState(this.speedMetersPerSecond, this.angle);
  }
}
