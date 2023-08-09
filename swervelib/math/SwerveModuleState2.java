package swervelib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
    SwerveModuleState2 optimized = new SwerveModuleState2(SwerveModuleState.optimize(desiredState, currentAngle),
                                                          desiredState.omegaRadPerSecond);
    if (desiredState.angle.equals(currentAngle) || desiredState.angle.equals(
        optimized.angle.rotateBy(Rotation2d.fromDegrees(180))) || moduleSteerFeedForwardClosedLoop == 0)
    {
      optimized.omegaRadPerSecond = 0;
    }
    if (desiredState.angle.equals(optimized.angle.rotateBy(Rotation2d.fromDegrees(180))))
    {
      desiredState.omegaRadPerSecond = 0;
      return desiredState;
    }
    return optimized;
    /*
    // NEVER optimize if it's the same angle, it just doesn't make sense...
    if (currentAngle.equals(desiredState.angle.rotateBy(Rotation2d.fromDegrees(180))))
    {
      desiredState.invert();
      desiredState.omegaRadPerSecond = 0;
      return desiredState;
    } else if (currentAngle.equals(desiredState.angle))
    {
      desiredState.omegaRadPerSecond = 0;
      return desiredState;
    }

    SwerveModuleState2 optimized;
    if (moduleSteerFeedForwardClosedLoop == 0)
    {
//    desiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(
//        lastState.omegaRadPerSecond * moduleSteerFeedForwardClosedLoop * 0.065));
//      return new SwerveModuleState2(SwerveModuleState.optimize(desiredState, currentAngle),
//                                    desiredState.omegaRadPerSecond);
      optimized = new SwerveModuleState2(SwerveModuleState.optimize(desiredState, currentAngle), 0);
//      return desiredState;
    } else
    {
      Rotation2d delta = desiredState.angle.plus(Rotation2d.fromRadians(lastState.omegaRadPerSecond *
                                                                        moduleSteerFeedForwardClosedLoop *
                                                                        0.065))
                                           .minus(currentAngle);
      if (Double.compare(Math.abs(delta.getDegrees()), 90.0) < 0)
      {
        optimized = desiredState.invert();
        optimized.angle = optimized.angle.plus(Rotation2d.fromRadians(lastState.omegaRadPerSecond *
                                                                      moduleSteerFeedForwardClosedLoop *
                                                                      0.065));
      } else
      {
        optimized = desiredState;
      }
    }

    if (Double.compare(Math.abs(currentAngle.minus(optimized.angle).getDegrees()),
                       Math.abs(currentAngle.minus(desiredState.angle)
                                            .getDegrees())) > 0)
    {
      desiredState.omegaRadPerSecond = 0;
      return desiredState;
    }

    // Maybe the omega rad per second will always be off when optimized?
//    optimized.omegaRadPerSecond = 0;
    return optimized;
     */
  }

  /**
   * Invert the swerve module state.
   *
   * @return Current inverted {@link SwerveModuleState2}
   */
  public SwerveModuleState2 invert()
  {
//    omegaRadPerSecond *= -1;
//    speedMetersPerSecond *= -1;
//    angle = angle.rotateBy(Rotation2d.fromDegrees(180));
//    return this;
    return new SwerveModuleState2(-speedMetersPerSecond,
                                  angle.rotateBy(Rotation2d.fromDegrees(180)),
                                  -omegaRadPerSecond);
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
