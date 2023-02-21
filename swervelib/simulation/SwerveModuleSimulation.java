package swervelib.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import swervelib.math.SwerveModuleState2;

/**
 * Class to hold simulation data for {@link swervelib.SwerveModule}
 */
public class SwerveModuleSimulation
{

  /**
   * Main timer to simulate the passage of time.
   */
  private final Timer              timer;
  /**
   * Time delta since last update
   */
  private       double             dt;
  /**
   * Fake motor position.
   */
  private       double             fakePos;
  /**
   * The fake speed of the previous state, used to calculate {@link SwerveModuleSimulation#fakePos}.
   */
  private       double             fakeSpeed;
  /**
   * Last time queried.
   */
  private       double             lastTime;
  /**
   * Current simulated swerve module state.
   */
  private       SwerveModuleState2 state;

  /**
   * Create simulation class and initialize module at 0.
   */
  public SwerveModuleSimulation()
  {
    timer = new Timer();
    timer.start();
    lastTime = timer.get();
    state = new SwerveModuleState2(0, Rotation2d.fromDegrees(0), 0);
    fakeSpeed = 0;
    fakePos = 0;
    dt = 0;
  }

  /**
   * Update the position and state of the module. Called from {@link swervelib.SwerveModule#setDesiredState} function
   * when simulated.
   *
   * @param desiredState State the swerve module is set to.
   */
  public void updateStateAndPosition(SwerveModuleState2 desiredState)
  {
    dt = timer.get() - lastTime;
    fakePos += (fakeSpeed * dt);
    lastTime = timer.get();

    state = desiredState;
    fakeSpeed = desiredState.speedMetersPerSecond;
  }

  /**
   * Get the simulated swerve module position.
   *
   * @return {@link SwerveModulePosition} of the simulated module.
   */
  public SwerveModulePosition getPosition()
  {
    return new SwerveModulePosition(
        fakePos, state.angle.plus(new Rotation2d(state.omegaRadPerSecond * dt)));
  }

  /**
   * Get the {@link SwerveModuleState2} of the simulated module.
   *
   * @return {@link SwerveModuleState2} of the simulated module.
   */
  public SwerveModuleState2 getState()
  {
    return state;
  }
}
