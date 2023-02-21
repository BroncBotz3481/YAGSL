package swervelib.simulation.ctre;

import static swervelib.simulation.ctre.PhysicsSim.random;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import swervelib.simulation.ctre.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated VictorSPX.
 */
class VictorSPXSimProfile extends SimProfile
{

  public final VictorSPX _victor;

  /**
   * Creates a new simulation profile for a VictorSPX device.
   *
   * @param victor The VictorSPX device
   */
  public VictorSPXSimProfile(final VictorSPX victor)
  {
    this._victor = victor;
  }

  /**
   * Runs the simulation profile.
   *
   * <p>This uses very rudimentary physics simulation and exists to allow users to test features of
   * our products in simulation using our examples out of the box. Users may modify this to utilize more accurate
   * physics simulation.
   */
  public void run()
  {
    // final double period = getPeriod();

    // Device voltage simulation
    double outPerc = _victor.getSimCollection().getMotorOutputLeadVoltage() / 12;
    _victor.getSimCollection().setBusVoltage(12 - outPerc * outPerc * 3 / 4 * random(0.95, 1.05));
  }
}
