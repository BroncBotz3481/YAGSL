package swervelib.simulation.ctre;

import static swervelib.simulation.ctre.PhysicsSim.random;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import swervelib.simulation.ctre.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile
{

  private final TalonFX _falcon;
  private final double  _accelToFullTime;
  private final double  _fullVel;
  private final boolean _sensorPhase;

  /** The current position */
  // private double _pos = 0;
  /**
   * The current velocity
   */
  private double _vel = 0;

  /**
   * Creates a new simulation profile for a TalonFX device.
   *
   * @param falcon          The TalonFX device
   * @param accelToFullTime The time the motor takes to accelerate from 0 to full, in seconds
   * @param fullVel         The maximum motor velocity, in ticks per 100ms
   * @param sensorPhase     The phase of the TalonFX sensors
   */
  public TalonFXSimProfile(
      final TalonFX falcon,
      final double accelToFullTime,
      final double fullVel,
      final boolean sensorPhase)
  {
    this._falcon = falcon;
    this._accelToFullTime = accelToFullTime;
    this._fullVel = fullVel;
    this._sensorPhase = sensorPhase;
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
    final double period      = getPeriod();
    final double accelAmount = _fullVel / _accelToFullTime * period / 1000;

    /// DEVICE SPEED SIMULATION

    double outPerc = _falcon.getSimCollection().getMotorOutputLeadVoltage() / 12;
    if (_sensorPhase)
    {
      outPerc *= -1;
    }
    // Calculate theoretical velocity with some randomness
    double theoreticalVel = outPerc * _fullVel * random(0.95, 1);
    // Simulate motor load
    if (theoreticalVel > _vel + accelAmount)
    {
      _vel += accelAmount;
    } else if (theoreticalVel < _vel - accelAmount)
    {
      _vel -= accelAmount;
    } else
    {
      _vel += 0.9 * (theoreticalVel - _vel);
    }
    // _pos += _vel * period / 100;

    /// SET SIM PHYSICS INPUTS

    _falcon.getSimCollection().addIntegratedSensorPosition((int) (_vel * period / 100));
    _falcon.getSimCollection().setIntegratedSensorVelocity((int) _vel);

    double supplyCurrent = Math.abs(outPerc) * 30 * random(0.95, 1.05);
    double statorCurrent = outPerc == 0 ? 0 : supplyCurrent / Math.abs(outPerc);
    _falcon.getSimCollection().setSupplyCurrent(supplyCurrent);
    _falcon.getSimCollection().setStatorCurrent(statorCurrent);

    _falcon.getSimCollection().setBusVoltage(12 - outPerc * outPerc * 3 / 4 * random(0.95, 1.05));
  }
}
