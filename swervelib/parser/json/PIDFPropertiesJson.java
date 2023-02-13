package frc.robot.subsystems.swervedrive2.swervelib.parser.json;

import frc.robot.subsystems.swervedrive2.swervelib.parser.PIDFConfig;

/**
 * {@link frc.robot.subsystems.swervedrive2.swervelib.SwerveModule} PID with Feedforward for the drive motor and angle
 * motor.
 */
public class PIDFPropertiesJson
{

  /**
   * The PIDF with Integral Zone used for the drive motor.
   */
  public PIDFConfig drive;
  /**
   * The PIDF with Integral Zone used for the angle motor.
   */
  public PIDFConfig angle;
}
