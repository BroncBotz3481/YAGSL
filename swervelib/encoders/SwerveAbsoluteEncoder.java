package swervelib.encoders;

/**
 * Swerve abstraction class to define a standard interface with absolute encoders for swerve modules..
 */
public abstract class SwerveAbsoluteEncoder
{

  /**
   * The maximum amount of times the swerve encoder will attempt to configure itself if failures occur.
   */
  public final int     maximumRetries = 5;
  /**
   * Last angle reading was faulty.
   */
  public       boolean readingError   = false;

  /**
   * Reset the encoder to factory defaults.
   */
  public abstract void factoryDefault();

  /**
   * Clear sticky faults on the encoder.
   */
  public abstract void clearStickyFaults();

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  public abstract void configure(boolean inverted);

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  public abstract double getAbsolutePosition();

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  public abstract Object getAbsoluteEncoder();
}
