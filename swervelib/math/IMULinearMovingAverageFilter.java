package swervelib.math;

import edu.wpi.first.util.DoubleCircularBuffer;

/**
 * A linear filter that does not calculate() each time a value is added to the DoubleCircularBuffer.
 */
public class IMULinearMovingAverageFilter
{

  /**
   * Circular buffer storing the current IMU readings
   */
  private final DoubleCircularBuffer m_inputs;
  /**
   * Gain on each reading.
   */
  private final double               m_inputGain;

  /**
   * Construct a linear moving average fitler
   *
   * @param bufferLength The number of values to average across
   */
  public IMULinearMovingAverageFilter(int bufferLength)
  {
    m_inputs = new DoubleCircularBuffer(bufferLength);
    m_inputGain = 1.0 / bufferLength;
  }

  /**
   * Add a value to the DoubleCircularBuffer
   *
   * @param input Value to add
   */
  public void addValue(double input)
  {
    m_inputs.addFirst(input);
  }

  /**
   * Calculate the average of the samples in the buffer
   *
   * @return The average of the values in the buffer
   */
  public double calculate()
  {
    double returnVal = 0.0;

    for (int i = 0; i < m_inputs.size(); i++)
    {
      returnVal += m_inputs.get(i) * m_inputGain;
    }

    return returnVal;
  }
}
