package swervelib.parser.json;

import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.wpilibj.SerialPort.Port;
import swervelib.encoders.AnalogAbsoluteEncoderSwerve;
import swervelib.encoders.CANCoderSwerve;
import swervelib.encoders.SparkMaxEncoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.imu.ADIS16448Swerve;
import swervelib.imu.ADIS16470Swerve;
import swervelib.imu.ADXRS450Swerve;
import swervelib.imu.AnalogGyroSwerve;
import swervelib.imu.NavXSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.imu.PigeonSwerve;
import swervelib.imu.SwerveIMU;
import swervelib.motors.SparkMaxBrushedMotorSwerve;
import swervelib.motors.SparkMaxSwerve;
import swervelib.motors.SwerveMotor;
import swervelib.motors.TalonFXSwerve;
import swervelib.motors.TalonSRXSwerve;

/**
 * Device JSON parsed class. Used to access the JSON data.
 */
public class DeviceJson
{

  /**
   * The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx
   */
  public String type;
  /**
   * The CAN ID or pin ID of the device.
   */
  public int    id;
  /**
   * The CAN bus name which the device resides on if using CAN.
   */
  public String canbus = "";

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the current configuration.
   *
   * @return {@link SwerveAbsoluteEncoder} given.
   */
  public SwerveAbsoluteEncoder createEncoder()
  {
    switch (type)
    {
      case "none":
      case "integrated":
      case "attached":
        return null;
      case "thrifty":
      case "throughbore":
      case "dutycycle":
      case "analog":
        return new AnalogAbsoluteEncoderSwerve(id);
      case "cancoder":
        return new CANCoderSwerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveIMU} from the given configuration.
   *
   * @return {@link SwerveIMU} given.
   */
  public SwerveIMU createIMU()
  {
    switch (type)
    {
      case "adis16448":
        return new ADIS16448Swerve();
      case "adis16470":
        return new ADIS16470Swerve();
      case "adxrs450":
        return new ADXRS450Swerve();
      case "analog":
        return new AnalogGyroSwerve(id);
      case "navx_onborard":
        return new NavXSwerve(Port.kOnboard);
      case "navx_usb":
        return new NavXSwerve(Port.kUSB);
      case "navx_mxp":
      case "navx":
        return new NavXSwerve(Port.kMXP);
      case "pigeon":
        return new PigeonSwerve(id);
      case "pigeon2":
        return new Pigeon2Swerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveMotor} from the given configuration.
   *
   * @param isDriveMotor If the motor being generated is a drive motor.
   * @return {@link SwerveMotor} given.
   */
  public SwerveMotor createMotor(boolean isDriveMotor)
  {
    switch (type)
    {
      case "sparkmax_brushed":
        switch (canbus)
        {
          case "greyhill_63r256":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 1024, false);
          case "srx_mag_encoder":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 4096, false);
          case "throughbore":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 8192, false);
          case "throughbore_dataport":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kNoSensor, 8192, true);
          case "greyhill_63r256_dataport":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 1024, true);
          case "srx_mag_encoder_dataport":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 4096, true);
          default:
            if (isDriveMotor)
            {
              throw new RuntimeException("Spark MAX " + id + " MUST have a encoder attached to the motor controller.");
            }
            // We are creating a motor for an angle motor which will use the absolute encoder attached to the data port.
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kNoSensor, 0, false);
        }
      case "neo":
      case "sparkmax":
        return new SparkMaxSwerve(id, isDriveMotor);
      case "falcon":
      case "talonfx":
        return new TalonFXSwerve(id, canbus != null ? canbus : "", isDriveMotor);
      case "talonsrx":
        return new TalonSRXSwerve(id, isDriveMotor);
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the data port on the motor controller.
   *
   * @param motor The motor to create the absolute encoder from.
   * @return {@link SwerveAbsoluteEncoder} from the motor controller.
   */
  public SwerveAbsoluteEncoder createIntegratedEncoder(SwerveMotor motor)
  {
    switch (type)
    {
      case "sparkmax":
        return new SparkMaxEncoderSwerve(motor);
      case "falcon":
      case "talonfx":
        return null;
    }
    throw new RuntimeException(
        "Could not create absolute encoder from data port of " + type + " id " + id);
  }
}
