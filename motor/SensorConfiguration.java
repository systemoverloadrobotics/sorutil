package frc.sorutil.motor;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;

/**
 * SensorConfiguration presents an abstracted interface for configuring sensor values used on motors.
 * 
 * There are three classes of encoder sensors available:
 * <ul>
 * <li>IntegratedSensorSource: A sensor integral to the motor controller, only supported on TalonFX (Falcon 500 motor)
 * and SparkMax controllers.</li>
 * <li>ConnectedSensorSource: A sensor that is external to the controller, but connected directly to it, for example
 * using the SRX port on a TalonSRX.</li>
 * <li>ExternalSensorSource: A sensor that is not connected to the motor controller, but instead connects to the
 * RoboRio. Anything that extends the WPILib Encoder class is usable in this mode.</li>
 * </ul>
 * 
 * <p>
 * Note: all three configurations support an "outputGearRatio" parameter that is applied to the setpoint of the motor as
 * a multiplier. This should be set to the effective overall gear ratio between the sensor (or motor, in the case of the
 * Integrated sensor) and the mechanism itself.
 * </p>
 * <p>
 * For example, if this motor is connected to a gearbox that reduces the output 5x (i.e. a 5:1 ratio), then
 * outputGearRatio should be 5. Note: if an overdrive is applied, this value should be a fraction, not a negative
 * number. For example, if the output is geared to increase the output 2x (i.e. a 2:1 ratio), this value should be 0.5,
 * not -2.
 * </p>
 * 
 * <p>
 * If the sensor is connected directly to the output, e.g. mounted directly on a shooter output shaft, then
 * outputGearRatio should be 1.0.
 * </p>
 * 
 * <p>
 * If you use ConnectedSensorSource or ExternalSensorSource, you will need to configure the encoder type separately.
 * </p>
 * 
 * <h2>Example Sensor Configurations</h2>
 * <h3>Falcon 500 powered roller with 6:1 gearbox</h3>
 * 
 * <pre>
 * var motorConfig = ...;
 * var integratedSensor = new SensorConfiguration.IntegratedSensorSource(6.0);
 * var sensorConfig = new SensorConfiguration(integratedSensor);
 * var motor = new SuController(new WPI_TalonFX(idx), "test", motorConfig, sensorConfig);
 * 
 * // ...
 * 
 * // This will set the *output* roller speed to 800 rpm, with the motor itself spinning at 4800rpm.
 * motor.set(ControlMode.VELOCITY, 800);
 * </pre>
 * 
 * <h3>TalonSRX powered arm with 4:1 reduction before the sensor, and 23.4:1 reduction after the sensor, using external
 * sensor</h3>
 * 
 * <pre>
 * var motorConfig = ...;
 * var encoder = new Encoder(3,4); // RoboRIO channels for encoder
 * var externalSensor = new SensorConfiguration.ExternalSensor(encoder, 2048); // 2048 counts per revolution
 * // Only use the reduction after the sensor here, which is 23.4 in this case.
 * var externalSensorSource = new SensorConfiguration.ExternalSensorSource(externalSensor, 23.4); 
 * var sensorConfig = new SensorConfiguration(externalSensorSource);
 * var motor = new SuController(new WPI_TalonSRX(idx), "test", motorConfig, sensorConfig);
 * 
 * // ...
 * 
 * // This will set the *output* position to this many degrees from zero.
 * motor.set(ControlMode.POSITION, 80);
 * motor.set(ControlMode.POSITION, -43.2)
 * </pre>
 */
public class SensorConfiguration {

  private final SensorSource source;

  public SensorConfiguration(SensorSource source) {
    this.source = source;
  }

  public SensorSource source() {
    return source;
  }

  /**
   * IntegratedSensorSource configures the motor controller to use the integrated sensor available on a motor for the
   * sensing source.
   * 
   * Only the SparkMax (in brushless mode) and TalonFX support this. To use a sensor connected directly to a TalonSRX,
   * use an instance of ExternalSensorSource.
   */
  public static class IntegratedSensorSource implements SensorSource {
    public final double outputGearRatio;

    public IntegratedSensorSource(double outputGearRatio) {
      this.outputGearRatio = outputGearRatio;
    }
  }

  /**
   * ExternalSensorSource configures the motor controller to use sensors connected to the RoboRIO or provided from other
   * sources.
   * 
   * <p>
   * This requires an instance of the WPILib Encoder class to be provided, which is used as the source for the sensor.
   * Note that using ExternalSensorSource will force the library to resort to software based PID control, instead of
   * being able to use the PID control built into the motor controller itself.
   * </p>
   */
  public static class ExternalSensorSource implements SensorSource {
    public final ExternalSensor sensor;
    public final double outputGearRatio;

    public ExternalSensorSource(ExternalSensor s, double outputGearRatio) {
      this.sensor = s;
      this.outputGearRatio = outputGearRatio;
    }
  }

  /**
   * ConnectedSensorSource configures the motor controller to use sensors connected directly to the motor controller
   * (only supported by the TalonSRX and SparkMAX).
   */
  public static class ConnectedSensorSource implements SensorSource {
    public final int countsPerRev;
    public final double outputGearRatio;
    public final ConnectedSensorType type;

    private boolean inverted;

    public ConnectedSensorSource(int countsPerRev, double outputGearRatio, ConnectedSensorType type) {
      this.countsPerRev = countsPerRev;
      this.outputGearRatio = outputGearRatio;
      this.type = type;
    }

    public void setInverted(boolean inverted) {
      this.inverted = inverted;
    }

    public boolean inverted() {
      return inverted;
    }
  }

  public static enum ConnectedSensorType {
    MAG_ENCODER_ABSOLUTE, MAG_ENCODER_RELATIVE, PWM_ENCODER, QUAD_ENCODER
  }

  // ------- External Sensor types -------

  public static class CanCoder implements ExternalSensor {
    private final CANCoder enc;

    public CanCoder(CANCoder enc) {
      this.enc = enc;

      enc.configFeedbackCoefficient(0.087890625, "degrees", SensorTimeBase.PerSecond);
    }

    public void setPosition(double position) {
      enc.setPosition(position);
    }

    public double position() {
      return enc.getPosition();
    }

    public double velocity() {
      return (enc.getVelocity() / 360.0) * 60;
    }
  }

  public static class Encoder implements ExternalSensor {
    private final edu.wpi.first.wpilibj.Encoder enc;

    private double offset;

    public Encoder(edu.wpi.first.wpilibj.Encoder enc, int countsPerRev) {
      this.enc = enc;

      enc.setDistancePerPulse(1.0 / countsPerRev);
    }

    public void setPosition(double position) {
      offset = enc.getDistance() - (position / 360.0);
    }

    public double position() {
      return (enc.getDistance() * 360.0) * (360.0 * offset);
    }

    public double velocity() {
      return enc.getRate();
    }
  }

  /**
   * SensorSource describes what source the motor should use for closed loop control, if any at all.
   */
  public static interface SensorSource {
    // TODO: remote
    // CTRE controllers support remote sensors (i.e. integral sensors from other controllers) as a sensor input. We
    // should add support for this.
  }

  public static interface ExternalSensor {
    public void setPosition(double position);

    public double position();

    public double velocity();
  }
}
