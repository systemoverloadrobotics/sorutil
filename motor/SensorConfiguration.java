package frc.sorutil.motor;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;

/**
 * SensorConfiguration presents an abstracted interface for configuring 
 */
public class SensorConfiguration {
  
  public static interface ExternalSensor {
    public void setPosition(double position);
    public double position();
    public double velocity();
  }

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
      return enc.getVelocity() / 6;
    }
  }

  public static class Encoder implements ExternalSensor {
    private final edu.wpi.first.wpilibj.Encoder enc;

    private double offset;

    public Encoder(edu.wpi.first.wpilibj.Encoder enc, int countsPerRev) {
      this.enc = enc;

      enc.setDistancePerPulse(1.0/countsPerRev);
    } 

    public void setPosition(double position) {
      offset = enc.getDistance() - position; 
    }

    public double position() {
      return enc.getDistance() * 360 - offset;
    }

    public double velocity() {
      return enc.getRate();
    }
  }

  // public static class ControllerSensor implements ExternalSensor {
  //   private final int countsPerRevolution;
  //   // Store a Talon SRX here directly, as it's the only controller that supports a connected sensor.
  //   private final TalonSRX talon;

  //   // TODO: fix this API
  //   // TODO: support CTRE MagEncoder
  //   public ControllerSensor(int countsPerRevolution, SuController controller) {
  //     this.countsPerRevolution = countsPerRevolution;

  //     if (!(controller instanceof SuTalonSrx)) {
  //       throw new MotorConfigurationError("Cannot created connected sensor with motor controller other than Talon SRX");
  //     }

  //     talon = (TalonSRX) controller.rawController();
  //     talon.configSelectedFeedbackCoefficient(1);
  //     talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  //   }

  //   public void setPosition(double position) {
  //     talon.setSelectedSensorPosition(position * (countsPerRevolution / 360));
  //   }

  //   public double position() {
  //     return talon.getSelectedSensorPosition();
  //   }
    
  //   public double velocity() {
  //     return (talon.getSelectedSensorVelocity() * 10) / (60.0 * countsPerRevolution);
  //   }
  // }

  /**
   * SensorSource describes what source the motor should use for closed loop control, if any at all.
   * 
   * <p>
   * Every mode takes an outputOffset parameter, which is required, and describes the ratio between the rotation of the
   * motor and the end effector. For example, if this motor is connected to a gearbox that reduces the output 5x (i.e. a
   * 5:1 ratio), then outputOffset should be 5. Note: if an overdrive is applied, this value should be a fraction, not a
   * negative number. For example, if the output is geared to increase the output 2x (i.e. a 2:1 ratio), this value should
   * be 0.5, not -2.
   * </p>
   * 
   * <p>
   * If the sensor is connected directly to the output, e.g. mounted directly on a shooter output shaft, then outputOffset
   * should be 1.0.
   * </p>
   */
  public static interface SensorSource {
    // TODO: remote
    // CTRE controllers support remote sensors (i.e. integral sensors from other controllers) as a sensor input. We
    // should add support for this.

  }

  /**
   * IntegratedSensorSource configures the motor controller to use the integrated sensor available on a motor for the
   * sensing source.
   * 
   * Only the SparkMax (in brushless mode) and TalonFX support this, to use a sensor connected directly to a TalonSRX, use
   * an instance of ExternalSensorSource.
   * 
   */
  public static class IntegratedSensorSource implements SensorSource {
    public final double outputOffset;

    public IntegratedSensorSource(double outputOffset) {
      this.outputOffset = outputOffset;
    }
  }
  
  public static class ExternalSensorSource implements SensorSource {
    public final ExternalSensor sensor;
    public final double outputOffset;

    public ExternalSensorSource(ExternalSensor s, double outputOffset) {
      this.sensor = s;
      this.outputOffset = outputOffset;
    }
  }

  public static enum ConnectedSensorType {
    MAG_ENCODER_ABSOLUTE,
    MAG_ENCODER_RELATIVE,
    PWM_ENCODER,
    QUAD_ENCODER
  }

  public static class ConnectedSensorSource implements SensorSource {
    public final int countsPerRev;
    public final double outputOffset;
    public final ConnectedSensorType type;

    private boolean inverted;

    public ConnectedSensorSource(int countsPerRev, double outputOffset, ConnectedSensorType type) {
      this.countsPerRev = countsPerRev;
      this.outputOffset = outputOffset;
      this.type = type;
    }

    public void setInverted(boolean inverted) {
      this.inverted = inverted;
    }

    public boolean inverted() {
      return inverted;
    }
  }

  private final SensorSource source;

  public SensorConfiguration(SensorSource source) {
    this.source = source;
  }
  
  public SensorSource source() {
    return source;
  }
}
