package frc.sorutil.motor;

import java.util.logging.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public abstract class SuController {
  public static final double DEFAULT_VOLTAGE_COMPENSTAION = 12.5;

  public static enum ControllerType {
    TALON_FX,
    TALON_SRX,
    SPARK_MAX,
    VICTOR_SPX,
  }

  public static enum ControlMode {
    PERCENT_OUTPUT,
    POSITION,
    VELOCITY,
    VOLTAGE,
  }

  public static enum IdleMode{
    BRAKE,
    COAST,
  }

  protected final Logger logger;

  protected final MotorController internalController;
  protected final MotorConfiguration config;
  protected final SensorConfiguration sensorConfig;

  protected PIDController softPidController;
  protected boolean softPidControllerEnabled;
  /**
   * softPidControllerMode is true when mode is velocity, false when it's position.
   */
  protected boolean softPidControllerMode;

  public SuController (MotorController controller, Logger logger) {
    this(controller, new MotorConfiguration(), logger);
  }

  public SuController(MotorController controller, MotorConfiguration config, Logger logger) {
    this(controller, config, null, logger);
  }

  public SuController(MotorController controller, MotorConfiguration config, SensorConfiguration sensorConfig, Logger logger) {
    MotorManager.instance().addMotor(this);

    this.logger = logger;
    this.sensorConfig = sensorConfig;
    this.internalController = controller;
    this.config = config;

    this.configure(config, sensorConfig);
  }

  protected void configureSoftPid() {
    softPidController = new PIDController(motorConfig().pidProfile().p(), motorConfig().pidProfile().i(),
        motorConfig().pidProfile().d());
  }

  protected MotorConfiguration motorConfig() {
    return config;
  }

  protected SensorConfiguration sensorConfig() {
    return sensorConfig;
  }

  protected abstract void configure(MotorConfiguration config, SensorConfiguration sensorConfig); 

  public abstract MotorController rawController();

  public abstract void tick();

  /**
   * Set the motor output based on a control mode to a given setpoint. The
   * setpoint will be sought using the configured sensor and the integrated PID
   * controller, if configured and available. Note that the sensor configuration
   * reduction <em>will</em> be respected by this method.
   * 
   * @param mode     Control Mode to use, with the following options supported:
   *                 <p>
   *                 PERCENT_OUTPUT: a percentage from -1.0 to 1.0 that directly
   *                 regulates the motor's output power to that percentage of the
   *                 input voltage
   *                 </p>
   *                 <p>
   *                 POSITION: position from starting position (or from set zero)
   *                 in degrees.
   *                 </p>
   *                 <p>
   *                 VELOCITY: rate of rotation in revolutions per minute (rpm)
   *                 </p>
   *                 <p>
   *                 VOLTAGE: commands the motor controller to regulate directly
   *                 to the specified value in volts
   *                 </p>
   * @param setpoint
   */
  public abstract void set(ControlMode mode, double setpoint);

  /**
   * Stops the motor regardless of output mode.
   */
  public abstract void stop();

  public abstract void follow(SuController other);

  /**
   * Using the SensorConfiguration on this motor, retreive the end effector position in degrees. This will take into
   * account the scaling factor provided in the SensorConfiguration.
   */
  public abstract double outputPosition();

  /**
   * Using the SensorConfiguration on this motor, retreive the end effector output velocity in RPM. This will take into
   * account the scaling factor provided in the SensorConfiguration.
   */
  public abstract double outputVelocity();

  /**
   * setSensorPosition will override the current sensor position and update the internal counter to the new position. As
   * with outputPosition, the value is in degrees.
   */
  public abstract void setSensorPosition(double position);
}
