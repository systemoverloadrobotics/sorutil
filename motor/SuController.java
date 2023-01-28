package frc.sorutil.motor;

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

  protected final java.util.logging.Logger logger;
  protected final org.littletonrobotics.junction.Logger aLogger;

  protected final MotorController internalController;
  protected final MotorConfiguration config;
  protected final SensorConfiguration sensorConfig;

  protected final String controllerName;
  protected final String loggerPrefix;

  protected PIDController softPidController;
  protected boolean softPidControllerEnabled;
  /**
   * softPidControllerMode is true when mode is velocity, false when it's position.
   */
  protected boolean softPidControllerMode;

  public SuController (MotorController controller, java.util.logging.Logger logger, String name) {
    this(controller, new MotorConfiguration(), logger, name);
  }

  public SuController(MotorController controller, MotorConfiguration config, java.util.logging.Logger logger, String name) {
    this(controller, config, null, logger, name);
  }

  public SuController(MotorController controller, MotorConfiguration config, SensorConfiguration sensorConfig,
      java.util.logging.Logger logger, String name) {
    MotorManager.instance().addMotor(this);

    this.logger = logger;
    this.sensorConfig = sensorConfig;
    this.internalController = controller;
    this.config = config;

    this.aLogger = org.littletonrobotics.junction.Logger.getInstance();

    this.controllerName = name.replace(" ", "");
    this.loggerPrefix = "motors/" + controllerName + "/";

    logMotorConfiguration();
    initializeLogNames();
  }

  private void logMotorConfiguration() {
    aLogger.recordOutput(loggerPrefix + "VoltageCompenstation", config.voltageCompenstationEnabled());
    // If this shows up as 0/NULL in the log, that means default.
    aLogger.recordOutput(loggerPrefix + "CurrentLimit", config.currentLimit() == null ? 0 : config.currentLimit());
    aLogger.recordOutput(loggerPrefix + "IdleMode", config.idleMode().toString());
    aLogger.recordOutput(loggerPrefix + "Inverted", config.inverted());
    aLogger.recordOutput(loggerPrefix + "MaxOutput", config.maxOutput());

    aLogger.recordOutput(loggerPrefix + "kP", config.pidProfile().p());
    aLogger.recordOutput(loggerPrefix + "kI", config.pidProfile().i());
    aLogger.recordOutput(loggerPrefix + "kD", config.pidProfile().d());

    aLogger.recordOutput(loggerPrefix + "SensorType", sensorConfig.source().getClass().getName());
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
   * Returns the last setpoint that the motor was set to, in the units of the library (Volts, RPM, degrees)
   */
  public abstract double currentSetpoint();

  /**
   * Returns the last setpoint's control mode.
   */
  public abstract ControlMode currentControlMode();

  /**
   * setSensorPosition will override the current sensor position and update the internal counter to the new position. As
   * with outputPosition, the value is in degrees.
   */
  public abstract void setSensorPosition(double position);

  // ------ Begin logged value names -----
  private String currentSetpointName;
  private String controlModeName;
  private String outputPositionName;
  private String outputVelocityName;
  // ------- End logged value names ------

  private void initializeLogNames() {
    currentSetpointName = loggerPrefix + "Setpoint";
    controlModeName = loggerPrefix + "ControlMode";
    outputPositionName = loggerPrefix + "OutputDegrees";
    outputVelocityName = loggerPrefix + "OutputRPM";
  }

  public void tick() {
    aLogger.recordOutput(currentSetpointName, currentSetpoint());
    aLogger.recordOutput(controlModeName, currentControlMode() == null ? "N/A" : currentControlMode().toString());
    aLogger.recordOutput(outputPositionName, outputPosition());
    aLogger.recordOutput(outputVelocityName, outputVelocity());
  }
}
