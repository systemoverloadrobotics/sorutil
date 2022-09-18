package frc.sorutil.motor;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SuMotor <T extends SuController> {
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

  private final Logger logger;

  private T controller;
  private final SensorConfiguration sensorConfig;

  public SuMotor (T controller) {
    this(controller, new MotorConfiguration());
  }

  public SuMotor (T controller, MotorConfiguration config) {
    this(controller, config, null);
  }

  public SuMotor (T controller, MotorConfiguration config, SensorConfiguration sensorConfig) {
    logger = Logger.getLogger(SuMotor.class.getName());
    this.controller = controller;
    MotorManager.instance().addMotor(this);
    this.sensorConfig = sensorConfig;

    controller.configure(config, sensorConfig);
  }

  public MotorController rawController() {
    return controller.rawController();
  }

  public void set(ControlMode mode, double setpoint) {
    controller.set(mode, setpoint);
  }

  public void stop() {
    controller.stop();
  }

  public double outputVelocity() {
    return controller.outputVelocity();
  }

  public double outputPosition() {
    return controller.outputPosition();
  }

  public void setSensorPosition(double position) {
    controller.setSensorPosition(position);
  }

  /**
   * Configure this motor to match the output of the other controller.
   * 
   * Note: the controllers must be of the same family, or this will throw an exception and fail.
   * 
   * @param other
   */
  public void follow(SuMotor other) {
    controller.follow(other.controller);
  }

  protected void tick() {
    controller.tick();
  }
}
