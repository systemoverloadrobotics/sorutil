package frc.sorutil.motor;

import java.util.logging.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.sorutil.Errors;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorSource;
import frc.sorutil.motor.SuMotor.ControlMode;

public class SuSparkMax implements SuController {
  private static final double STALL_LIMIT = 30;
  private static final double DEFAULT_CURRENT_LIMIT = 70;
  private static final double DEFAULT_NEUTRAL_DEADBAND = 0.04;

  private final Logger logger;

  private final CANSparkMax sparkMax;

  private SuMotor.ControlMode lastMode;
  private double lastSetpoint;

  private SensorConfiguration sensorConfig;

  public SuSparkMax(CANSparkMax sparkMax, String name) {
    int channel = sparkMax.getDeviceId();
    logger = Logger.getLogger(String.format("SparkMax(%d: %s)", channel, name));

    this.sparkMax = sparkMax;
  }

  @Override
  public void configure(MotorConfiguration config, SensorConfiguration sensorConfig) {
    this.sensorConfig = sensorConfig;

    Errors.handleRev(sparkMax.restoreFactoryDefaults(), logger, "resetting motor config");

    sparkMax.enableVoltageCompensation(12);
    if (!config.voltageCompenstationEnabled()) {
      sparkMax.disableVoltageCompensation();
    }

    sparkMax.setInverted(config.inverted());

    Errors.handleRev(sparkMax.getPIDController().setP(config.pidProfile().p()), logger, "setting P constant");
    Errors.handleRev(sparkMax.getPIDController().setI(config.pidProfile().i()), logger, "setting I constant");
    Errors.handleRev(sparkMax.getPIDController().setD(config.pidProfile().d()), logger, "setting D constant");
    Errors.handleRev(sparkMax.getPIDController().setFF(config.pidProfile().f()), logger, "setting F constant");

    double limit = DEFAULT_CURRENT_LIMIT;
    if (config.currentLimit() != null) {
      limit = config.currentLimit();
    }

    Errors.handleRev(sparkMax.setSmartCurrentLimit((int) STALL_LIMIT, (int) limit), logger, "setting current limit");

    CANSparkMax.IdleMode desiredMode = CANSparkMax.IdleMode.kCoast;
    if (config.idleMode() == SuMotor.IdleMode.BRAKE) {
      desiredMode = CANSparkMax.IdleMode.kBrake;
    }
    Errors.handleRev(sparkMax.setIdleMode(desiredMode), logger, "setting idle mode");

    double neutralDeadband = DEFAULT_NEUTRAL_DEADBAND;
    if (config.neutralDeadband() != null){
      neutralDeadband = config.neutralDeadband();
    }

    // TODO: this should be improved to support multiple PID controllers.
    Errors.handleRev(sparkMax.getPIDController().setOutputRange(neutralDeadband, config.maxOutput()), logger, "setting neutral deadband");

    Errors.handleRev(sparkMax.burnFlash(), logger, "saving settings to onboard Flash");

    if (sensorConfig != null) {
      if (sensorConfig.source() instanceof ConnectedSensorSource) {
        throw new MotorConfigurationError(
            "Library doesn't currently support using externally connected sensors with Spark Max, but was configured to use one.");
      }
    }
  }

  @Override
  public MotorController rawController() {
    return sparkMax;
  }

  @Override
  public void tick() {
    Errors.handleRev(sparkMax.getLastError(), logger, "in motor loop, likely due to setting output");
  }

  @Override
  public void set(ControlMode mode, double setpoint) {
    // Skip updating the motor if the setpoint is the same, this reduces unneccessary CAN messages.
    if (setpoint == lastSetpoint && mode == lastMode) {
      return;
    }
    lastSetpoint = setpoint;
    lastMode = mode;

    switch(mode) {
      case PERCENT_OUTPUT:
        sparkMax.set(setpoint);
      case POSITION:
        Errors.handleRev(sparkMax.getPIDController().setReference(setpoint, ControlType.kPosition), logger, "setting motor output");
      case VELOCITY:
        Errors.handleRev(sparkMax.getPIDController().setReference(setpoint, ControlType.kVelocity), logger, "setting motor output");
      case VOLTAGE:
        Errors.handleRev(sparkMax.getPIDController().setReference(setpoint, ControlType.kVoltage), logger, "setting motor output");
    }
  }

  @Override
  public void stop() {
    sparkMax.stopMotor();
  }

  @Override
  public void follow(SuController other) { 
    // Techincally, the spark max can follow other motor controllers, but for now we will only allow the following of other
    // SparkMaxes for safety.

    if (!(other.rawController() instanceof CANSparkMax)) {
      throw new MotorConfigurationError(
          "SparkMax may only follow other spark max, got instance of: "
              + other.rawController().getClass().getName());
    }

    sparkMax.follow((CANSparkMax) other.rawController());
  }

  @Override
  public double outputPosition() {
    return 0;
  }

  @Override
  public double outputVelocity() {
    return 0;
  }

  @Override
  public void setSensorPosition(double position) {

  }
}