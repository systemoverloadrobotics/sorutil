// NEEDS FULL REWRITING, NO USE !!!!!!!!

// package frc.sorutil.motor;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import frc.robot.Constants;
// import frc.sorutil.Errors;
// import frc.sorutil.motor.SensorConfiguration.ConnectedSensorSource;
// import frc.sorutil.motor.SensorConfiguration.ExternalSensorSource;
// import frc.sorutil.motor.SensorConfiguration.IntegratedSensorSource;

// public class SuTalonFx extends SuController {
//   private static final double DEFAULT_CURRENT_LIMIT = 80;
//   private static final double DEFAULT_NEUTRAL_DEADBAND = 0.04;
//   private static final double COUNTS_PER_REVOLUTION_INTEGRATED = 2048;

//   private final TalonFX talon;

//   private boolean voltageControlOverrideSet = false;
//   private Double lastVoltageCompensation = null;

//   private SuController.ControlMode lastMode;
//   private double lastSetpoint;
//   private double lastArbFf;

//   public SuTalonFx(TalonFX talon, String name, MotorConfiguration motorConfig, SensorConfiguration sensorConfig) {
//     super(talon, motorConfig, sensorConfig,
//         java.util.logging.Logger.getLogger(String.format("TalonFX(%d: %s)", talon.getDeviceID(), name)), name);

//     this.talon = talon;

//     configure(motorConfig, sensorConfig);
//     initializeLogNames();
//     aLogger.recordOutput(loggerPrefix + "ID", talon.getDeviceID());
//   }

//   @Override
//   protected void configure(MotorConfiguration config, SensorConfiguration sensorConfig) {
//     TalonFXConfiguration cfg = new TalonFXConfiguration();

//     Errors.handleCtre(talon.clearStickyFaults(), logger, "clearing sticky faults");

//     talon.setInverted(config.inverted());

//     var slot0Configs = new Slot0Configs();
//     slot0Configs.kV = config.pidProfile().f();
//     slot0Configs.kP = config.pidProfile().p();
//     slot0Configs.kI = config.pidProfile().i();
//     slot0Configs.kD = config.pidProfile().d();
//     talon.getConfigurator().apply(slot0Configs, 0.050);

//     Errors.handleCtre(talon.getConfigurator().apply(slot0Configs, 0.050), logger, "setting PIDF constant");

//     double limit = DEFAULT_CURRENT_LIMIT;
//     if (config.currentLimit() != null) {
//       limit = config.currentLimit();
//     }

//     cfg.withCurrentLimits(
//       new CurrentLimitsConfigs().withStatorCurrentLimit(limit).withStatorCurrentLimitEnable(true)
//     );

//     NeutralModeValue desiredMode = NeutralModeValue.Coast;
//     if (config.idleMode() == IdleMode.BRAKE) {
//       desiredMode = NeutralModeValue.Brake;
//     }
//     talon.setNeutralMode(desiredMode);

//     double neutralDeadband = DEFAULT_NEUTRAL_DEADBAND;
//     if (config.neutralDeadband() != null) {
//       neutralDeadband = config.neutralDeadband();
//     }
//     cfg.withMotorOutput(new MotorOutputConfigs()
//       .withDutyCycleNeutralDeadband(neutralDeadband)
//       .withPeakForwardDutyCycle(config.maxOutput())
//       .withPeakReverseDutyCycle(-config.maxOutput())
//     );
//     // Errors.handleCtre(talon.configNominalOutputReverse(0), logger, "configuring nominal output");
//     // Errors.handleCtre(talon.configNominalOutputForward(0), logger, "configuring nominal output");

//     if (sensorConfig != null) {
//       if (sensorConfig.source() instanceof ConnectedSensorSource) {
//         throw new MotorConfigurationError(
//             "Talon FX does not supported directly connected sensors, but was configured to use one.");
//       }

//       if (sensorConfig.source() instanceof IntegratedSensorSource) {
//         // cfg.withDifferentialSensors(new DifferentialSensorsConfigs().with)
//         // Errors.handleCtre(talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100), logger,
//         //     "configuring sensor to integrated feedback sensor");
//       }

//       if (sensorConfig.source() instanceof ExternalSensorSource) {
//         configureSoftPid();
//       }
//     }
//   }

//   @Override
//   public MotorController rawController() {
//     return talon;
//   }

//   @Override
//   public void tick() {
//     super.tick();

//     if (talon.hasResetOccurred()) {

//     }

//     if (softPidControllerEnabled) {
//       double current = 0;
//       if (softPidControllerMode) {
//         // velocity mode
//         current = ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
//       } else {
//         // position mode
//         current = ((ExternalSensorSource) sensorConfig.source()).sensor.position();
//       }
//       double output = softPidController.calculate(current);
//       if (lastArbFf == 0) {
//         talon.set(output);
//       } else {
//         // TODO FIX :(
//         // talon.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, lastArbFf / Constants.NOMINAL_VOLTAGE);
//       }
//     }

//     recordLogs();
//   }

//   @Override
//   public void set(SuController.ControlMode mode, double setpoint) {
//     set(mode, setpoint, 0);
//   }

//   @Override
//   public void set(SuController.ControlMode mode, double setpoint, double arbFfVolts) {
//     if (voltageControlOverrideSet && mode != SuController.ControlMode.VOLTAGE) {
//       voltageControlOverrideSet = false;
//       lastVoltageCompensation = null;
//     }

//     // Skip updating the motor if the setpoint is the same, this reduces
//     // unneccessary CAN messages.
//     if (setpoint == lastSetpoint && mode == lastMode && arbFfVolts == lastArbFf) {
//       return;
//     }
//     lastSetpoint = setpoint;
//     lastMode = mode;
//     softPidControllerEnabled = false;
//     lastArbFf = arbFfVolts;

//     switch (mode) {
//       case PERCENT_OUTPUT:
//         talon.set(setpoint);
//         break;
//       case POSITION:
//         setPosition(setpoint, arbFfVolts);
//         break;
//       case VELOCITY:
//         setVelocity(setpoint, arbFfVolts);
//         break;
//       case VOLTAGE:
//         boolean negative = setpoint < 0;
//         double abs = Math.abs(setpoint);
//         talon.setControl(new VoltageOut(negative ? -1 : 1));
//         break;
//     }
//   }

//   private void setPosition(double setpoint, double arbFfVolts) {
//     // Using the integrated Falcon source
//     if (sensorConfig.source() instanceof SensorConfiguration.IntegratedSensorSource) {
//       var integrated = (SensorConfiguration.IntegratedSensorSource) sensorConfig.source();
//       double motorDegrees = integrated.outputGearRatio * setpoint;
//       double countsToDegrees = COUNTS_PER_REVOLUTION_INTEGRATED / 360.0;
//       double output = motorDegrees * countsToDegrees;

//       if (arbFfVolts == 0) {
//         talon.set(output);
//       } else {
//         // talon.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, output, DemandType.ArbitraryFeedForward, arbFfVolts / Constants.NOMINAL_VOLTAGE);
//       }
//       return;
//     }
//     // Using sensor external to the Falcon.
//     if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
//       softPidControllerEnabled = true;
//       softPidControllerMode = false;

//       double current = ((ExternalSensorSource) sensorConfig.source()).sensor.position();
//       double output = softPidController.calculate(current, setpoint);
//       if (arbFfVolts == 0) {
//         talon.set(output);
//       } else {
//         // talon.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward,
//         //     arbFfVolts / Constants.NOMINAL_VOLTAGE);
//       }
//       return;
//     }
//     throw new MotorConfigurationError(
//         "unkonwn type of sensor configuration: " + sensorConfig.source().getClass().getName());
//   }

//   private void setVelocity(double setpoint, double arbFfVolts) {
//     // Using the integrated Falcon source
//     if (sensorConfig.source() instanceof SensorConfiguration.IntegratedSensorSource) {
//       var integrated = (SensorConfiguration.IntegratedSensorSource) sensorConfig.source();
//       double motorRpm = integrated.outputGearRatio * setpoint;
//       double motorRps = motorRpm / 60.0;
//       double output = (COUNTS_PER_REVOLUTION_INTEGRATED * motorRps) / 10.0;

//       if (arbFfVolts == 0) {
//         talon.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, output);
//       } else {
//         talon.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, output, DemandType.ArbitraryFeedForward,
//             arbFfVolts / Constants.NOMINAL_VOLTAGE);
//       }
//       return;
//     }
//     // Using sensor external to the Falcon.
//     if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
//       softPidControllerEnabled = true;
//       softPidControllerMode = true;

//       double current = ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
//       double output = softPidController.calculate(current, setpoint);
//       if (arbFfVolts == 0) {
//         talon.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, output);
//       } else {
//         talon.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, output, DemandType.ArbitraryFeedForward,
//             arbFfVolts / Constants.NOMINAL_VOLTAGE);
//       }
//       return;
//     }
//     throw new MotorConfigurationError(
//         "unkonwn type of sensor configuration: " + sensorConfig.source().getClass().getName());
//   }

//   @Override
//   public void stop() {
//     talon.stopMotor();

//     lastMode = null;
//     lastSetpoint = 0;
//   }

//   @Override
//   public void follow(SuController other) {
//     if (!(other.rawController() instanceof IFollower)) {
//       throw new MotorConfigurationError(
//           "CTRE motor controllers can only follow other motor controllers from CTRE");
//     }

//     talon.follow((IMotorController) other.rawController());
//   }

//   @Override
//   public double outputPosition() {
//     if (sensorConfig.source() instanceof IntegratedSensorSource) {
//       IntegratedSensorSource source = (IntegratedSensorSource) sensorConfig.source();
//       double outputShaftPosition = talon.getSelectedSensorPosition() / COUNTS_PER_REVOLUTION_INTEGRATED * 360.0;
//       return outputShaftPosition / source.outputGearRatio;
//     }
//     if (sensorConfig.source() instanceof ExternalSensorSource) {
//       ExternalSensorSource source = (ExternalSensorSource) sensorConfig.source();
//       double sensorPosition = source.sensor.position();
//       return sensorPosition / source.outputGearRatio;
//     }
//     return 0;
//   }

//   @Override
//   public double outputVelocity() {
//     if (sensorConfig.source() instanceof IntegratedSensorSource) {
//       IntegratedSensorSource source = (IntegratedSensorSource) sensorConfig.source();
//       double outputShaftRpm = (talon.getSelectedSensorVelocity() * 10.0 * 60.0) / COUNTS_PER_REVOLUTION_INTEGRATED;
//       return outputShaftRpm / source.outputGearRatio;
//     }
//     if (sensorConfig.source() instanceof ExternalSensorSource) {
//       ExternalSensorSource source = (ExternalSensorSource) sensorConfig.source();
//       double sensorRpm = source.sensor.velocity();
//       return sensorRpm / source.outputGearRatio;
//     }
//     return 0;
//   }

//   @Override
//   public void setSensorPosition(double position) {
//     if (sensorConfig.source() instanceof IntegratedSensorSource) {
//       System.out.println("Hi start");
//       IntegratedSensorSource source = (IntegratedSensorSource) sensorConfig.source();
//       double offset = source.outputGearRatio * COUNTS_PER_REVOLUTION_INTEGRATED * position;
//       talon.setSelectedSensorPosition(offset);
//     }
//     if (sensorConfig.source() instanceof ExternalSensorSource) {
//       ExternalSensorSource source = (ExternalSensorSource) sensorConfig.source();
//       source.sensor.setPosition(position * source.outputGearRatio);
//     }
//   }

//   @Override
//   public double currentSetpoint() {
//     return lastSetpoint;
//   }

//   @Override
//   public ControlMode currentControlMode() {
//     return lastMode;
//   }

//   // ------ Begin logged value names -----
//   private String inputVoltageName;
//   private String outputVoltageName;
//   private String supplyCurrentName;
//   private String lastErrorName;
//   private String selectedSensorPositionName;
//   private String selectedSensorVelocityName;
//   private String controllerTemperatureName;
//   private String hasResetName;
//   private String closedLoopTargetName;
//   private String closedLoopErrorName;
//   private String motorOutputName;
//   private String arbFfName;
//   // ------- End logged value names ------

//   private void initializeLogNames() {
//     inputVoltageName = loggerPrefix + "InputVoltage";
//     outputVoltageName = loggerPrefix + "OutputVoltage";
//     supplyCurrentName = loggerPrefix + "SupplyCurrent";
//     lastErrorName = loggerPrefix + "LastError";
//     selectedSensorPositionName = loggerPrefix + "SelectedSensorPosition";
//     selectedSensorVelocityName = loggerPrefix + "SelectedSensorVelocity";
//     controllerTemperatureName = loggerPrefix + "ControllerTemperature";
//     hasResetName = loggerPrefix + "HasResetOccurred";
//     closedLoopTargetName = loggerPrefix + "ClosedLoopTarget";
//     closedLoopErrorName = loggerPrefix + "ClosedLoopError";
//     motorOutputName = loggerPrefix + "MotorOutputPercent";
//     arbFfName = loggerPrefix + "ArbitraryFeedForwardVolts";
//   }

//   private void recordLogs() {
//     aLogger.recordOutput(inputVoltageName, talon.getBusVoltage());
//     aLogger.recordOutput(outputVoltageName, talon.getMotorOutputVoltage());
//     aLogger.recordOutput(supplyCurrentName, talon.getSupplyCurrent());
//     aLogger.recordOutput(lastErrorName, talon.getLastError().value);
//     aLogger.recordOutput(selectedSensorPositionName, talon.getSelectedSensorPosition());
//     aLogger.recordOutput(selectedSensorVelocityName, talon.getSelectedSensorVelocity());
//     aLogger.recordOutput(controllerTemperatureName, talon.getTemperature());
//     aLogger.recordOutput(hasResetName, talon.hasResetOccurred());
//     aLogger.recordOutput(motorOutputName, talon.getMotorOutputPercent());
//     aLogger.recordOutput(arbFfName, lastArbFf);

//     if (lastMode == ControlMode.POSITION || lastMode == ControlMode.VELOCITY) {
//       aLogger.recordOutput(closedLoopTargetName, talon.getClosedLoopTarget());
//       aLogger.recordOutput(closedLoopErrorName, talon.getClosedLoopError());
//     }
//   }
// }
