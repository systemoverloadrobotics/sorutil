package frc.sorutil.motor;

/**
 * Class that holds meta configuration values for all types of motor
 * controllers. Using options not supported by a motor controller will result in
 * an error.
 * 
 * <p>
 * Note that all PID control is currently done in the native units of the motor
 * controller, and will require adjusting from values calculated. This does not
 * apply to soft PID controllers, which are implemented with the WPILib values.
 * </p>
 */
public class MotorConfiguration {

    // ---- Begin configuration options ----

    private boolean voltageCompenstation = true; 

    private PidProfile pidProfile = new PidProfile(0, 0, 0);

    /**
     * currentLimit specifies a current limit for the motor in Amps. If this is left
     * as null (default), the current limit will be choosen automatically by the
     * library using a default for the given motor controller type.
     */
    private Double currentLimit = null;

    private SuController.IdleMode idleMode = SuController.IdleMode.COAST;

    /**
     * neutralDeadband provides a range of motor inputs that are too low for the
     * motor output to respond, if left null, this will be left to the default of
     * the motor controller.
     */
    private Double neutralDeadband = null;

    private boolean inverted = false;

    private double maxOutput = 1;

    // ---- End configuration options ----

    public MotorConfiguration() {}

    public void setVoltageCompenstationEnabled(boolean enable) {
      voltageCompenstation = enable;
    }

    public void setNeutralDeadband(Double deadband) {
      neutralDeadband = deadband;
    }

    public void setPidProfile(PidProfile profile) {
      pidProfile = profile;
    }

    public void setCurrentLimit(Double limit) {
      currentLimit = limit;
    }
    
    public void setIdleMode(SuController.IdleMode mode) {
      this.idleMode = mode;
    }

    public void setInverted(boolean inverted) {
      this.inverted = inverted;
    }

    public void setMaxOutput(double output) {
      this.maxOutput = output;
    }

    public boolean voltageCompenstationEnabled() {
      return voltageCompenstation;
    }

    public PidProfile pidProfile() {
      return pidProfile;
    }

    public Double currentLimit () {
      return currentLimit;
    }
    
    public SuController.IdleMode idleMode() {
      return idleMode;
    }

    public Double neutralDeadband() {
      return neutralDeadband;
    }

    public boolean inverted() {
      return inverted;
    }

    public double maxOutput() {
      return maxOutput;
    }
  }
