package frc.sorutil.motor;

public class MotorConfiguration {
    private boolean voltageCompenstation = true; 
    private PidProfile pidProfile = new PidProfile(0, 0, 0);
    /**
     * currentLimit is left null, where a default will be chosen for a given motor by the library, if not overriden.
     */
    private Double currentLimit = null;

    private SuController.IdleMode idleMode = SuController.IdleMode.COAST;

    private Double neutralDeadband = null;

    private boolean inverted = false;

    private double maxOutput = 1;

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
