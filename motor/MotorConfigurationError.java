package frc.sorutil.motor;

public class MotorConfigurationError extends IllegalArgumentException {
  public MotorConfigurationError(String error) {
    super(error);
  }
}
