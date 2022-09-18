package frc.sorutil.motor;

public class PidProfile {
  private double p, i, d, f;

  public PidProfile(double p, double i, double d) {
    this(p, i, d, 0);
  }
  public PidProfile(double p, double i, double d, double f) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.f = f;
  }

  public double p() {
    return p;
  }

  public double i() {
    return i;
  }

  public double d() {
    return d;
  }
  
  public double f() {
    return f;
  }
}
