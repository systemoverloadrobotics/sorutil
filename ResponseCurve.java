package frc.sorutil;

/**
 * NOTE: This code doesn't really work, it requires that the user compute curves on their own that line up, which seems
 * like it isn't the ideal solution. I'll need to come back and rewrite this completely.
 * 
 * ResponseCurve (and its contained classes) help to define a function that relates one input value to a single output
 * value. Generally, input values are expected to be in the range (-1, 1), outputs can either be the raw output of the
 * function or optionally capped to (-1, 1) using "cap()".
 * 
 * Inputs below zero can have one of the following behaviours:
 * <ul>
 * <li>Direct function value, only valid for certain functions (polynomials).</li>
 * <li>Absolute value function, where the input's absolute value is taken and the function is effectively mirrored about
 * the Y-axis.</li>
 * <li>Mirrored function, where the input's absolute value is taken <b>and</b> the function's output's sign is matched
 * to the input before processing, effectively mirroring the function about the X- and Y-axes</li>
 * </ul>
 */
public class ResponseCurve {
  public static interface Function {
    /**
     * Evaluates a function for the given value, i.e. returns f(x).
     * 
     * @param x
     * @return f(x)
     */
    public double eval(double x);

    /**
     * Evalutes the slope of a function at a given value, must never error. Discontinuities should be either 0 or
     * DOUBLE_MAX.
     * 
     * @param x
     * @return the slope of the function at the given value, i.e. f'(x)
     */
    public double slope(double x);

    /**
     * An intrinsic property of a given function, returns true if the function is valid below zero, false if it is not. Note
     * that the function may be defined below zero, this is whether it is useful.
     * 
     * @return whether the function is valid negative.
     */
    public boolean signValid();
  }

  /**
   * Quadratic implements a quadratic equation of the standard form
   * 
   * ax^2 + bx + c
   */
  public static class Quadratic implements Function {
    private double a, b, c;

    public Quadratic(double a, double b, double c) {
      this.a = a;
      this.b = b;
      this.c = c;
    }

    public Quadratic(double a, double b) {
      this.a = a;
      this.b = b;
    }

    @Override
    public double eval(double x) {
      return (a * x * x) + (b * x) + c;
    }

    @Override
    public double slope(double x) {
      return (2 * a * x) + b;
    }

    @Override
    public boolean signValid() {
      return true;
    }
  }

  /**
   * Cubic implements a cubic equation of the standard form
   * 
   * ax^3 + bx^2 + cx + d
   */
  public static class Cubic implements Function {
    private double a, b, c, d;

    public Cubic(double a, double b, double c, double d) {
      this.a = a;
      this.b = b;
      this.c = c;
      this.d = d;
    }

    public Cubic(double a, double b, double c) {
      this.a = a;
      this.b = b;
      this.c = c;
    }

    @Override
    public double eval(double x) {
      return (a * x * x * x) + (b * x * x) + (c * x) + d;
    }

    @Override
    public double slope(double x) {
      return (3 * a * x * x) + (2 * b * x) + c;
    }

    @Override
    public boolean signValid() {
      return true;
    }
  }

  public class Sqrt implements Function {
    @Override
    public double eval(double x) {
      return Math.sqrt(x);
    }

    @Override
    public double slope(double x) {
      // Handle the discontinuity at 0 by assigning the slope to 0.
      if (SorMath.epsilonEquals(0d, x)) {
        return 0;
      }

      return 1 / 2 * Math.sqrt(x);
    }

    @Override
    public boolean signValid() {
      return false;
    }
  }

  /**
   * FractionalExponentPolynominal implements a polynominal of the form:
   * 
   * ax^f+bx+c
   * 
   * Where f is a constant provided as a parameter to the constructor, may be a non-integer.
   */
  public static class FractionalExponentPolynominal implements Function {
    private double a, b, c, f;

    public FractionalExponentPolynominal(double a, double b, double c, double f) {
      this.a = a;
      this.b = b;
      this.c = c;
      this.f = f;
    }

    @Override
    public double eval(double x) {
      return (a * Math.pow(x, f)) + (b * x) + c;
    }

    @Override
    public double slope(double x) {
      return (a * f * Math.pow(x, f - 1d)) + b;
    }

    @Override
    public boolean signValid() {
      return false;
    }
  }

  public static enum NegativeMode {
    DIRECT, ABSOLUTE, MIRROR;
  }

  public static class FunctionRange {
    public static final double LOW = -Double.MAX_VALUE;
    public final double start;
    public final Function f;

    public FunctionRange(Function f) {
      this.start = LOW;
      this.f = f;
    }

    public FunctionRange(double start, Function f) {
      this.start = start;
      this.f = f;
    }
  }

  private final NegativeMode mode;
  private final FunctionRange[] ranges;

  public ResponseCurve(Function f) {
    this(f, NegativeMode.DIRECT);
  }

  public ResponseCurve(Function f, NegativeMode mode) {
    this(new FunctionRange[] {new FunctionRange(f)}, mode);
  }

  public ResponseCurve(FunctionRange[] ranges, NegativeMode mode) {
    this.mode = mode;
    this.ranges = ranges;
  }
}
