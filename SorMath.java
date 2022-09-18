package frc.sorutil;

import frc.robot.Constants;

public class SorMath {
  public static final double EPSILON = 1e-6;
  public static final float F_EPSILON = 1e-5f;


  public static double[] cartesianToPolar(double x, double y) {
    return new double[] {Math.sqrt(x * x + y * y), (Math.atan2(y, x) * 180) / Math.PI};
  }

  public static double[] polarToCartesian(double r, double theta) {
    theta = (theta / 180) * Math.PI;
    return new double[] {r * Math.cos(theta), r * Math.sin(theta)};
  }

  /**
   * epsilonEquals determines whether two floats are approximately equal.
   */
  public static boolean epsilonEquals(float a, float b) {
    float diff = a - b;
    return diff < F_EPSILON && diff > 0f - F_EPSILON;
  }

  /**
   * epsilonEquals determines whether two doubles are approximately equal. Should be used any time comparing doubles.
   */
  public static boolean epsilonEquals(double a, double b) {
    double diff = a - b;
    return diff < EPSILON && diff > 0d - EPSILON;
  }

  /**
   * epsilonEquals determines whether two doubles are approximately equal, with a specified epsilon value. Should be used
   * any time comparing doubles.
   */
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    double diff = a - b;
    return diff < epsilon && diff > 0d - epsilon;
  }

  /**
   * linearInterpolate takes the value specified in input, using a simple linear interpolation to determine where along
   * the scale between scaleLow and scaleHigh this lies. This value is then used to scale the input vs. low/high.
   * 
   * <p>
   * That is to say, if we have the relationship:
   * <ul>
   * <li>10 -> 100</li>
   * <li>20 -> x</li>
   * <li>30 -> 300</li>
   * </ul>
   * We would expect linearInterpolate to return 200.
   * </p>
   * 
   * Note that there is an expectation that scaleLow < scaleHigh, however, low is not neccessarily "lower" than high.
   * 
   * @param input the value for which you'd like an interpolated input in the (low, high) range for
   * @param scaleLow the lower bound of the interpolation window, 10 in the example
   * @param scaleHigh the upper bound of the interpolation window, 30 in the example
   * @param low the corresponding value for the lower bound, 100 in the example
   * @param high the corresponding value for the higher bound, 300 in the example
   * @return
   */
  public static double linearInterpolate(double input, double scaleLow, double scaleHigh,
      double low, double high) {
    if (SorMath.epsilonEquals(scaleLow, scaleHigh)) {
      throw new IllegalArgumentException("Scale arguments to linearInterpolate must be non-equal!");
    }
    double x = scaleHigh - scaleLow;
    double y = input - scaleLow;

    double proportion = y / x;

    double delta = high - low;

    return (proportion * delta) + low;
  }


  public static double ticksToDegrees(double ticks, double ticksPerRotation) {
    return (ticks / ticksPerRotation) * 360;
  }

  public static double degreesToTicks(double angle, double ticksPerRotation) {
    return (angle / 360d) * ticksPerRotation;
  }

  public static double sensorUnitsPer100msToMetersPerSecond(double sensorUnitsPer100ms) {
    return (sensorUnitsPer100ms * (Constants.RobotDimensions.WHEEL_CIRCUMFERENCE / 4096)) * 10;
  }

  public static double signedSquare(double a) {
    if (a < 0) {
      return -(a * a);
    }
    return a * a;
  }
}
