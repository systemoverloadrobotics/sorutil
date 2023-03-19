package frc.sorutil;

import frc.robot.Constants;

public class SorMath {
  public static final double EPSILON = 1e-6;
  public static final float F_EPSILON = 1e-5f;


  /**
   * caresianToPolar converts an x, y coordinate pair into polar coordinates
   * 
   * Note that negative values can produce a negative theta, which indicates clockwise rotation, rather than an angle
   * >180.
   * 
   * @param x
   * @param y
   * @return an array with two elements representing the r and theta of the polar vector.
   */
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

  /* public static double signedCube(double a) {
    if (a < 0) {
      return -(a * a * a);
    }
    return a * a * a;
  } */

  // TODO: Change to this function if we cube response curve function


  /**
   * Calculate revolutions per minute, given a wheel size and a desired speed in
   * meters per second.
   * 
   * @return revolutions per minute that the given wheel would be spinning
   */
  public static double speedMetersPerSecondToRevsPerMinute(double diameterWheelSize, double metersPerSecond) {
    return (metersPerSecond * 60) / (diameterWheelSize * Math.PI * 0.0254); 
  }

  /**
   * Calculate distance travelled by a given wheel if it rotates a certain number
   * of degrees.
   * 
   * @return distance travelled by the wheel specified by diameter
   */
  public static double degreesToMeters(double diameterWheelSize, double degrees) {
    return (degrees / 360) * (4 * Math.PI * 0.0254); 
  }

  /**
   * circleSnappingDegrees will "snap" an angle to the nearest division, given as a number of divisions to create from a
   * circle.
   * 
   * For example, calling this function with
   * 
   * <pre>
   * circleSnappingDegrees(85, 4)
   * </pre>
   * 
   * will return 90, since 90 is the closest angle to 85 degrees that is a valid snapping value.
   * 
   * <p>
   * If the input angle is negative, it will be treated as a positive angle with as if it were subtracted from 360
   * degrees. This means that 320 degrees and -40 degrees will be treated identically, and would both snap to 0 if the
   * function was invoked with 4 divisions.
   * </p>
   * 
   * @param input
   * @param divisions a number of divisions to make of a circle. Note that this value *must* divide evenly into 360. 
   * This means that e.g. 4 is a valid input, but 13 is not. Using divisors that result in non-integer divisions of 360, 
   * the output of this function will be unstable.
   * @return
   */
  public static int circleSnappingDegrees(double input, int divisions) {
    // Make all angles positive
    if (input < 0) {
      input = 360.0 + input;
    }

    // Cap all angles to 360 degrees
    input %= 360.0;

    int degPerDivision = 360 / divisions;

    // Find the lower division bound
    int lowerDivision = ((int)input) / degPerDivision;

    // Find how far along the division this value is
    double divisionPosition = (input - (double) lowerDivision) / (double) degPerDivision;

    // If we're more than halfway thorugh the division, the closest is the next highest value.
    int snappedDivision = 0;
    if (divisionPosition > 0.5) {
      snappedDivision = lowerDivision + 1;
    } else {
      snappedDivision = lowerDivision;
    }

    // Make sure the value is at most 359
    return (snappedDivision * degPerDivision) % 360;
  }
}
