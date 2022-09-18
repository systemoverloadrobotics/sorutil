package frc.sorutil.interpolate;

import java.util.TreeMap;
import frc.sorutil.SorMath;

/**
 * Interpolator implements an interpolation algorithm that linearly interpolates between points fed to it.
 * 
 * For example, if the input to the Interpolator were:
 * <ul>
 * <li>(100, 1000)</li>
 * <li>(200, 2000)</li>
 * <li>(300, 4000)</li>
 * </ul>
 * 
 * A call to interpolate with the input 150 would be expected to return 1500, and a call with 250 would be expected to
 * return 3000.
 * 
 * <p>
 * Useful for creating lookup tables with a fallback to interpolation for things like distance to target vs. RPM, or
 * other similar lookups.
 * </p>
 */
public class Interpolator extends TreeMap<Double, Double> {
  /**
   * Interpolate between known values, producing a new value using a linear relationship between the nearest points. The
   * input must be defined, however the result may be null if this is unable to evaluate a suitable output.
   * 
   * @param input a value to use as the input to the mapping
   * @return an interpolated value, or null if interpolation failed.
   */
  public Double interpolate(double input) {
    if (this.get(input) != null) {
      return this.get(input);
    }

    var floor = this.lowerEntry(input);
    var ceiling = this.higherEntry(input);

    if (floor == null) {
      return null;
    }
    if (ceiling == null) {
      return null;
    }

    return SorMath.linearInterpolate(input, floor.getKey(), ceiling.getKey(), floor.getValue(), ceiling.getValue());
  }

  public Double put(int a0, int a1) {
    return this.put((double) a0, (double) a1);
  }
}
