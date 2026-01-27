package com.aembot.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.concurrent.ConcurrentNavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

/**
 * Concurrent Map that implements bilinear interpolation
 *
 * <p>NOTE: Meant to be used for data structured as a grid, will not function well for a
 * distribution of random points
 */
public class ConcurrentInterpolatable2DMap<T> {

  private final Interpolator<T> interpolatingFunc;

  private final ConcurrentNavigableMap<Double, ConcurrentNavigableMap<Double, T>> internalMap =
      new ConcurrentSkipListMap<>();

  public ConcurrentInterpolatable2DMap(Interpolator<T> interpolatingFunc) {
    this.interpolatingFunc = interpolatingFunc;
  }

  /**
   * Create a new ConcurrentInterpolatable2DMap.
   *
   * @param interpolatingFunc The funtion used to interpolate between values.
   * @return The new ConcurrentInterpolatable2DMap.
   */
  public static <T> ConcurrentInterpolatable2DMap<T> createMap(Interpolator<T> interpolatingFunc) {
    return new ConcurrentInterpolatable2DMap<>(interpolatingFunc);
  }

  /**
   * Create a new ConcurrentInterpolatable2DMap.
   *
   * @return The new ConcurrentInterpolatable2DMap.
   */
  public static <T extends Interpolatable<T>> ConcurrentInterpolatable2DMap<T> createMap() {
    return new ConcurrentInterpolatable2DMap<>(Interpolatable::interpolate);
  }

  /**
   * Create a new ConcurrentInterpolatable2DMap for Double values.
   *
   * @return The new ConcurrentInterpolatable2DMap.
   */
  public static ConcurrentInterpolatable2DMap<Double> createDoubleMap() {
    return new ConcurrentInterpolatable2DMap<>(MathUtil::interpolate);
  }

  /**
   * Add a point to the map.
   *
   * @param q1Key Key for the value in the first list
   * @param q2Key Key for the value in the second list
   * @param value Value to map to the keys
   */
  public void addPoint(double q1Key, double q2Key, T value) {
    if (!internalMap.containsKey(q1Key)) {
      internalMap.put(q1Key, new ConcurrentSkipListMap<>());
    }
    internalMap.get(q1Key).put(q2Key, value);
  }

  /**
   * Sample the map at the given point. If the list is empty, return an empty Optional.
   *
   * @param q1Key Key for the sample point in the first list
   * @param q2Key Key for the sample point in the second list
   * @return The interpolated value at that point, or an empty Optional.
   */
  public Optional<T> getPoint(double q1Key, double q2Key) {
    if (internalMap.isEmpty()) {
      return Optional.empty();
    }

    // possibly need to check if point is exactly on map (?)

    Entry<Double, T> q11 = internalMap.floorEntry(q1Key).getValue().floorEntry(q2Key);
    Entry<Double, T> q12 = internalMap.floorEntry(q1Key).getValue().ceilingEntry(q2Key);
    Entry<Double, T> q21 = internalMap.ceilingEntry(q1Key).getValue().floorEntry(q2Key);
    Entry<Double, T> q22 = internalMap.ceilingEntry(q1Key).getValue().ceilingEntry(q2Key);

    Double q1InterpolationTime = (q1Key - q11.getKey()) / (q21.getKey() - q11.getKey());
    Double q2InterpolationTime = (q2Key - q11.getKey()) / (q12.getKey() - q11.getKey());

    T floorInterpolatedQ1Value =
        interpolatingFunc.interpolate(q11.getValue(), q12.getValue(), q1InterpolationTime);
    T ceilingInterpolatedQ1Value =
        interpolatingFunc.interpolate(q21.getValue(), q22.getValue(), q1InterpolationTime);

    return Optional.of(
        interpolatingFunc.interpolate(
            floorInterpolatedQ1Value, ceilingInterpolatedQ1Value, q2InterpolationTime));
  }

  /**
   * @return The internal 2D map used to store points
   */
  public ConcurrentNavigableMap<Double, ConcurrentNavigableMap<Double, T>> getInternalMap() {
    return internalMap;
  }
}
