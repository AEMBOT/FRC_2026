package com.aembot.lib.math.geometry;

import static edu.wpi.first.units.Units.Meters;

import com.aembot.lib.math.geometry.structs.Translation2dMutStruct;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Objects;

public class Translation2dMut implements StructSerializable {
  public static final Translation2dMutStruct struct = new Translation2dMutStruct();

  private double xMeters;
  private double yMeters;

  /* ---- CONSTRUCTORS ---- */
  public Translation2dMut(double xMeters, double yMeters) {
    set(xMeters, yMeters);
  }

  public Translation2dMut() {
    this(0, 0);
  }

  public Translation2dMut(Translation2d other) {
    importImmutable(other);
  }

  public Translation2dMut(Translation2dMut other) {
    importMutable(other);
  }

  public Translation2dMut(double meters, Rotation2d angle) {
    set(meters, angle);
  }

  public Translation2dMut(double meters, Rotation2dMut angle) {
    set(meters, angle);
  }

  /**
   * Construct a Translation2dMut from a WPILib 2d Vector.
   *
   * @param vec The WPILib 2d Vector. Units are assumed to be meters.
   */
  public Translation2dMut(Vector<N2> vec) {
    importVector(vec);
  }

  /* ---- SETTERS ---- */
  /**
   * Set the x value of this Translation2dMut.
   *
   * @param xMeters The new x value in meters
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut setX(double xMeters) {
    this.xMeters = xMeters;
    return this;
  }

  /**
   * Set the x value of this Translation2dMut.
   *
   * @param x The new x value
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut setX(Distance x) {
    this.xMeters = x.in(Meters);
    return this;
  }

  /**
   * Set the x value of this Translation2dMut.
   *
   * @param x The new x value
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut setX(MutDistance x) {
    this.xMeters = x.in(Meters);
    return this;
  }

  /**
   * Set the y value of this Translation2dMut.
   *
   * @param yMeters The new y value in meters
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut setY(double yMeters) {
    this.yMeters = yMeters;
    return this;
  }

  /**
   * Set the y value of this Translation2dMut.
   *
   * @param y The new y value
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut setY(Distance y) {
    this.yMeters = y.in(Meters);
    return this;
  }

  /**
   * Set the y value of this Translation2dMut.
   *
   * @param y The new y value
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut setY(MutDistance y) {
    this.yMeters = y.in(Meters);
    return this;
  }

  /**
   * Set both the x and y values of this Translation2dMut.
   *
   * @param xMeters The new x value in meters
   * @param yMeters The new y value in meters
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut set(double xMeters, double yMeters) {
    this.xMeters = xMeters;
    this.yMeters = yMeters;
    return this;
  }

  /**
   * Set both the x and y values of this Translation2dMut.
   *
   * @param x The new x value
   * @param y The new y value
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut set(Distance x, Distance y) {
    this.xMeters = x.in(Meters);
    this.yMeters = y.in(Meters);
    return this;
  }

  /**
   * Set both the x and y values of this Translation2dMut.
   *
   * @param x The new x value
   * @param y The new y value
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut set(MutDistance x, MutDistance y) {
    this.xMeters = x.in(Meters);
    this.yMeters = y.in(Meters);
    return this;
  }

  public Translation2dMut set(double meters, Rotation2d angle) {
    return set(angle.getCos() * meters, angle.getSin() * meters);
  }

  public Translation2dMut set(double meters, Rotation2dMut angle) {
    return set(angle.getCos() * meters, angle.getSin() * meters);
  }

  public Translation2dMut importImmutable(Translation2d other) {
    return set(other.getX(), other.getY());
  }

  public Translation2dMut importMutable(Translation2dMut other) {
    return set(other.getX(), other.getY());
  }

  public Translation2dMut importVector(Vector<N2> vec) {
    return set(vec.get(0), vec.get(1));
  }

  /* ---- GETTERS ---- */
  public double getX() {
    return xMeters;
  }

  public double getY() {
    return yMeters;
  }

  /**
   * Returns a 2D translation vector representation of this translation.
   *
   * @return A 2D translation vector representation of this translation.
   */
  public Vector<N2> toVector() {
    return VecBuilder.fill(getX(), getY());
  }

  /** Create a new {@link Translation2d} with the same values as this mutable translation */
  public Translation2d exportImmutable() {
    return new Translation2d(getX(), getY());
  }

  /**
   * Make a deep copy of this Translation2dMut. Should be avoided if possible, as it creates a new
   * object.
   *
   * @see #importMutable(Translation2dMut)
   */
  public Translation2dMut makeCopy() {
    return new Translation2dMut(this);
  }

  /**
   * Returns the norm, or distance from the origin to the translation.
   *
   * @return The norm of the translation.
   */
  public double getNorm() {
    return Math.hypot(getX(), getY());
  }

  /**
   * Returns the squared norm, or squared distance from the origin to the translation. This is
   * equivalent to squaring the result of {@link #getNorm()}, but avoids computing a square root.
   *
   * @return The squared norm of the translation, in square meters.
   */
  public double getSquaredNorm() {
    return Math.pow(getX(), 2) + Math.pow(getY(), 2);
  }

  /**
   * Returns the angle this translation forms with the positive X axis.
   *
   * @return The angle of the translation in radians.
   */
  public double getAngleRadians() {
    return Math.atan2(getY(), getX());
  }

  /*
   * |------------------|
   * |--- OPERATIONS ---|
   * |------------------|
   */

  /**
   * Get the distance between this Translation2dMut and another Translation2d in meters.
   *
   * @return The distance in meters
   */
  public double getDistance(Translation2d other) {
    return Math.hypot(other.getX() - getX(), other.getY() - getY());
  }

  /**
   * Get the distance between this Translation2dMut and another Translation2dMut in meters.
   *
   * @return The distance in meters
   */
  public double getDistance(Translation2dMut other) {
    return Math.hypot(other.getX() - getX(), other.getY() - getY());
  }

  /**
   * Calculates the square of the distance between two translations in 2D space. This is equivalent
   * to squaring the result of {@link #getDistance(Translation2d)}, but avoids computing a square
   * root.
   *
   * <p>The square of the distance between translations is defined as (x₂−x₁)²+(y₂−y₁)².
   *
   * @param other The translation to compute the squared distance to.
   * @return The square of the distance between the two translations, in square meters.
   */
  public double getSquaredDistance(Translation2d other) {
    double dx = other.getX() - getX();
    double dy = other.getY() - getY();
    return dx * dx + dy * dy;
  }

  /**
   * Calculates the square of the distance between two translations in 2D space. This is equivalent
   * to squaring the result of {@link #getDistance(Translation2dMut)}, but avoids computing a square
   * root.
   *
   * <p>The square of the distance between translations is defined as (x₂−x₁)²+(y₂−y₁)².
   *
   * @param other The translation to compute the squared distance to.
   * @return The square of the distance between the two translations, in square meters.
   */
  public double getSquaredDistance(Translation2dMut other) {
    double dx = other.getX() - getX();
    double dy = other.getY() - getY();
    return dx * dx + dy * dy;
  }

  /**
   * Applies a rotation to the translation in 2D space.
   *
   * <p>This multiplies the translation vector by a counterclockwise rotation matrix of the given
   * angle.
   *
   * <pre>
   * [x_new]   [other.cos, -other.sin][x]
   * [y_new] = [other.sin,  other.cos][y]
   * </pre>
   *
   * <p>For example, rotating a Translation2d of &lt;2, 0&gt; by 90 degrees will return a
   * Translation2d of &lt;0, 2&gt;.
   *
   * @param other The rotation to rotate the translation by.
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut rotateBy(Rotation2d other) {
    return set(
        getX() * other.getCos() - getY() * other.getSin(),
        getX() * other.getSin() + getY() * other.getCos());
  }

  /**
   * Applies a rotation to the translation in 2D space.
   *
   * <p>This multiplies the translation vector by a counterclockwise rotation matrix of the given
   * angle.
   *
   * <pre>
   * [x_new]   [other.cos, -other.sin][x]
   * [y_new] = [other.sin,  other.cos][y]
   * </pre>
   *
   * <p>For example, rotating a Translation2d of &lt;2, 0&gt; by 90 degrees will return a
   * Translation2d of &lt;0, 2&gt;.
   *
   * @param other The rotation to rotate the translation by.
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut rotateBy(Rotation2dMut other) {
    return set(
        getX() * other.getCos() - getY() * other.getSin(),
        getX() * other.getSin() + getY() * other.getCos());
  }

  /**
   * Rotates this translation around another translation in 2D space.
   *
   * <pre>
   * [x_new]   [rot.cos, -rot.sin][x - other.x]   [other.x]
   * [y_new] = [rot.sin,  rot.cos][y - other.y] + [other.y]
   * </pre>
   *
   * @param other The other translation to rotate around.
   * @param rot The rotation to rotate the translation by.
   * @return The rotated translation as a new Translation2dMut for chaining
   */
  public Translation2dMut rotateAround(Translation2d other, Rotation2d rot) {
    return set(
        (getX() - other.getX()) * rot.getCos()
            - (getY() - other.getY()) * rot.getSin()
            + other.getX(),
        (getX() - other.getX()) * rot.getSin()
            + (getY() - other.getY()) * rot.getCos()
            + other.getY());
  }

  /**
   * Rotates this translation around another translation in 2D space.
   *
   * <pre>
   * [x_new]   [rot.cos, -rot.sin][x - other.x]   [other.x]
   * [y_new] = [rot.sin,  rot.cos][y - other.y] + [other.y]
   * </pre>
   *
   * @param other The other translation to rotate around.
   * @param rot The rotation to rotate the translation by.
   * @return The rotated translation as a new Translation2dMut for chaining
   */
  public Translation2dMut rotateAround(Translation2dMut other, Rotation2d rot) {
    return set(
        (getX() - other.getX()) * rot.getCos()
            - (getY() - other.getY()) * rot.getSin()
            + other.getX(),
        (getX() - other.getX()) * rot.getSin()
            + (getY() - other.getY()) * rot.getCos()
            + other.getY());
  }

  /**
   * Rotates this translation around another translation in 2D space.
   *
   * <pre>
   * [x_new]   [rot.cos, -rot.sin][x - other.x]   [other.x]
   * [y_new] = [rot.sin,  rot.cos][y - other.y] + [other.y]
   * </pre>
   *
   * @param other The other translation to rotate around.
   * @param rot The rotation to rotate the translation by.
   * @return The rotated translation as a new Translation2dMut for chaining
   */
  public Translation2dMut rotateAround(Translation2d other, Rotation2dMut rot) {
    return set(
        (getX() - other.getX()) * rot.getCos()
            - (getY() - other.getY()) * rot.getSin()
            + other.getX(),
        (getX() - other.getX()) * rot.getSin()
            + (getY() - other.getY()) * rot.getCos()
            + other.getY());
  }

  /**
   * Rotates this translation around another translation in 2D space.
   *
   * <pre>
   * [x_new]   [rot.cos, -rot.sin][x - other.x]   [other.x]
   * [y_new] = [rot.sin,  rot.cos][y - other.y] + [other.y]
   * </pre>
   *
   * @param other The other translation to rotate around.
   * @param rot The rotation to rotate the translation by.
   * @return The rotated translation as a new Translation2dMut for chaining
   */
  public Translation2dMut rotateAround(Translation2dMut other, Rotation2dMut rot) {
    return set(
        (getX() - other.getX()) * rot.getCos()
            - (getY() - other.getY()) * rot.getSin()
            + other.getX(),
        (getX() - other.getX()) * rot.getSin()
            + (getY() - other.getY()) * rot.getCos()
            + other.getY());
  }

  /**
   * Computes the dot product between this translation and another translation in 2D space.
   *
   * <p>The dot product between two translations is defined as x₁x₂+y₁y₂.
   *
   * @param other The translation to compute the dot product with.
   * @return The dot product between the two translations, in square meters.
   */
  public double dot(Translation2d other) {
    return getX() * other.getX() + getY() * other.getY();
  }

  /**
   * Computes the dot product between this translation and another translation in 2D space.
   *
   * <p>The dot product between two translations is defined as x₁x₂+y₁y₂.
   *
   * @param other The translation to compute the dot product with.
   * @return The dot product between the two translations, in square meters.
   */
  public double dot(Translation2dMut other) {
    return getX() * other.getX() + getY() * other.getY();
  }

  /**
   * Computes the cross product between this translation and another translation in 2D space.
   *
   * <p>The 2D cross product between two translations is defined as x₁y₂-x₂y₁.
   *
   * @param other The translation to compute the cross product with.
   * @return The cross product between the two translations, in square meters.
   */
  public double cross(Translation2d other) {
    return getX() * other.getY() - getY() * other.getX();
  }

  /**
   * Computes the cross product between this translation and another translation in 2D space.
   *
   * <p>The 2D cross product between two translations is defined as x₁y₂-x₂y₁.
   *
   * @param other The translation to compute the cross product with.
   * @return The cross product between the two translations, in square meters.
   */
  public double cross(Translation2dMut other) {
    return getX() * other.getY() - getY() * other.getX();
  }

  /**
   * Add a translation to this Translation2dMut.
   *
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut add(Translation2d other) {
    return set(getX() + other.getX(), getY() + other.getY());
  }

  /**
   * Add a translation to this Translation2dMut.
   *
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut add(Translation2dMut other) {
    return set(getX() + other.getX(), getY() + other.getY());
  }

  /**
   * Subtract a translation from this Translation2dMut.
   *
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut subtract(Translation2d other) {
    return set(getX() - other.getX(), getY() - other.getY());
  }

  /**
   * Subtract a translation from this Translation2dMut.
   *
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut subtract(Translation2dMut other) {
    return set(getX() - other.getX(), getY() - other.getY());
  }

  /**
   * Returns the inverse of the current translation. This is equivalent to rotating by 180 degrees,
   * flipping the point over both axes, or negating all components of the translation.
   *
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut unarySubtract() {
    return set(-getX(), -getY());
  }

  /**
   * Returns the translation multiplied by a scalar.
   *
   * <p>For example, Translation2d(2.0, 2.5) * 2 = Translation2d(4.0, 5.0).
   *
   * @param scalar The scalar to multiply by.
   * @return This Translation2dMut for chaining
   */
  public Translation2dMut multiply(double scalar) {
    return set(getX() * scalar, getY() * scalar);
  }

  /**
   * Returns the translation divided by a scalar.
   *
   * <p>For example, Translation3d(2.0, 2.5) / 2 = Translation3d(1.0, 1.25).
   *
   * @param scalar The scalar to multiply by.
   * @return The reference to the new mutated object.
   */
  public Translation2dMut divide(double scalar) {
    return set(getX() / scalar, getY() / scalar);
  }

  /**
   * Returns the nearest Translation2d from a collection of translations.
   *
   * @param translations The collection of translations.
   * @return The nearest Translation2d from the collection.
   */
  // this needs to be named differently bcuz apparently Java doesn't let you do overloads w/
  // generics
  public Translation2d nearestImmutable(Collection<Translation2d> translations) {
    return Collections.min(translations, Comparator.comparing(this::getDistance));
  }

  /**
   * Returns the nearest Translation2d from a collection of translations.
   *
   * @param translations The collection of translations.
   * @return The nearest Translation2d from the collection.
   */
  public Translation2dMut nearest(Collection<Translation2dMut> translations) {
    return Collections.min(translations, Comparator.comparing(this::getDistance));
  }

  /*
   * |------------------------|
   * |--- OBJECT OVERRIDES ---|
   * |------------------------|
   */

  @Override
  public String toString() {
    return String.format("Translation2dMut(X: %.2f, Y: %.2f)", getX(), getY());
  }

  /**
   * Checks equality between this Translation2d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    return obj instanceof Translation2dMut other
        && Math.abs(other.getX() - getX()) < 1E-9
        && Math.abs(other.getY() - getY()) < 1E-9;
  }

  @Override
  public int hashCode() {
    return Objects.hash(getX(), getY());
  }
}
