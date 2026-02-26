package com.aembot.lib.math.geometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

import com.aembot.lib.math.geometry.structs.QuaternionMutStruct;

/** Mutable representation of a quaternion */
public class QuaternionMut implements StructSerializable {
  public static final QuaternionMutStruct stuct = new QuaternionMutStruct();

  // Scalar r in versor form
  private double w;

  // Vector v in versor form
  private double x;
  private double y;
  private double z;

  /*
   * |--------------------|
   * |--- CONSTRUCTORS ---|
   * |--------------------|
   */

  /**
   * Constructs a quaternion with the given components.
   *
   * @param w W component of the quaternion.
   * @param x X component of the quaternion.
   * @param y Y component of the quaternion.
   * @param z Z component of the quaternion.
   */
  public QuaternionMut(double w, double x, double y, double z) {
    this.set(w, x, y, z);
  }

  /** Constructs a quaternion with a default angle of 0 degrees. */
  public QuaternionMut() {
    this(1, 0, 0, 0);
  }

  public QuaternionMut(Quaternion value) {
    this.importImmutable(value);
  }

  public QuaternionMut(QuaternionMut value) {
    this.importMutable(value);
  }

  /*
   * |---------------|
   * |--- SETTERS ---|
   * |---------------|
   */

  public QuaternionMut setW(double w) {
    this.w = w;
    return this;
  }

  public QuaternionMut setX(double x) {
    this.x = x;
    return this;
  }

  public QuaternionMut setY(double y) {
    this.y = y;
    return this;
  }

  public QuaternionMut setZ(double z) {
    this.z = z;
    return this;
  }

  public QuaternionMut set(double w, double x, double y, double z) {
    return this.setW(w).setX(x).setY(y).setZ(z);
  }

  public QuaternionMut importImmutable(Quaternion newValue) {
    return this.set(newValue.getW(), newValue.getX(), newValue.getY(), newValue.getZ());
  }

  public QuaternionMut importMutable(QuaternionMut newValue) {
    return this.set(newValue.getW(), newValue.getX(), newValue.getY(), newValue.getZ());
  }

  /*
   * |---------------|
   * |--- GETTERS ---|
   * |---------------|
   */
  public double getW() {
    return w;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getZ() {
    return z;
  }

  public Quaternion toImmutable() {
    return new Quaternion(w, x, y, z);
  }

  public QuaternionMut makeCopy() {
    return new QuaternionMut(w, x, y, z);
  }

  /*
   * |------------------|
   * |--- OPERATIONS ---|
   * |------------------|
   */

  /**
   * Adds another quaternion to this quaternion entrywise.
   *
   * @param other The other quaternion.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut add(Quaternion other) {
    return setW(this.getW() + other.getW())
        .setX(this.getX() + other.getX())
        .setY(this.getY() + other.getY())
        .setZ(this.getZ() + other.getZ());
  }

  /**
   * Subtracts another quaternion from this quaternion entrywise.
   *
   * @param other The other quaternion.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut subtract(Quaternion other) {
    return setW(this.getW() - other.getW())
        .setX(this.getX() - other.getX())
        .setY(this.getY() - other.getY())
        .setZ(this.getZ() - other.getZ());
  }

  /**
   * Adds another quaternion to this quaternion entrywise.
   *
   * @param other The other quaternion.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut add(QuaternionMut other) {
    return setW(this.getW() + other.getW())
        .setX(this.getX() + other.getX())
        .setY(this.getY() + other.getY())
        .setZ(this.getZ() + other.getZ());
  }

  /**
   * Subtracts another quaternion from this quaternion entrywise.
   *
   * @param other The other quaternion.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut subtract(QuaternionMut other) {
    return setW(this.getW() - other.getW())
        .setX(this.getX() - other.getX())
        .setY(this.getY() - other.getY())
        .setZ(this.getZ() - other.getZ());
  }

  /**
   * Multiplies with a scalar.
   *
   * @param scalar The value to scale each component by.
   * @return The scaled quaternion.
   */
  public QuaternionMut multiply(double scalar) {
    return setW(getW() * scalar).setX(getX() * scalar).setY(getY() * scalar).setZ(getZ() * scalar);
  }

  /**
   * Divides by a scalar.
   *
   * @param scalar The value to divide each component by.
   * @return The scaled quaternion.
   */
  public QuaternionMut divide(double scalar) {
    return this.multiply(1.0 / scalar);
  }

  /**
   * Multiply by another quaternion.
   *
   * @param other The other quaternion.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut multiply(Quaternion other) {
    // https://en.wikipedia.org/wiki/Quaternion#Scalar_and_vector_parts
    final double r1 = getW();
    final double r2 = other.getW();

    // v₁ ⋅ v₂
    double dot = getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ();
    // v₁ x v₂
    double cross_x = getY() * other.getZ() - other.getY() * getZ();
    double cross_y = other.getX() * getZ() - getX() * other.getZ();
    double cross_z = getX() * other.getY() - other.getX() * getY();

    return this.setW(r1 * r2 - dot)
        .setX(r1 * other.getX() + r2 * getX() + cross_x)
        .setY(r1 * other.getY() + r2 * getY() + cross_y)
        .setZ(r1 * other.getZ() + r2 * getZ() + cross_z);
  }

  /**
   * Multiply by another quaternion.
   *
   * @param other The other quaternion.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut multiply(QuaternionMut other) {
    // https://en.wikipedia.org/wiki/Quaternion#Scalar_and_vector_parts
    final var r1 = getW();
    final var r2 = other.getW();

    // v₁ ⋅ v₂
    double dot = getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ();
    // v₁ x v₂
    double cross_x = getY() * other.getZ() - other.getY() * getZ();
    double cross_y = other.getX() * getZ() - getX() * other.getZ();
    double cross_z = getX() * other.getY() - other.getX() * getY();

    return this.setW(r1 * r2 - dot)
        .setX(r1 * other.getX() + r2 * getX() + cross_x)
        .setY(r1 * other.getY() + r2 * getY() + cross_y)
        .setZ(r1 * other.getZ() + r2 * getZ() + cross_z);
  }

  /**
   * Pre-multiplies this quaternion by another: sets this = other * this.
   *
   * @param other The left-hand quaternion.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut premultiply(Quaternion other) {
    final double r1 = other.getW();
    final double r2 = getW();

    double dot = other.getX() * getX() + other.getY() * getY() + other.getZ() * getZ();

    double cross_x = other.getY() * getZ() - getY() * other.getZ();
    double cross_y = getX() * other.getZ() - other.getX() * getZ();
    double cross_z = other.getX() * getY() - getX() * other.getY();

    return this.setW(r1 * r2 - dot)
        .setX(r1 * getX() + r2 * other.getX() + cross_x)
        .setY(r1 * getY() + r2 * other.getY() + cross_y)
        .setZ(r1 * getZ() + r2 * other.getZ() + cross_z);
  }

  /**
   * Pre-multiplies this quaternion by another: sets this = other * this.
   *
   * @param other The left-hand quaternion.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut premultiply(QuaternionMut other) {
    final double r1 = other.getW();
    final double r2 = getW();

    double dot = other.getX() * getX() + other.getY() * getY() + other.getZ() * getZ();

    double cross_x = other.getY() * getZ() - getY() * other.getZ();
    double cross_y = getX() * other.getZ() - other.getX() * getZ();
    double cross_z = other.getX() * getY() - getX() * other.getY();

    return this.setW(r1 * r2 - dot)
        .setX(r1 * getX() + r2 * other.getX() + cross_x)
        .setY(r1 * getY() + r2 * other.getY() + cross_y)
        .setZ(r1 * getZ() + r2 * other.getZ() + cross_z);
  }

  /**
   * Conjugate this quaternion
   *
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut conjugate() {
    return this.setX(-getX()).setY(-getY()).setZ(-getZ());
  }

  /**
   * Returns the elementwise product of two quaternions.
   *
   * @param other The other quaternion.
   * @return The dot product of two quaternions.
   */
  public double dot(final Quaternion other) {
    return getW() * other.getW()
        + getX() * other.getX()
        + getY() * other.getY()
        + getZ() * other.getZ();
  }

  /**
   * Returns the elementwise product of two quaternions.
   *
   * @param other The other quaternion.
   * @return The dot product of two quaternions.
   */
  public double dot(final QuaternionMut other) {
    return getW() * other.getW()
        + getX() * other.getX()
        + getY() * other.getY()
        + getZ() * other.getZ();
  }

  /**
   * Calculates the L2 norm of the quaternion.
   *
   * @return The L2 norm.
   */
  public double norm() {
    return Math.sqrt(dot(this));
  }

  /**
   * Returns the inverse of the quaternion.
   *
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut invert() {
    var norm = norm();
    return this.conjugate().divide(norm * norm);
  }

  /**
   * Normalizes the quaternion.
   *
   * @return The normalized quaternion.
   */
  public QuaternionMut normalize() {
    double norm = norm();
    if (norm == 0.0) {
      return this;
    } else {
      return this.setW(getW() / norm).setX(getX() / norm).setY(getY() / norm).setZ(getZ() / norm);
    }
  }

  /**
   * The Log operator of a general quaternion. Sets this quaternion to the result of the log
   * operation
   *
   * <p>source: wpimath/algorithms.md
   *
   * <p>If this quaternion is in SO(3) and you are looking for an element of 𝖘𝖔(3), use {@link
   * #toRotationVector}
   *
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut log() {
    var norm = norm();
    var scalar = Math.log(norm);

    var v_norm = Math.sqrt(getX() * getX() + getY() * getY() + getZ() * getZ());

    var s_norm = getW() / norm;

    if (Math.abs(s_norm + 1) < 1e-9) {
      // return new Quaternion(scalar, -Math.PI, 0, 0);
      return this.setW(scalar).setX(-Math.PI).setY(0).setZ(0);
    }

    double v_scalar;

    if (v_norm < 1e-9) {
      // Taylor series expansion of atan2(y/x)/y at y = 0:
      //
      //   1/x - 1/3 y²/x³ + O(y⁴)
      v_scalar = 1.0 / getW() - 1.0 / 3.0 * v_norm * v_norm / (getW() * getW() * getW());
    } else {
      v_scalar = Math.atan2(v_norm, getW()) / v_norm;
    }

    // return new Quaternion(scalar, v_scalar * getX(), v_scalar * getY(), v_scalar * getZ());
    return this.setW(scalar)
        .setX(getX() * v_scalar)
        .setY(getY() * v_scalar)
        .setZ(getZ() * v_scalar);
  }

  /**
   * Log operator of a quaternion. Sets this QuaternionMut to the "Twist" that maps this quaternion
   * to the argument.
   *
   * @param end The quaternion to map this quaternion onto.
   * @return this QuaternionMut for chaining
   */
  public QuaternionMut log(Quaternion end) {
    // return end.times(this.inverse()).log();
    return this.invert().premultiply(end).log();
  }

  /**
   * Log operator of a quaternion. Sets this QuaternionMut to the "Twist" that maps this quaternion
   * to the argument.
   *
   * @param end The quaternion to map this quaternion onto.
   * @return this QuaternionMut for chaining
   */
  public QuaternionMut log(QuaternionMut end) {
    // return end.times(this.inverse()).log();
    return this.invert().premultiply(end).log();
  }

  /**
   * Set this QuaternionMut to the matrix exponential of its current value.
   *
   * <p>source: wpimath/algorithms.md
   *
   * <p>If this quaternion is in 𝖘𝖔(3) and you are looking for an element of SO(3), use {@link
   * #fromRotationVector}
   *
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut exp() {
    double scalar = Math.exp(getW());

    double axial_magnitude = Math.sqrt(getX() * getX() + getY() * getY() + getZ() * getZ());
    double cosine = Math.cos(axial_magnitude);

    double axial_scalar;

    if (axial_magnitude < 1e-9) {
      // Taylor series of sin(θ) / θ near θ = 0: 1 − θ²/6 + θ⁴/120 + O(n⁶)
      double axial_magnitude_sq = axial_magnitude * axial_magnitude;
      double axial_magnitude_sq_sq = axial_magnitude_sq * axial_magnitude_sq;
      axial_scalar = 1.0 - axial_magnitude_sq / 6.0 + axial_magnitude_sq_sq / 120.0;
    } else {
      axial_scalar = Math.sin(axial_magnitude) / axial_magnitude;
    }

    return this.set(
        cosine * scalar,
        getX() * axial_scalar * scalar,
        getY() * axial_scalar * scalar,
        getZ() * axial_scalar * scalar);
  }

  /**
   * Set this QuaternionMut to the quaternion product of exp(adjustment) * this
   *
   * @param adjustment the "Twist" that will be applied to this quaternion.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut exp(Quaternion adjustment) {
    // return adjustment.exp().times(this);

    // This allocation kinda sucks, but the only solution I can think of is just doing the math here
    // and I really don't wanna do that... I kinda doubt we're gonna use this method anyways though,
    // so...
    return this.importMutable(new QuaternionMut(adjustment).exp().multiply(this));
  }

  /**
   * Set this QuaternionMut to the quaternion product of exp(adjustment) * this
   *
   * @param adjustment the "Twist" that will be applied to this quaternion.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut exp(QuaternionMut adjustment) {
    // return adjustment.exp().times(this);

    // This allocation kinda sucks, but the only solution I can think of is just doing the math here
    // and I really don't wanna do that... I kinda doubt we're gonna use this method anyways though,
    // so...
    return this.importMutable(adjustment.makeCopy().exp().multiply(this));
  }

  /**
   * Set this QuaternionMut to the rational power of its current value.
   *
   * @param t the power to raise this quaternion to.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut pow(double t) {
    // q^t = e^(ln(q^t)) = e^(t * ln(q))

    // return this.log().times(t).exp();
    return this.log().multiply(t).exp();
  }

  /**
   * Import a rotation vector as this quaternion's value
   *
   * <p>This is also the exp operator of 𝖘𝖔(3).
   *
   * <p>source: wpimath/algorithms.md
   *
   * @param rvec The rotation vector.
   * @return This QuaternionMut for chaining
   */
  public QuaternionMut importRotationVector(Vector<N3> rvec) {
    double theta = rvec.norm();

    double cos = Math.cos(theta / 2);

    double axial_scalar;

    if (theta < 1e-9) {
      // taylor series expansion of sin(θ/2) / θ = 1/2 - θ²/48 + O(θ⁴)
      axial_scalar = 1.0 / 2.0 - theta * theta / 48.0;
    } else {
      axial_scalar = Math.sin(theta / 2) / theta;
    }

    return this.set(
        cos,
        axial_scalar * rvec.get(0, 0),
        axial_scalar * rvec.get(1, 0),
        axial_scalar * rvec.get(2, 0));
  }

  /**
   * Returns the quaternion representation of this rotation vector.
   *
   * <p>This is also the exp operator of 𝖘𝖔(3).
   *
   * <p>source: wpimath/algorithms.md
   *
   * @param rvec The rotation vector.
   * @return The quaternion representation of this rotation vector.
   */
  public static QuaternionMut fromRotationVector(Vector<N3> rvec) {
    return new QuaternionMut().importRotationVector(rvec);
  }

  /**
   * Returns the rotation vector representation of this quaternion.
   *
   * <p>This is also the log operator of SO(3).
   *
   * @return The rotation vector representation of this quaternion.
   */
  public Vector<N3> toRotationVector() {
    // See equation (31) in "Integrating Generic Sensor Fusion Algorithms with
    // Sound State Representation through Encapsulation of Manifolds"
    //
    // https://arxiv.org/pdf/1107.1119.pdf
    double norm = Math.sqrt(getX() * getX() + getY() * getY() + getZ() * getZ());

    double coeff;
    if (norm < 1e-9) {
      coeff = 2.0 / getW() - 2.0 / 3.0 * norm * norm / (getW() * getW() * getW());
    } else {
      if (getW() < 0.0) {
        coeff = 2.0 * Math.atan2(-norm, -getW()) / norm;
      } else {
        coeff = 2.0 * Math.atan2(norm, getW()) / norm;
      }
    }

    return VecBuilder.fill(coeff * getX(), coeff * getY(), coeff * getZ());
  }

  /*
   * |------------------------|
   * |--- OBJECT OVERRIDES ---|
   * |------------------------|
   */
  @Override
  public String toString() {
    return String.format("QuaternionMut(%s, %s, %s, %s)", getW(), getX(), getY(), getZ());
  }

  /**
   * Checks equality between this Quaternion and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    return obj instanceof QuaternionMut other
        && Math.abs(dot(other) - norm() * other.norm()) < 1e-9
        && Math.abs(norm() - other.norm()) < 1e-9;
  }

  @Override
  public int hashCode() {
    return Objects.hash(getW(), getX(), getY(), getZ());
  }
}
