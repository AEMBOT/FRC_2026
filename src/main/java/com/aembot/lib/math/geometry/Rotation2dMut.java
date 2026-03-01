package com.aembot.lib.math.geometry;

import static edu.wpi.first.units.Units.Radians;

import com.aembot.lib.math.geometry.structs.Rotation2dMutStruct;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

/** A mutable version of WPILib's {@link Rotation2d}. Many methods return self for chaining. */
public class Rotation2dMut implements StructSerializable {
  public static final Rotation2dMutStruct struct = new Rotation2dMutStruct();

  private double valueRadians;
  private double cos;
  private double sin;

  /*
   * |--------------------|
   * |--- CONSTRUCTORS ---|
   * |--------------------|
   */
  public Rotation2dMut(double radians) {
    setRadians(radians);
  }

  public Rotation2dMut() {
    this(0);
  }

  public Rotation2dMut(Rotation2d from) {
    this(from.getRadians());
  }

  public Rotation2dMut(Rotation2dMut from) {
    this(from.getRadians());
  }

  /**
   * Constructs a Rotation2dMut with the given x and y (cosine and sine) components.
   *
   * @param x The x component or cosine of the rotation.
   * @param y The y component or sine of the rotation.
   */
  public Rotation2dMut(double x, double y) {
    setXY(x, y);
  }

  /**
   * Constructs a Rotation2dMut with the given angle.
   *
   * @param angle The angle of the rotation.
   */
  public Rotation2dMut(Angle angle) {
    this(angle.in(Radians));
  }

  /**
   * Constructs a Rotation2dMut with the given mutable angle.
   *
   * @param angle The mutable angle of the rotation.
   */
  public Rotation2dMut(MutAngle angle) {
    this(angle.in(Radians));
  }

  /**
   * Constructs a Rotation2dMut from a rotation matrix.
   *
   * @param rotationMatrix The rotation matrix.
   * @throws IllegalArgumentException if the rotation matrix isn't special orthogonal.
   */
  public Rotation2dMut(Matrix<N2, N2> rotationMatrix) {
    setFromRotationMatrix(rotationMatrix);
  }

  private void updateSinCos() {
    this.cos = Math.cos(getRadians());
    this.sin = Math.sin(getRadians());
  }

  /*
   * |---------------|
   * |--- SETTERS ---|
   * |---------------|
   */

  /**
   * Set the value of this Rotation2dMut in radians. Returns self for chaining.
   *
   * @return this Rotation2dMut
   */
  public Rotation2dMut setRadians(double radians) {
    this.valueRadians = radians;
    updateSinCos();
    return this;
  }

  /**
   * Set the value of this Rotation2dMut in degrees. Returns self for chaining.
   *
   * @return this Rotation2dMut
   */
  public Rotation2dMut setDegrees(double degrees) {
    setRadians(Math.toRadians(degrees));
    return this;
  }

  /**
   * Set the value of this Rotation2dMut in rotations. Returns self for chaining.
   *
   * @return this Rotation2dMut
   */
  public Rotation2dMut setRotations(double rotations) {
    setRadians(Units.rotationsToRadians(rotations));
    return this;
  }

  /**
   * Set the value of this Rotation2dMut using an Angle object. Returns self for chaining.
   *
   * @return this Rotation2dMut
   */
  public Rotation2dMut setAngle(Angle angle) {
    setRadians(angle.in(Radians));
    return this;
  }

  /**
   * Set the value of this Rotation2dMut using a MutAngle object. Returns self for chaining.
   *
   * @return this Rotation2dMut
   */
  public Rotation2dMut setMutAngle(MutAngle angle) {
    setRadians(angle.in(Radians));
    return this;
  }

  /** Set value with the given x and y (cosine and sine) components */
  public Rotation2dMut setXY(double x, double y) {
    double magnitude = Math.hypot(x, y);
    if (magnitude > 1e-6) {
      setRadians(Math.atan2(y / magnitude, x / magnitude));
    } else {
      setRadians(0.0);
      MathSharedStore.reportError(
          "x and y components of Rotation2dMut are zero\n", Thread.currentThread().getStackTrace());
    }
    return this;
  }

  /**
   * Set the value of this Rotation2dMut from a rotation matrix. Returns self for chaining.
   *
   * @param rotationMatrix The rotation matrix.
   * @throws IllegalArgumentException if the rotation matrix isn't special orthogonal.
   */
  public Rotation2dMut setFromRotationMatrix(Matrix<N2, N2> rotationMatrix) {
    final var R = rotationMatrix;

    // Require that the rotation matrix is special orthogonal. This is true if
    // the matrix is orthogonal (RRᵀ = I) and normalized (determinant is 1).
    if (R.times(R.transpose()).minus(Matrix.eye(Nat.N2())).normF() > 1e-9) {
      var msg = "Rotation matrix isn't orthogonal\n\nR =\n" + R.getStorage().toString() + '\n';
      MathSharedStore.reportError(msg, Thread.currentThread().getStackTrace());
      throw new IllegalArgumentException(msg);
    }
    if (Math.abs(R.det() - 1.0) > 1e-9) {
      var msg =
          "Rotation matrix is orthogonal but not special orthogonal\n\nR =\n"
              + R.getStorage().toString()
              + '\n';
      MathSharedStore.reportError(msg, Thread.currentThread().getStackTrace());
      throw new IllegalArgumentException(msg);
    }

    // R = [cosθ  −sinθ]
    //     [sinθ   cosθ]
    setRadians(Math.atan2(R.get(1, 0), R.get(0, 0)));

    return this;
  }

  /**
   * Import value from an immutable Rotation2d. Returns self for chaining.
   *
   * @return this Rotation2dMut
   */
  public Rotation2dMut importImmutable(Rotation2d from) {
    this.valueRadians = from.getRadians();
    this.cos = from.getCos();
    this.sin = from.getSin();
    return this;
  }

  /**
   * Import the value from another Rotation2dMut. Useful for importing one rotation to another to
   * perform operations on it while retaining values on the other.
   *
   * @return this Rotation2dMut
   */
  public Rotation2dMut importMutable(Rotation2dMut from) {
    this.valueRadians = from.valueRadians;
    this.cos = from.cos;
    this.sin = from.sin;
    return this;
  }

  /**
   * Import z value from an immutable Rotation3d. Returns self for chaining.
   *
   * @return this Rotation2dMut
   */
  public Rotation2dMut importImmutable(Rotation3d from) {
    return this.setRadians(from.getZ());
  }

  /**
   * Import z value from a mutable Rotation3d. Returns self for chaining.
   *
   * @return this Rotation2dMut
   */
  public Rotation2dMut importMutable(Rotation3dMut from) {
    return this.setRadians(from.getZ());
  }

  /* ---- GETTERS ---- */
  public double getRadians() {
    return valueRadians;
  }

  public double getDegrees() {
    return Math.toDegrees(getRadians());
  }

  /** Get sine of the rotation angle. Note that this is cached when we update the angle. */
  public double getSin() {
    return sin;
  }

  /** Get cosine of the rotation angle. Note that this is cached when we update the angle. */
  public double getCos() {
    return cos;
  }

  /** Get tangent of the rotation angle. Note that this is cached when we update the angle. */
  public double getTan() {
    return sin / cos;
  }

  /** Export to a new immutable {@link Rotation2d} */
  public Rotation2d exportImmutable() {
    return new Rotation2d(getRadians());
  }

  /**
   * Make a deep copy of this Rotation2dMut. Should be avoided if possible, as it creates a new
   * object.
   *
   * @see #importMutable(Rotation2dMut)
   */
  public Rotation2dMut makeCopy() {
    return new Rotation2dMut(getRadians());
  }

  /**
   * Returns matrix representation of this rotation.
   *
   * @return Matrix representation of this rotation.
   */
  public Matrix<N2, N2> toMatrix() {
    // R = [cosθ  −sinθ]
    //     [sinθ   cosθ]
    return MatBuilder.fill(Nat.N2(), Nat.N2(), getCos(), -getSin(), getSin(), getCos());
  }

  /*
   * |------------------|
   * |--- OPERATIONS ---|
   * |------------------|
   */

  /**
   * Normalize the rotation to be within the range [-π, π].
   *
   * @return This Rotation2dMut for chaining
   */
  public Rotation2dMut normalize() {
    return setRadians(MathUtil.angleModulus(getRadians()));
  }

  /**
   * Adds the other rotation to the current rotation using a rotation matrix. The result will be
   * between -π and π radians.
   *
   * <p>The matrix multiplication is as follows:
   *
   * <pre>
   * [cos_new]   [other.cos, -other.sin][cos]
   * [sin_new] = [other.sin,  other.cos][sin]
   * value_new = atan2(sin_new, cos_new)
   * </pre>
   *
   * @param other The rotation to rotate by.
   * @return This Rotation2dMut for chaining
   */
  public Rotation2dMut add(Rotation2dMut other) {
    return setXY(
        getCos() * other.getCos() - getSin() * other.getSin(),
        getCos() * other.getSin() + getSin() * other.getCos());
  }

  /**
   * Adds the other rotation to the current rotation using a rotation matrix. The result will be
   * between -π and π radians.
   *
   * <p>The matrix multiplication is as follows:
   *
   * <pre>
   * [cos_new]   [other.cos, -other.sin][cos]
   * [sin_new] = [other.sin,  other.cos][sin]
   * value_new = atan2(sin_new, cos_new)
   * </pre>
   *
   * @param other The rotation to rotate by.
   * @return This Rotation2dMut for chaining
   */
  public Rotation2dMut add(Rotation2d other) {
    return setXY(
        getCos() * other.getCos() - getSin() * other.getSin(),
        getCos() * other.getSin() + getSin() * other.getCos());
  }

  /**
   * Adds the given angle in radians to the current rotation. The result will be between -π and π
   * radians.
   *
   * @param radians The angle in radians to add.
   * @return This Rotation2dMut for chaining
   */
  public Rotation2dMut add(double radians) {
    return setRadians(getRadians() + radians).normalize();
  }

  /**
   * Subtracts the other rotation from the current rotation using a rotation matrix. The result will
   * be between -π and π radians.
   *
   * <p>The matrix multiplication is as follows:
   *
   * <pre>
   * [cos_new]   [other.cos, other.sin][cos]
   * [sin_new] = [-other.sin,  other.cos][sin]
   * value_new = atan2(sin_new, cos_new)
   * </pre>
   *
   * @param other The rotation to rotate by.
   * @return This Rotation2dMut for chaining
   */
  public Rotation2dMut subtract(Rotation2dMut other) {
    return setXY(
        getCos() * other.getCos() + getSin() * other.getSin(),
        getCos() * other.getSin() - getSin() * other.getCos());
  }

  /**
   * Subtracts the other rotation from the current rotation using a rotation matrix. The result will
   * be between -π and π radians.
   *
   * <p>The matrix multiplication is as follows:
   *
   * <pre>
   * [cos_new]   [other.cos, other.sin][cos]
   * [sin_new] = [-other.sin,  other.cos][sin]
   * value_new = atan2(sin_new, cos_new)
   * </pre>
   *
   * @param other The rotation to rotate by.
   * @return This Rotation2dMut for chaining
   */
  public Rotation2dMut subtract(Rotation2d other) {
    return setXY(
        getCos() * other.getCos() + getSin() * other.getSin(),
        getCos() * other.getSin() - getSin() * other.getCos());
  }

  /**
   * Subtracts the given angle in radians from the current rotation. The result will be between -π
   * and π radians.
   *
   * @param radians The angle in radians to subtract.
   * @return This Rotation2dMut for chaining
   */
  public Rotation2dMut subtract(double radians) {
    return add(-radians);
  }

  /**
   * Inverts the rotation. This is simply the negative of the current angular value. Returns self
   * for chaining
   */
  public Rotation2dMut unarySubtract() {
    return setRadians(-getRadians());
  }

  /**
   * Multiplies the current rotation by a scalar.
   *
   * @param scalar The scalar.
   * @return This Rotation2dMut for chaining
   */
  public Rotation2dMut multiply(double scalar) {
    return setRadians(getRadians() * scalar);
  }

  /**
   * Divides the current rotation by a scalar.
   *
   * @param scalar The scalar.
   * @return This Rotation2dMut for chaining
   */
  public Rotation2dMut divide(double scalar) {
    return multiply(1.0 / scalar);
  }

  /**
   * Returns the current rotation relative to the given rotation.
   *
   * @param other The rotation describing the orientation of the new coordinate frame that the
   *     current rotation will be converted into.
   * @return The current rotation relative to the new orientation of the coordinate frame.
   */
  public Rotation2dMut relativeTo(Rotation2dMut other) {
    // literally just the subtract method, but it exists in Rotation2d & ig it's more inuitive?
    return subtract(other);
  }

  /**
   * Returns the current rotation relative to the given rotation.
   *
   * @param other The rotation describing the orientation of the new coordinate frame that the
   *     current rotation will be converted into.
   * @return The current rotation relative to the new orientation of the coordinate frame.
   */
  public Rotation2dMut relativeTo(Rotation2d other) {
    // literally just the subtract method, but it exists in Rotation2d & ig it's more inuitive?
    return subtract(other);
  }

  /**
   * Returns the current rotation relative to the given rotation.
   *
   * @param otherRadians The rotation describing the orientation of the new coordinate frame that
   *     the current rotation will be converted into.
   * @return The current rotation relative to the new orientation of the coordinate frame.
   */
  public Rotation2dMut relativeTo(double otherRadians) {
    // literally just the subtract method, but it exists in Rotation2d & ig it's more inuitive?
    return subtract(otherRadians);
  }

  /*
   * |------------------------|
   * |--- OBJECT OVERRIDES ---|
   * |------------------------|
   */

  @Override
  public String toString() {
    return String.format(
        "Rotation2dMut(Rads: %.2f, Deg: %.2f)", getRadians(), Math.toDegrees(getRadians()));
  }

  /**
   * Checks equality between this Rotation2dMut and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    return obj instanceof Rotation2dMut other
        && Math.hypot(getCos() - other.getCos(), getSin() - other.getSin()) < 1E-9;
  }

  @Override
  public int hashCode() {
    return Objects.hash(getRadians());
  }
}
