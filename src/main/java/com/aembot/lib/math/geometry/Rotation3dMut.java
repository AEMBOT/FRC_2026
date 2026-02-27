package com.aembot.lib.math.geometry;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

/**
 * A rotation in a 3D coordinate frame, represented by a quaternion. Note that unlike 2D rotations,
 * 3D rotations are not always commutative, so changing the order of rotations changes the result.
 *
 * <p>As an example of the order of rotations mattering, suppose we have a card that looks like
 * this:
 *
 * <pre>
 *          ┌───┐        ┌───┐
 *          │ X │        │ x │
 *   Front: │ | │  Back: │ · │
 *          │ | │        │ · │
 *          └───┘        └───┘
 * </pre>
 *
 * <p>If we rotate 90º clockwise around the axis into the page, then flip around the left/right
 * axis, we get one result:
 *
 * <pre>
 *   ┌───┐
 *   │ X │   ┌───────┐   ┌───────┐
 *   │ | │ → │------X│ → │······x│
 *   │ | │   └───────┘   └───────┘
 *   └───┘
 * </pre>
 *
 * <p>If we flip around the left/right axis, then rotate 90º clockwise around the axis into the
 * page, we get a different result:
 *
 * <pre>
 *   ┌───┐   ┌───┐
 *   │ X │   │ · │   ┌───────┐
 *   │ | │ → │ · │ → │x······│
 *   │ | │   │ x │   └───────┘
 *   └───┘   └───┘
 * </pre>
 *
 * <p>Because order matters for 3D rotations, we need to distinguish between <em>extrinsic</em>
 * rotations and <em>intrinsic</em> rotations. Rotating extrinsically means rotating around the
 * global axes, while rotating intrinsically means rotating from the perspective of the other
 * rotation. A neat property is that applying a series of rotations extrinsically is the same as
 * applying the same series in the opposite order intrinsically.
 */
public class Rotation3dMut implements StructSerializable {
  private final QuaternionMut quaternion;

  /**
   * Construct a Rotation3dMut wrapping the given mutable quaternion. Note that this constructor
   * does not copy the quaternion; it uses the reference directly. Modifications to this Rotation3d
   * will modify the given quaternion.
   */
  public Rotation3dMut(QuaternionMut quaternion) {
    this.quaternion = quaternion;
  }

  /** Construct a Rotation3dMut with the given initial value */
  public Rotation3dMut(Quaternion quaternion) {
    this(new QuaternionMut(quaternion));
  }

  /** Construct a new Rotation3dMut with a default angle of 0 degrees */
  public Rotation3dMut() {
    this(new QuaternionMut());
  }

  /**
   * Constructs a Rotation3dMut from extrinsic roll, pitch, and yaw.
   *
   * <p>Extrinsic rotations occur in that order around the axes in the fixed global frame rather
   * than the body frame.
   *
   * <p>Angles are measured counterclockwise with the rotation axis pointing "out of the page". If
   * you point your right thumb along the positive axis direction, your fingers curl in the
   * direction of positive rotation.
   *
   * @param roll The counterclockwise rotation angle around the X axis (roll) in radians.
   * @param pitch The counterclockwise rotation angle around the Y axis (pitch) in radians.
   * @param yaw The counterclockwise rotation angle around the Z axis (yaw) in radians.
   */
  public Rotation3dMut(double roll, double pitch, double yaw) {
    this.quaternion = new QuaternionMut();
    this.set(roll, pitch, yaw);
  }

  /**
   * Constructs a Rotation3dMut from extrinsic roll, pitch, and yaw.
   *
   * <p>Extrinsic rotations occur in that order around the axes in the fixed global frame rather
   * than the body frame.
   *
   * <p>Angles are measured counterclockwise with the rotation axis pointing "out of the page". If
   * you point your right thumb along the positive axis direction, your fingers curl in the
   * direction of positive rotation.
   *
   * @param roll The counterclockwise rotation angle around the X axis (roll).
   * @param pitch The counterclockwise rotation angle around the Y axis (pitch).
   * @param yaw The counterclockwise rotation angle around the Z axis (yaw).
   */
  public Rotation3dMut(Angle roll, Angle pitch, Angle yaw) {
    this(roll.in(Radians), pitch.in(Radians), yaw.in(Radians));
  }

  /**
   * Constructs a Rotation3dMut from extrinsic roll, pitch, and yaw.
   *
   * <p>Extrinsic rotations occur in that order around the axes in the fixed global frame rather
   * than the body frame.
   *
   * <p>Angles are measured counterclockwise with the rotation axis pointing "out of the page". If
   * you point your right thumb along the positive axis direction, your fingers curl in the
   * direction of positive rotation.
   *
   * @param roll The counterclockwise rotation angle around the X axis (roll). Note that just the
   *     current value will be used. If the MutAngle changes, this Rotation3dMut will NOT change.
   * @param pitch The counterclockwise rotation angle around the Y axis (pitch). Note that just the
   *     current value will be used. If the MutAngle changes, this Rotation3dMut will NOT change.
   * @param yaw The counterclockwise rotation angle around the Z axis (yaw). Note that just the
   *     current value will be used. If the MutAngle changes, this Rotation3dMut will NOT change.
   */
  public Rotation3dMut(MutAngle roll, MutAngle pitch, MutAngle yaw) {
    this(roll.in(Radians), pitch.in(Radians), yaw.in(Radians));
  }

  /**
   * Constructs a Rotation3d with the given axis-angle representation. The axis doesn't have to be
   * normalized.
   *
   * @param axis The rotation axis.
   * @param angleRadians The rotation around the axis in radians.
   */
  public Rotation3dMut(Vector<N3> axis, double angleRadians) {
    this.quaternion = new QuaternionMut();
    this.set(axis, angleRadians);
  }

  /**
   * Constructs a Rotation3d with the given axis-angle representation. The axis doesn't have to be
   * normalized.
   *
   * @param axis The rotation axis.
   * @param angle The rotation around the axis.
   */
  public Rotation3dMut(Vector<N3> axis, Angle angle) {
    this.quaternion = new QuaternionMut();
    this.set(axis, angle);
  }

  /**
   * Constructs a Rotation3d with the given axis-angle representation. The axis doesn't have to be
   * normalized.
   *
   * @param axis The rotation axis.
   * @param angle The rotation around the axis.
   */
  public Rotation3dMut(Vector<N3> axis, MutAngle angle) {
    this.quaternion = new QuaternionMut();
    this.set(axis, angle);
  }

  /**
   * Constructs a Rotation3d from a rotation matrix.
   *
   * @param rotationMatrix The rotation matrix.
   * @throws IllegalArgumentException if the rotation matrix isn't special orthogonal.
   */
  public Rotation3dMut(Matrix<N3, N3> rotationMatrix) {
    this.quaternion = new QuaternionMut();
    this.set(rotationMatrix);
  }

  /**
   * Import the value of a 2d rotation in the XY plane into a new Rotation3dMut, using its value for
   * the z axis (yaw) value
   */
  public Rotation3dMut(Rotation2dMut rotation) {
    this.quaternion = new QuaternionMut();
    this.importRotation2d(rotation);
  }

  /**
   * Import the value of a 2d rotation in the XY plane into a new Rotation3dMut, using its value for
   * the z axis (yaw) value
   */
  public Rotation3dMut(Rotation2d rotation) {
    this.quaternion = new QuaternionMut();
    this.importRotation2d(rotation);
  }

  /*
   * |---------------|
   * |--- SETTERS ---|
   * |---------------|
   */
  public Rotation3dMut set(double w, double x, double y, double z) {
    this.getQuaternion().set(w, x, y, z);
    return this;
  }

  /**
   * Set the Rotation3dMut's value with extrinsic roll, pitch, and yaw.
   *
   * <p>Extrinsic rotations occur in that order around the axes in the fixed global frame rather
   * than the body frame.
   *
   * <p>Angles are measured counterclockwise with the rotation axis pointing "out of the page". If
   * you point your right thumb along the positive axis direction, your fingers curl in the
   * direction of positive rotation.
   *
   * @param roll The counterclockwise rotation angle around the X axis (roll).
   * @param pitch The counterclockwise rotation angle around the Y axis (pitch).
   * @param yaw The counterclockwise rotation angle around the Z axis (yaw).
   */
  public Rotation3dMut set(double roll, double pitch, double yaw) {
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
    double cr = Math.cos(roll * 0.5);
    double sr = Math.sin(roll * 0.5);

    double cp = Math.cos(pitch * 0.5);
    double sp = Math.sin(pitch * 0.5);

    double cy = Math.cos(yaw * 0.5);
    double sy = Math.sin(yaw * 0.5);

    return this.set(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy);
  }

  /**
   * Set the Rotation3dMut's value with extrinsic roll, pitch, and yaw.
   *
   * <p>Extrinsic rotations occur in that order around the axes in the fixed global frame rather
   * than the body frame.
   *
   * <p>Angles are measured counterclockwise with the rotation axis pointing "out of the page". If
   * you point your right thumb along the positive axis direction, your fingers curl in the
   * direction of positive rotation.
   *
   * @param roll The counterclockwise rotation angle around the X axis (roll).
   * @param pitch The counterclockwise rotation angle around the Y axis (pitch).
   * @param yaw The counterclockwise rotation angle around the Z axis (yaw).
   */
  public Rotation3dMut set(Angle roll, Angle pitch, Angle yaw) {
    return set(roll.in(Radians), pitch.in(Radians), yaw.in(Radians));
  }

  /**
   * Set the Rotation3dMut's value with extrinsic roll, pitch, and yaw.
   *
   * <p>Extrinsic rotations occur in that order around the axes in the fixed global frame rather
   * than the body frame.
   *
   * <p>Angles are measured counterclockwise with the rotation axis pointing "out of the page". If
   * you point your right thumb along the positive axis direction, your fingers curl in the
   * direction of positive rotation.
   *
   * @param roll The counterclockwise rotation angle around the X axis (roll). Note that just the
   *     current value will be used. If the MutAngle changes, this Rotation3dMut will NOT change.
   * @param pitch The counterclockwise rotation angle around the Y axis (pitch). Note that just the
   *     current value will be used. If the MutAngle changes, this Rotation3dMut will NOT change.
   * @param yaw The counterclockwise rotation angle around the Z axis (yaw). Note that just the
   *     current value will be used. If the MutAngle changes, this Rotation3dMut will NOT change.
   */
  public Rotation3dMut set(MutAngle roll, MutAngle pitch, MutAngle yaw) {
    return set(roll.in(Radians), pitch.in(Radians), yaw.in(Radians));
  }

  /**
   * Constructs a Rotation3d with the given axis-angle representation. The axis doesn't have to be
   * normalized.
   *
   * @param axis The rotation axis.
   * @param angleRadians The rotation around the axis in radians.
   */
  public Rotation3dMut set(Vector<N3> axis, double angleRadians) {
    double norm = axis.norm();
    if (norm == 0.0) {
      return set(0, 0, 0);
    }

    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Definition
    Vector<N3> v = axis.times(1.0 / norm).times(Math.sin(angleRadians / 2.0));
    return set(Math.cos(angleRadians / 2.0), v.get(0, 0), v.get(1, 0), v.get(2, 0));
  }

  /**
   * Constructs a Rotation3d with the given axis-angle representation. The axis doesn't have to be
   * normalized.
   *
   * @param axis The rotation axis.
   * @param angleRadians The rotation around the axis in radians.
   */
  public Rotation3dMut set(Vector<N3> axis, Angle angle) {
    return set(axis, angle.in(Radians));
  }

  /**
   * Constructs a Rotation3d with the given axis-angle representation. The axis doesn't have to be
   * normalized.
   *
   * @param axis The rotation axis.
   * @param angleRadians The rotation around the axis in radians.
   */
  public Rotation3dMut set(Vector<N3> axis, MutAngle angle) {
    return set(axis, angle.in(Radians));
  }

  /**
   * Set the value of this Rotation3dMut from a rotation matrix.
   *
   * @param rotationMatrix The rotation matrix.
   * @throws IllegalArgumentException if the rotation matrix isn't special orthogonal.
   */
  public Rotation3dMut set(Matrix<N3, N3> rotationMatrix) {
    final var R = rotationMatrix;

    // Require that the rotation matrix is special orthogonal. This is true if
    // the matrix is orthogonal (RRᵀ = I) and normalized (determinant is 1).
    if (R.times(R.transpose()).minus(Matrix.eye(Nat.N3())).normF() > 1e-9) {
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

    // Turn rotation matrix into a quaternion
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    double trace = R.get(0, 0) + R.get(1, 1) + R.get(2, 2);
    double w;
    double x;
    double y;
    double z;

    if (trace > 0.0) {
      double s = 0.5 / Math.sqrt(trace + 1.0);
      w = 0.25 / s;
      x = (R.get(2, 1) - R.get(1, 2)) * s;
      y = (R.get(0, 2) - R.get(2, 0)) * s;
      z = (R.get(1, 0) - R.get(0, 1)) * s;
    } else {
      if (R.get(0, 0) > R.get(1, 1) && R.get(0, 0) > R.get(2, 2)) {
        double s = 2.0 * Math.sqrt(1.0 + R.get(0, 0) - R.get(1, 1) - R.get(2, 2));
        w = (R.get(2, 1) - R.get(1, 2)) / s;
        x = 0.25 * s;
        y = (R.get(0, 1) + R.get(1, 0)) / s;
        z = (R.get(0, 2) + R.get(2, 0)) / s;
      } else if (R.get(1, 1) > R.get(2, 2)) {
        double s = 2.0 * Math.sqrt(1.0 + R.get(1, 1) - R.get(0, 0) - R.get(2, 2));
        w = (R.get(0, 2) - R.get(2, 0)) / s;
        x = (R.get(0, 1) + R.get(1, 0)) / s;
        y = 0.25 * s;
        z = (R.get(1, 2) + R.get(2, 1)) / s;
      } else {
        double s = 2.0 * Math.sqrt(1.0 + R.get(2, 2) - R.get(0, 0) - R.get(1, 1));
        w = (R.get(1, 0) - R.get(0, 1)) / s;
        x = (R.get(0, 2) + R.get(2, 0)) / s;
        y = (R.get(1, 2) + R.get(2, 1)) / s;
        z = 0.25 * s;
      }
    }

    return this.set(w, x, y, z);
  }

  /**
   * Import the value of a 2d rotation in the XY plane into this Rotation3dMut, using its value for
   * the z axis (yaw) value
   */
  public Rotation3dMut importRotation2d(Rotation2dMut rotation) {
    return this.set(0, 0, rotation.getRadians());
  }

  /**
   * Import the value of a 2d rotation in the XY plane into this Rotation3dMut, using its value for
   * the z axis (yaw) value
   */
  public Rotation3dMut importRotation2d(Rotation2d rotation) {
    return this.set(0, 0, rotation.getRadians());
  }

  /*
   * |---------------|
   * |--- GETTERS ---|
   * |---------------|
   */
  public QuaternionMut getQuaternion() {
    return this.getQuaternion();
  }

  public Rotation3dMut makeCopy() {
    return new Rotation3dMut(getQuaternion().makeCopy());
  }

  /**
   * Returns the counterclockwise rotation angle around the X axis (roll) in radians.
   *
   * @return The counterclockwise rotation angle around the X axis (roll) in radians.
   */
  public double getX() {
    final var w = getQuaternion().getW();
    final var x = getQuaternion().getX();
    final var y = getQuaternion().getY();
    final var z = getQuaternion().getZ();

    // wpimath/algorithms.md
    final var cxcy = 1.0 - 2.0 * (x * x + y * y);
    final var sxcy = 2.0 * (w * x + y * z);
    final var cy_sq = cxcy * cxcy + sxcy * sxcy;
    if (cy_sq > 1e-20) {
      return Math.atan2(sxcy, cxcy);
    } else {
      return 0.0;
    }
  }

  /**
   * Returns the counterclockwise rotation angle around the Y axis (pitch) in radians.
   *
   * @return The counterclockwise rotation angle around the Y axis (pitch) in radians.
   */
  public double getY() {
    final var w = getQuaternion().getW();
    final var x = getQuaternion().getX();
    final var y = getQuaternion().getY();
    final var z = getQuaternion().getZ();

    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_(in_3-2-1_sequence)_conversion
    double ratio = 2.0 * (w * y - z * x);
    if (Math.abs(ratio) >= 1.0) {
      return Math.copySign(Math.PI / 2.0, ratio);
    } else {
      return Math.asin(ratio);
    }
  }

  /**
   * Returns the counterclockwise rotation angle around the Z axis (yaw) in radians.
   *
   * @return The counterclockwise rotation angle around the Z axis (yaw) in radians.
   */
  public double getZ() {
    final var w = getQuaternion().getW();
    final var x = getQuaternion().getX();
    final var y = getQuaternion().getY();
    final var z = getQuaternion().getZ();

    // wpimath/algorithms.md
    final var cycz = 1.0 - 2.0 * (y * y + z * z);
    final var cysz = 2.0 * (w * z + x * y);
    final var cy_sq = cycz * cycz + cysz * cysz;
    if (cy_sq > 1e-20) {
      return Math.atan2(cysz, cycz);
    } else {
      return Math.atan2(2.0 * w * z, w * w - z * z);
    }
  }

  /**
   * Returns the axis in the axis-angle representation of this rotation.
   *
   * @return The axis in the axis-angle representation.
   */
  public Vector<N3> getAxis() {
    double norm =
        Math.sqrt(
            getQuaternion().getX() * getQuaternion().getX()
                + getQuaternion().getY() * getQuaternion().getY()
                + getQuaternion().getZ() * getQuaternion().getZ());
    if (norm == 0.0) {
      return VecBuilder.fill(0.0, 0.0, 0.0);
    } else {
      return VecBuilder.fill(
          getQuaternion().getX() / norm,
          getQuaternion().getY() / norm,
          getQuaternion().getZ() / norm);
    }
  }

  /**
   * Returns the angle in radians in the axis-angle representation of this rotation.
   *
   * @return The angle in radians in the axis-angle representation of this rotation.
   */
  public double getAngle() {
    double norm =
        Math.sqrt(
            getQuaternion().getX() * getQuaternion().getX()
                + getQuaternion().getY() * getQuaternion().getY()
                + getQuaternion().getZ() * getQuaternion().getZ());
    return 2.0 * Math.atan2(norm, getQuaternion().getW());
  }

  /**
   * Returns rotation matrix representation of this rotation.
   *
   * @return Rotation matrix representation of this rotation.
   */
  public Matrix<N3, N3> toMatrix() {
    double w = getQuaternion().getW();
    double x = getQuaternion().getX();
    double y = getQuaternion().getY();
    double z = getQuaternion().getZ();

    // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
    return MatBuilder.fill(
        Nat.N3(),
        Nat.N3(),
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - w * z),
        2.0 * (x * z + w * y),
        2.0 * (x * y + w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - w * x),
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        1.0 - 2.0 * (x * x + y * y));
  }

  /**
   * Returns rotation vector representation of this rotation.
   *
   * @return Rotation vector representation of this rotation.
   */
  public Vector<N3> toVector() {
    return getQuaternion().toRotationVector();
  }

  /**
   * @return A Rotation2d representing this Rotation3d projected into the X-Y plane.
   * @see #toRotation2dMut()
   * @see Rotation2dMut#importMutable(Rotation3dMut)
   */
  public Rotation2d toRotation2d() {
    return new Rotation2d(getZ());
  }

  /**
   * @return A Rotation2dMut representing this Rotation3d projected into the X-Y plane. This only
   *     represents the current angle of the Rotation3dMut, and will <strong>not</strong> be updated
   *     automatically. If you want an updated Rotation2dMut, it's better to create one in an outer
   *     scope and then use {@link Rotation2dMut#importMutable(Rotation3dMut)} to update it.
   * @see {@link Rotation2dMut#importMutable(Rotation3dMut)}
   */
  public Rotation2dMut toRotation2dMut() {
    return new Rotation2dMut(getZ());
  }

  /*
   * |------------------|
   * |--- OPERATIONS ---|
   * |------------------|
   */

  /**
   * Invert this Rotation3dMut
   *
   * @return This Rotation3dMut for chaining
   */
  public Rotation3dMut unarySubtract() {
    getQuaternion().invert();
    return this;
  }

  /**
   * Adds the new rotation to the current rotation. The other rotation is applied extrinsically,
   * which means that it rotates around the global axes. For example, {@code new
   * Rotation3dMut(Units.degreesToRadians(90), 0, 0).add(new Rotation3dMut(0,
   * Units.degreesToRadians(45), 0))} rotates by 90 degrees around the +X axis and then by 45
   * degrees around the global +Y axis. (This is equivalent to {@code new
   * Rotation3dMut(Units.degreesToRadians(90), Units.degreesToRadians(45), 0)})
   *
   * @param other The extrinsic rotation to rotate by.
   * @return The new rotated Rotation3d.
   */
  public Rotation3dMut add(Rotation3d other) {
    getQuaternion().premultiply(other.getQuaternion());
    return this;
    // return new Rotation3d(other.quaternion.times(quaternion));
  }

  /**
   * Adds the new rotation to the current rotation. The other rotation is applied extrinsically,
   * which means that it rotates around the global axes. For example, {@code new
   * Rotation3dMut(Units.degreesToRadians(90), 0, 0).add(new Rotation3dMut(0,
   * Units.degreesToRadians(45), 0))} rotates by 90 degrees around the +X axis and then by 45
   * degrees around the global +Y axis. (This is equivalent to {@code new
   * Rotation3dMut(Units.degreesToRadians(90), Units.degreesToRadians(45), 0)})
   *
   * @param other The extrinsic rotation to rotate by.
   * @return The new rotated Rotation3d.
   */
  public Rotation3dMut add(Rotation3dMut other) {
    getQuaternion().premultiply(other.getQuaternion());
    return this;
    // return new Rotation3d(other.quaternion.times(quaternion));
  }

  /**
   * Subtracts the other rotation from the current rotation and returns the new rotation. The new
   * rotation is from the perspective of the other rotation (like {@link Pose3d#minus}), so it needs
   * to be applied intrinsically. See the class comment for definitions of extrinsic and intrinsic
   * rotations.
   *
   * <p>Note that {@code a.minus(b).plus(b)} always equals {@code a}, but {@code b.plus(a.minus(b))}
   * sometimes doesn't. To apply a rotation offset, use either {@code offset =
   * measurement.unaryMinus().plus(actual); newAngle = angle.plus(offset);} or {@code offset =
   * actual.minus(measurement); newAngle = offset.plus(angle);}, depending on how the corrected
   * angle needs to change as the input angle changes.
   *
   * @param other The rotation to subtract.
   * @return The difference between the two rotations, from the perspective of the other rotation.
   */
  public Rotation3dMut minus(Rotation3dMut other) {
    // return add(other.makeCopy().unarySubtract());
    getQuaternion()
        .premultiply(
            other.getQuaternion().getW(),
            -other.getQuaternion().getX(),
            -other.getQuaternion().getY(),
            -other.getQuaternion().getZ());
    return this;
  }

  /**
   * Subtracts the other rotation from the current rotation and returns the new rotation. The new
   * rotation is from the perspective of the other rotation (like {@link Pose3d#minus}), so it needs
   * to be applied intrinsically. See the class comment for definitions of extrinsic and intrinsic
   * rotations.
   *
   * <p>Note that {@code a.minus(b).plus(b)} always equals {@code a}, but {@code b.plus(a.minus(b))}
   * sometimes doesn't. To apply a rotation offset, use either {@code offset =
   * measurement.unaryMinus().plus(actual); newAngle = angle.plus(offset);} or {@code offset =
   * actual.minus(measurement); newAngle = offset.plus(angle);}, depending on how the corrected
   * angle needs to change as the input angle changes.
   *
   * @param other The rotation to subtract.
   * @return The difference between the two rotations, from the perspective of the other rotation.
   */
  public Rotation3dMut minus(Rotation3d other) {
    // return add(other.makeCopy().unarySubtract());
    getQuaternion()
        .premultiply(
            other.getQuaternion().getW(),
            -other.getQuaternion().getX(),
            -other.getQuaternion().getY(),
            -other.getQuaternion().getZ());
    return this;
  }

  /**
   * Multiplies the current rotation by a scalar.
   *
   * @param scalar The scalar.
   * @return This Rotation3dMut for chaining
   */
  public Rotation3dMut times(double scalar) {
    // https://en.wikipedia.org/wiki/Slerp#Quaternion_Slerp
    if (getQuaternion().getW() >= 0.0) {
      return this.set(
          VecBuilder.fill(getQuaternion().getX(), getQuaternion().getY(), getQuaternion().getZ()),
          2.0 * scalar * Math.acos(getQuaternion().getW()));
    } else {
      return this.set(
          VecBuilder.fill(
              -getQuaternion().getX(), -getQuaternion().getY(), -getQuaternion().getZ()),
          2.0 * scalar * Math.acos(-getQuaternion().getW()));
    }
  }

  /**
   * Divides the current rotation by a scalar.
   *
   * @param scalar The scalar.
   * @return This Rotation3dMut for chaining
   */
  public Rotation3dMut div(double scalar) {
    return times(1.0 / scalar);
  }

  /**
   * Returns the current rotation relative to the given rotation. Because the result is in the
   * perspective of the given rotation, it must be applied intrinsically. See the class comment for
   * definitions of extrinsic and intrinsic rotations.
   *
   * @param other The rotation describing the orientation of the new coordinate frame that the
   *     current rotation will be converted into.
   * @return The current rotation relative to the new orientation of the coordinate frame.
   */
  public Rotation3dMut relativeTo(Rotation3d other) {
    // To apply a quaternion intrinsically, we must right-multiply by that quaternion.
    // Therefore, "this_q relative to other_q" is the q such that other_q q = this_q:
    //
    //   other_q q = this_q
    //   q = other_q⁻¹ this_q
    // return new Rotation3d(other.getQuaternion().inverse().times(getQuaternion()));

    getQuaternion()
        .premultiply(
            other.getQuaternion().getW(),
            -other.getQuaternion().getX(),
            -other.getQuaternion().getY(),
            -other.getQuaternion().getZ());
    return this;
  }

  /**
   * Returns the current rotation relative to the given rotation. Because the result is in the
   * perspective of the given rotation, it must be applied intrinsically. See the class comment for
   * definitions of extrinsic and intrinsic rotations.
   *
   * @param other The rotation describing the orientation of the new coordinate frame that the
   *     current rotation will be converted into.
   * @return The current rotation relative to the new orientation of the coordinate frame.
   */
  public Rotation3dMut relativeTo(Rotation3dMut other) {
    // To apply a quaternion intrinsically, we must right-multiply by that quaternion.
    // Therefore, "this_q relative to other_q" is the q such that other_q q = this_q:
    //
    //   other_q q = this_q
    //   q = other_q⁻¹ this_q
    // return new Rotation3d(other.getQuaternion().inverse().times(getQuaternion()));

    getQuaternion()
        .premultiply(
            other.getQuaternion().getW(),
            -other.getQuaternion().getX(),
            -other.getQuaternion().getY(),
            -other.getQuaternion().getZ());
    return this;
  }

  /*
   * |------------------------|
   * |--- OBJECT OVERRIDES ---|
   * |------------------------|
   */

  @Override
  public String toString() {
    return String.format("Rotation3d(%s)", getQuaternion());
  }

  /**
   * Checks equality between this Rotation3d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    return obj instanceof Rotation3dMut other
        && Math.abs(
                Math.abs(getQuaternion().dot(other.getQuaternion()))
                    - getQuaternion().norm() * other.getQuaternion().norm())
            < 1e-9;
  }

  @Override
  public int hashCode() {
    return Objects.hash(getQuaternion());
  }
}
