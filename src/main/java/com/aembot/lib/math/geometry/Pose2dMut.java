package com.aembot.lib.math.geometry;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Objects;

public class Pose2dMut implements StructSerializable {
  private final Translation2dMut translation;
  private final Rotation2dMut rotation;

  /*
   * |--------------------|
   * |--- CONSTRUCTORS ---|
   * |--------------------|
   */

  /**
   * Construct a new Pose2dMut with given references to translation and rotation objects. Note that
   * this constructor does not clone the given objects on its own.
   */
  public Pose2dMut(Translation2dMut translation, Rotation2dMut rotation) {
    this.translation = translation;
    this.rotation = rotation;
  }

  /**
   * Construct a new Pose2dMut with given reference to a translation object and a rotation value.
   * Note that this constructor does not clone the given translation on its own. The rotation of
   * this Pose2dMut will be a mutable Rotation2d with the initial value of the given Rotation2d.
   */
  public Pose2dMut(Translation2dMut translation, Rotation2d rotation) {
    this(translation, new Rotation2dMut(rotation));
  }

  /**
   * Construct a new Pose2dMut with given references to a rotation object and a translation value.
   * Note that this constructor does not clone the given rotation on its own. The translation of
   * this Pose2dMut will be a mutable Translation2d with the initial value of the given
   * Translation2d.
   */
  public Pose2dMut(Translation2d translation, Rotation2dMut rotation) {
    this(new Translation2dMut(translation), rotation);
  }

  /**
   * Construct a new Pose2dMut with the given translation and rotation values. These values will be
   * copied and made mutable.
   */
  public Pose2dMut(Translation2d translation, Rotation2d rotation) {
    this(new Translation2dMut(translation), new Rotation2dMut(rotation));
  }

  /**
   * Construct a new Pose2dMut with given references to a rotation object and a translation value.
   * Note that this constructor does not clone the given rotation on its own. The translation of
   * this Pose2dMut will be a mutable Translation2d with the initial value of the given coordinates.
   */
  public Pose2dMut(double x, double y, Rotation2dMut rotation) {
    this(new Translation2dMut(x, y), rotation);
  }

  /** Construct a new Pose2dMut at 0,0 with a rotation of 0 */
  public Pose2dMut() {
    this(new Translation2dMut(), new Rotation2dMut());
  }

  /** Create a deep copy of the given {@link Pose2dMut} */
  public Pose2dMut(Pose2dMut from) {
    this(new Translation2dMut(from.getTranslation()), new Rotation2dMut(from.getRotation()));
  }

  public Pose2dMut(Pose2d from) {
    this(new Translation2dMut(from.getTranslation()), new Rotation2dMut(from.getRotation()));
  }

  /**
   * Constructs a pose with the specified affine transformation matrix.
   *
   * @param matrix The affine transformation matrix.
   * @throws IllegalArgumentException if the affine transformation matrix is invalid.
   */
  public Pose2dMut(Matrix<N3, N3> matrix) {
    this();
    set(matrix);
  }

  /*
   * |---------------|
   * |--- SETTERS ---|
   * |---------------|
   */

  public Pose2dMut setTranslation(double x, double y) {
    getTranslation().set(x, y);
    return this;
  }

  public Pose2dMut importTranslation(Translation2d translation) {
    getTranslation().importImmutable(translation);
    return this;
  }

  public Pose2dMut importTranslation(Translation2dMut translation) {
    getTranslation().importMutable(translation);
    return this;
  }

  public Pose2dMut setRotationRadians(double valueRadians) {
    getRotation().setRadians(valueRadians);
    return this;
  }

  public Pose2dMut setRotationDegrees(double valueDegrees) {
    getRotation().setDegrees(valueDegrees);
    return this;
  }

  public Pose2dMut setRotation(Angle value) {
    getRotation().setAngle(value);
    return this;
  }

  public Pose2dMut importRotation(Rotation2d rotation) {
    getRotation().importImmutable(rotation);
    return this;
  }

  public Pose2dMut importRotation(Rotation2dMut rotation) {
    getRotation().importMutable(rotation);
    return this;
  }

  public Pose2dMut importMutable(Pose2dMut other) {
    return this.importTranslation(other.getTranslation()).importRotation(other.getRotation());
  }

  public Pose2dMut importImmutable(Pose2d other) {
    return this.importTranslation(other.getTranslation()).importRotation(other.getRotation());
  }

  public Pose2dMut set(double x, double y, double rotationRadians) {
    return this.setTranslation(x, y).setRotationRadians(rotationRadians);
  }

  public Pose2dMut set(Distance x, Distance y, double rotationRadians) {
    return this.set(x.in(Meters), y.in(Meters), rotationRadians);
  }

  public Pose2dMut set(double x, double y, Angle rotation) {
    return this.set(x, y, rotation.in(Radians));
  }

  public Pose2dMut set(Distance x, Distance y, Angle rotation) {
    return this.set(x.in(Meters), y.in(Meters), rotation.in(Radians));
  }

  /**
   * Constructs a pose with the specified affine transformation matrix.
   *
   * @param matrix The affine transformation matrix.
   * @return This Pose2dMut for chaining.
   * @throws IllegalArgumentException if the affine transformation matrix is invalid.
   */
  public Pose2dMut set(Matrix<N3, N3> matrix) {
    this.translation.set(matrix.get(0, 2), matrix.get(1, 2));
    this.rotation.setFromRotationMatrix(matrix.block(2, 2, 0, 0));
    if (matrix.get(2, 0) != 0.0 || matrix.get(2, 1) != 0.0 || matrix.get(2, 2) != 1.0) {
      throw new IllegalArgumentException("Affine transformation matrix is invalid");
    }
    return this;
  }

  /*
   * |---------------|
   * |--- GETTERS ---|
   * |---------------|
   */

  public Translation2dMut getTranslation() {
    return translation;
  }

  /** Get the x component of this Pose2dMut in meters */
  public double getX() {
    return getTranslation().getX();
  }

  /** Get the y component of this Pose2dMut in meters */
  public double getY() {
    return getTranslation().getY();
  }

  public Rotation2dMut getRotation() {
    return rotation;
  }

  /** Get the θ component of this Pose2dMut in radians */
  public double getRotationRadians() {
    return getRotation().getRadians();
  }

  /** Get the θ component of this Pose2dMut in degrees */
  public double getRotationDegrees() {
    return getRotation().getDegrees();
  }

  public Pose2d toImmutable() {
    return new Pose2d(getTranslation().exportImmutable(), getRotation().exportImmutable());
  }

  /**
   * Make a deep copy of this Rotation2dMut. Should be avoided if possible, as it creates a new
   * object.
   *
   * @see #importMutable(Pose2dMut)
   */
  public Pose2dMut makeCopy() {
    return new Pose2dMut(this);
  }

  /*
   * |------------------|
   * |--- OPERATIONS ---|
  .* |------------------|
   */

  /**
   * Transforms the pose by the given transformation and returns the new transformed pose.
   *
   * <pre>
   * [x_new]    [cos, -sin, 0][transform.x]
   * [y_new] += [sin,  cos, 0][transform.y]
   * [t_new]    [  0,    0, 1][transform.t]
   * </pre>
   *
   * @param other The transform to transform the pose by.
   * @return The transformed pose.
   */
  public Pose2dMut add(Pose2d other) {
    this.getTranslation().add(other.getTranslation());
    this.getRotation().add(other.getRotation());
    return this;
  }

  /**
   * Transforms the pose by the given transformation and returns the new transformed pose.
   *
   * <pre>
   * [x_new]    [cos, -sin, 0][transform.x]
   * [y_new] += [sin,  cos, 0][transform.y]
   * [t_new]    [  0,    0, 1][transform.t]
   * </pre>
   *
   * @param other The transform to transform the pose by.
   * @return This Pose2dMut for chaining.
   */
  public Pose2dMut add(Pose2dMut other) {
    this.getTranslation().add(other.getTranslation());
    this.getRotation().add(other.getRotation());
    return this;
  }

  /**
   * Transforms the pose by the given transformation and returns the new transformed pose.
   *
   * <pre>
   * [x_new]    [cos, -sin, 0][transform.x]
   * [y_new] += [sin,  cos, 0][transform.y]
   * [t_new]    [  0,    0, 1][transform.t]
   * </pre>
   *
   * @param other The transform to transform the pose by.
   * @return This Pose2dMut for chaining.
   */
  public Pose2dMut add(Transform2d other) {
    this.getTranslation().add(other.getTranslation());
    this.getRotation().add(other.getRotation());
    return this;
  }

  /**
   * Subtract the given pose from this one.
   *
   * @param other The initial pose of the transformation.
   * @return This Pose2dMut for chaining.
   */
  public Pose2dMut subtract(Pose2d other) {
    this.getTranslation().subtract(other.getTranslation());
    this.getRotation().subtract(other.getRotation());
    return this;
  }

  /**
   * Subtract the given pose from this one.
   *
   * @param other The initial pose of the transformation.
   * @return This Pose2dMut for chaining.
   */
  public Pose2dMut subtract(Pose2dMut other) {
    this.getTranslation().subtract(other.getTranslation());
    this.getRotation().subtract(other.getRotation());
    return this;
  }

  /**
   * Subtract the given pose from this one.
   *
   * @param other The initial pose of the transformation.
   * @return This Pose2dMut for chaining.
   */
  public Pose2dMut subtract(Transform2d other) {
    this.getTranslation().subtract(other.getTranslation());
    this.getRotation().subtract(other.getRotation());
    return this;
  }

  /**
   * Multiplies the Pose2dMut by the given scalar
   *
   * @return This Pose2dMut for chaining.
   */
  public Pose2dMut multiply(double scalar) {
    this.getTranslation().multiply(scalar);
    this.getRotation().multiply(scalar);
    return this;
  }

  /**
   * Divides the Pose2dMut by the given scalar
   *
   * @return This Pose2dMut for chaining.
   */
  public Pose2dMut divide(double scalar) {
    return this.multiply(1.0 / scalar);
  }

  /**
   * Rotates the pose around the origin
   *
   * @param other The rotation to transform the pose by.
   * @return This Pose2dMut for chaining.
   */
  public Pose2dMut rotateBy(Rotation2d other) {
    getTranslation().rotateBy(other);
    getRotation().add(other);
    return this;
  }

  /**
   * Rotates the pose around the origin
   *
   * @param other The rotation to transform the pose by.
   * @return This Pose2dMut for chaining.
   */
  public Pose2dMut rotateBy(Rotation2dMut other) {
    getTranslation().rotateBy(other);
    getRotation().add(other);
    return this;
  }

  /**
   * Rotates the current pose around a point in 2D space.
   *
   * @param point The point in 2D space to rotate around.
   * @param rot The rotation to rotate the pose by.
   * @return The new rotated pose.
   */
  public Pose2dMut rotateAround(Translation2d point, Rotation2d rot) {
    getTranslation().rotateAround(point, rotation);
    getRotation().add(rot);
    return this;
  }

  /**
   * Rotates the current pose around a point in 2D space.
   *
   * @param point The point in 2D space to rotate around.
   * @param rot The rotation to rotate the pose by.
   * @return The new rotated pose.
   */
  public Pose2dMut rotateAround(Translation2dMut point, Rotation2d rot) {
    getTranslation().rotateAround(point, rotation);
    getRotation().add(rot);
    return this;
  }

  /**
   * Rotates the current pose around a point in 2D space.
   *
   * @param point The point in 2D space to rotate around.
   * @param rot The rotation to rotate the pose by.
   * @return The new rotated pose.
   */
  public Pose2dMut rotateAround(Translation2d point, Rotation2dMut rot) {
    getTranslation().rotateAround(point, rotation);
    getRotation().add(rot);
    return this;
  }

  /**
   * Rotates the current pose around a point in 2D space.
   *
   * @param point The point in 2D space to rotate around.
   * @param rot The rotation to rotate the pose by.
   * @return The new rotated pose.
   */
  public Pose2dMut rotateAround(Translation2dMut point, Rotation2dMut rot) {
    getTranslation().rotateAround(point, rotation);
    getRotation().add(rot);
    return this;
  }

  /**
   * Update this Pose2dMut with a (constant curvature) velocity.
   *
   * <p>See <a href="https://file.tavsys.net/control/controls-engineering-in-frc.pdf">Controls
   * Engineering in the FIRST Robotics Competition</a> section 10.2 "Pose exponential" for a
   * derivation.
   *
   * <p>The twist is a change in pose in the robot's coordinate frame since the previous pose
   * update. When the user runs exp() on the previous known field-relative pose with the argument
   * being the twist, the user will receive the new field-relative pose.
   *
   * <p>"Exp" represents the pose exponential, which is solving a differential equation moving the
   * pose forward in time.
   *
   * @param twist The change in pose in the robot's coordinate frame since the previous pose update.
   *     For example, if a non-holonomic robot moves forward 0.01 meters and changes angle by 0.5
   *     degrees since the previous pose update, the twist would be Twist2d(0.01, 0.0,
   *     Units.degreesToRadians(0.5)).
   * @return This Pose2dMut for chaining
   */
  public Pose2dMut exp(Twist2d twist) {
    double dx = twist.dx;
    double dy = twist.dy;
    double dtheta = twist.dtheta;

    double sinTheta = Math.sin(dtheta);
    double cosTheta = Math.cos(dtheta);

    double s;
    double c;
    if (Math.abs(dtheta) < 1E-9) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
      c = 0.5 * dtheta;
    } else {
      s = sinTheta / dtheta;
      c = (1 - cosTheta) / dtheta;
    }

    double transformX = dx * s - dy * c;
    double transformY = dx * c + dy * s;

    // WPILib's implementation used atan2 of cos and sin for transform theta, which seems very
    // unnecesary given those are just derived from dtheta.

    this.getTranslation()
        .set(getTranslation().getX() + transformX, getTranslation().getY() + transformY);
    this.getRotation().add(dtheta);

    return this;
  }

  /**
   * Returns an affine transformation matrix representation of this pose.
   *
   * @return An affine transformation matrix representation of this pose.
   */
  public Matrix<N3, N3> toMatrix() {
    var vec = getTranslation().toVector();
    var mat = getRotation().toMatrix();
    return MatBuilder.fill(
        Nat.N3(),
        Nat.N3(),
        mat.get(0, 0),
        mat.get(0, 1),
        vec.get(0),
        mat.get(1, 0),
        mat.get(1, 1),
        vec.get(1),
        0.0,
        0.0,
        1.0);
  }

  /**
   * Returns the nearest Pose2d from a collection of poses. If two or more poses in the collection
   * have the same distance from this pose, return the one with the closest rotation component.
   *
   * @param poses The collection of poses to find the nearest.
   * @return The nearest Pose2d from the collection.
   */
  public Pose2dMut nearestMutable(Collection<Pose2dMut> poses) {
    return Collections.min(
        poses,
        Comparator.comparing(
                (Pose2dMut other) -> this.getTranslation().getDistance(other.getTranslation()))
            .thenComparing(
                (Pose2dMut other) ->
                    Math.abs(this.getRotationRadians() - other.getRotationRadians())));
  }

  /**
   * Returns the nearest Pose2d from a collection of poses. If two or more poses in the collection
   * have the same distance from this pose, return the one with the closest rotation component.
   *
   * @param poses The collection of poses to find the nearest.
   * @return The nearest Pose2d from the collection.
   */
  public Pose2d nearestImmutable(Collection<Pose2d> poses) {
    return Collections.min(
        poses,
        Comparator.comparing(
                (Pose2d other) -> this.getTranslation().getDistance(other.getTranslation()))
            .thenComparing(
                (Pose2d other) ->
                    Math.abs(this.getRotationRadians() - other.getRotation().getRadians())));
  }

  /*
   * |------------------------|
   * |--- OBJECT OVERRIDES ---|
   * |------------------------|
   */
  @Override
  public String toString() {
    return String.format("Pose2dMut(%s, %s)", getTranslation(), getRotation());
  }

  /**
   * Checks equality between this Pose2d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    return obj instanceof Pose2dMut pose
        && getTranslation().equals(pose.getTranslation())
        && getRotation().equals(pose.getRotation());
  }

  @Override
  public int hashCode() {
    return Objects.hash(getTranslation(), getRotation());
  }
}
