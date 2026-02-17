package com.aembot.lib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;

public class MatrixBuilder<R extends Num, C extends Num> {
  public final Matrix<R, C> matrix;

  /**
   * The index of the row or column we should fill with the next call of {@link #addRow(double[])}
   * or {@link #addColumn(double[])}. If it's -1, we've finished filling and neither of the
   * aforementioned methods should be called.
   */
  private int fillingIndex = 0;

  /**
   * If false, we're filling rows in sequential order. Otherwise we're filling columns in sequential
   * order.
   */
  private Boolean fillingColumn = null;

  /**
   * Construct a MatrixBuilder with a numRows x numColumns matrix. Equivalent to
   *
   * <pre>{@code new Matrix<>(numRows, numColumns)}</pre>
   */
  public MatrixBuilder(Nat<R> numRows, Nat<C> numColumns) {
    this.matrix = new Matrix<R, C>(numRows, numColumns);
  }

  /**
   * Construct a numRows x numColumns MatrixBuilder populated with the given values. For example, in
   * a 3x3 matrix the values will be inserted in the order:
   *
   * <pre>
   * .
   * |0|1|2|
   * |-|-|-|
   * |3|4|5|
   * |-|-|-|
   * |6|7|8|
   * </pre>
   */
  public MatrixBuilder(Nat<R> numRows, Nat<C> numColumns, double... values) {
    if (values.length > numRows.getNum() * numColumns.getNum()) {
      throw new IllegalArgumentException("Length of values exceeds capacity of matrix.");
    }

    this.matrix = new Matrix<>(numRows, numColumns);

    for (int i = 0; i < numRows.getNum() * numColumns.getNum(); i++) {
      int row = i % numRows.getNum();
      int column = Math.floorDiv(i, numRows.getNum());

      if (values.length < i) {
        matrix.set(row, column, values[i]);
      } else {
        matrix.set(row, column, 0);
      }
    }
  }

  public MatrixBuilder<R, C> withRow(int row, double... values) {
    if (row + 1 > matrix.getNumRows() || row < 0) throw new IndexOutOfBoundsException(row);
    if (values.length > matrix.getNumCols())
      throw new IllegalArgumentException("values array length exeeds number of columns");

    for (int i = 0; i < matrix.getNumCols(); i++) {
      if (i < values.length) {
        matrix.set(row, i, values[i]);
      } else {
        matrix.set(row, i, 0);
      }
    }

    return this;
  }

  public MatrixBuilder<R, C> withColumn(int column, double... values) {
    if (column + 1 > matrix.getNumCols() || column < 0) throw new IndexOutOfBoundsException(column);
    if (values.length > matrix.getNumRows())
      throw new IllegalArgumentException("values array length exeeds number of rows");

    for (int i = 0; i < matrix.getNumRows(); i++) {
      if (i < values.length) {
        System.out.println(matrix);
        matrix.set(i, column, values[i]);
      } else {
        matrix.set(i, column, 0);
      }
    }

    return this;
  }

  /**
   * Method to fill rows in sequential order. The first call of this method will set the mode of
   * this matrix builder to row fill mode, meaning you can't call {@link #addColumn(double...)}.
   * Each call of this method will increase an internal index until the matrix is full, at which
   * point calling this method will throw an {@link IllegalStateException}.
   *
   * @param values The values to fill this row with
   * @return This {@link MatrixBuilder} for chaining
   */
  public MatrixBuilder<R, C> addRow(double... values) {
    if (fillingColumn != null && fillingColumn == true)
      throw new IllegalStateException("This builder is in column filling mode");
    if (fillingIndex == -1) throw new IllegalStateException("The matrix has already been filled");

    fillingColumn = false;
    this.withRow(fillingIndex, values);

    fillingIndex++;
    if (fillingIndex >= matrix.getNumRows()) fillingIndex = -1;
    return this;
  }

  /**
   * Method to fill columns in sequential order. The first call of this method will set the mode of
   * this matrix builder to column fill mode, meaning you can't call {@link #addRow(double...)}.
   * Each call of this method will increase an internal index until the matrix is full, at which
   * point calling this method will throw an {@link IllegalStateException}.
   *
   * @param values The values to fill this column with
   * @return This {@link MatrixBuilder} for chaining
   */
  public MatrixBuilder<R, C> addColumn(double... values) {
    if (fillingColumn != null && fillingColumn == false)
      throw new IllegalStateException("This builder is in row filling mode");
    if (fillingIndex == -1) throw new IllegalStateException("The matrix has already been filled");

    fillingColumn = true;
    this.withColumn(fillingIndex, values);

    fillingIndex++;
    if (fillingIndex >= matrix.getNumCols()) fillingIndex = -1;
    return this;
  }

  /** Get the matrix this builder is building */
  public Matrix<R, C> get() {
    return matrix;
  }
}
