package com.aembot.lib.core.tracing;

/**
 * Represents a single traced span (function call) with timing information. Pre-allocated to avoid
 * GC during robot loop.
 */
public final class TraceSpan {
  /** Start time in nanoseconds (System.nanoTime) for high precision duration */
  public long startNanos;

  /** End time in nanoseconds */
  public long endNanos;

  /** Start time as FPGA timestamp for correlation with other logs */
  public double startFPGA;

  /** Name of the traced function (ClassName.methodName or custom) */
  public String name;

  /** Nesting depth (0 = top level, increases with nested calls) */
  public byte depth;

  /** Whether this span has completed (endSpan was called) */
  public boolean complete;

  /** Reset this span for reuse */
  public void reset() {
    startNanos = 0;
    endNanos = 0;
    startFPGA = 0;
    name = null;
    depth = 0;
    complete = false;
  }

  /** Get duration in microseconds */
  public double getDurationMicros() {
    return (endNanos - startNanos) / 1000.0;
  }

  /** Get duration in milliseconds */
  public double getDurationMillis() {
    return (endNanos - startNanos) / 1_000_000.0;
  }
}
