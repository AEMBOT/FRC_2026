package com.aembot.lib.core.tracing;

import edu.wpi.first.wpilibj.Timer;

/**
 * Container for all trace spans within a single robot loop iteration. Pre-allocated to avoid GC
 * during robot loop.
 */
public final class TraceFrame {
  /** Maximum number of spans per frame (128 should be plenty for one loop) */
  public static final int MAX_SPANS = 128;

  /** Pre-allocated array of spans */
  public final TraceSpan[] spans;

  /** Current number of active spans in this frame */
  public int spanCount;

  /** Frame number (incrementing counter) */
  public int frameNumber;

  /** FPGA timestamp when this frame started */
  public double startTime;

  /** FPGA timestamp when this frame ended */
  public double endTime;

  /** Create a new frame with pre-allocated spans */
  public TraceFrame() {
    spans = new TraceSpan[MAX_SPANS];
    for (int i = 0; i < MAX_SPANS; i++) {
      spans[i] = new TraceSpan();
    }
    spanCount = 0;
    frameNumber = 0;
    startTime = 0;
    endTime = 0;
  }

  /** Reset this frame for reuse */
  public void reset() {
    for (int i = 0; i < spanCount; i++) {
      spans[i].reset();
    }
    spanCount = 0;
    startTime = Timer.getFPGATimestamp();
    endTime = 0;
  }

  /** Get total frame duration in milliseconds */
  public double getDurationMillis() {
    return (endTime - startTime) * 1000.0;
  }
}
