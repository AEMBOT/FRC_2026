package com.aembot.lib.core.tracing;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Exports trace data to Chrome Tracing JSON format. This format is compatible with:
 *
 * <ul>
 *   <li>Chrome's built-in tracing viewer (chrome://tracing)
 *   <li>Perfetto UI (https://ui.perfetto.dev)
 *   <li>Custom trace viewers
 * </ul>
 */
public final class TraceExporter {

  private TraceExporter() {} // Static only

  /**
   * Export all frames in the buffer to Chrome Tracing JSON format.
   *
   * @param frames The frame buffer
   * @param currentIndex Current write index in circular buffer
   * @param totalFrameCount Total frames recorded
   * @param path Output file path
   */
  public static void exportToJson(
      TraceFrame[] frames, int currentIndex, int totalFrameCount, String path) {
    int numFrames = Math.min(totalFrameCount, frames.length);
    exportRecentToJson(frames, currentIndex, numFrames, path);
  }

  /**
   * Export the most recent N frames to Chrome Tracing JSON format.
   *
   * @param frames The frame buffer
   * @param currentIndex Current write index in circular buffer
   * @param numFrames Number of frames to export
   * @param path Output file path
   */
  public static void exportRecentToJson(
      TraceFrame[] frames, int currentIndex, int numFrames, String path) {
    StringBuilder json = new StringBuilder(1024 * 1024); // 1MB initial capacity
    json.append("{\"traceEvents\":[\n");

    boolean firstEvent = true;
    int bufferSize = frames.length;

    // Iterate through the requested frames (oldest to newest)
    int startIndex = (currentIndex - numFrames + 1 + bufferSize) % bufferSize;

    for (int i = 0; i < numFrames; i++) {
      int frameIdx = (startIndex + i) % bufferSize;
      TraceFrame frame = frames[frameIdx];

      // Skip empty frames
      if (frame.spanCount == 0) continue;

      for (int s = 0; s < frame.spanCount; s++) {
        TraceSpan span = frame.spans[s];
        if (!span.complete) continue;

        if (!firstEvent) {
          json.append(",\n");
        }
        firstEvent = false;

        // Chrome Trace Event format (Duration Event "X")
        // ts = timestamp in microseconds
        // dur = duration in microseconds
        // tid = thread id (we use depth for visual stacking)
        // pid = process id (always 1)
        double tsUs = span.startFPGA * 1_000_000;
        double durUs = span.getDurationMicros();

        json.append("{\"name\":\"");
        escapeJsonString(json, span.name);
        json.append("\",\"cat\":\"robot\",\"ph\":\"X\",\"ts\":");
        json.append(String.format("%.3f", tsUs));
        json.append(",\"dur\":");
        json.append(String.format("%.3f", durUs));
        json.append(",\"pid\":1,\"tid\":");
        json.append(span.depth);
        json.append(",\"args\":{\"frame\":");
        json.append(frame.frameNumber);
        json.append(",\"depth\":");
        json.append(span.depth);
        json.append("}}");
      }
    }

    json.append("\n]}");

    // Write to file
    try (PrintWriter writer = new PrintWriter(new FileWriter(path))) {
      writer.print(json.toString());
      System.out.println("[Tracer] Exported " + numFrames + " frames to " + path);
    } catch (IOException e) {
      System.err.println("[Tracer] Failed to export traces: " + e.getMessage());
    }
  }

  /** Escape special characters in JSON strings */
  private static void escapeJsonString(StringBuilder sb, String s) {
    if (s == null) {
      sb.append("null");
      return;
    }
    for (int i = 0; i < s.length(); i++) {
      char c = s.charAt(i);
      switch (c) {
        case '"':
          sb.append("\\\"");
          break;
        case '\\':
          sb.append("\\\\");
          break;
        case '\n':
          sb.append("\\n");
          break;
        case '\r':
          sb.append("\\r");
          break;
        case '\t':
          sb.append("\\t");
          break;
        default:
          sb.append(c);
      }
    }
  }
}
