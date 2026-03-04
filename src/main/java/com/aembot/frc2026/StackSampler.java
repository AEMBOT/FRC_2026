package com.aembot.frc2026;

import java.io.*;
import java.lang.management.*;
import java.util.concurrent.atomic.*;

public class StackSampler {

  private static final int MAX_SAMPLES = 20; // ~100ms @ 5ms
  private static final int SAMPLE_INTERVAL_MS = 5;
  private static final int MAX_DEPTH = 20;

  private final ThreadMXBean bean;
  private final long targetThreadId;

  private final Sample[] ring = new Sample[MAX_SAMPLES];
  private final AtomicInteger writeIndex = new AtomicInteger(0);

  private volatile boolean running = true;

  static class Sample {
    long timestampNs;
    Thread.State state;
    StackTraceElement[] stack;
  }

  public StackSampler(Thread targetThread) {
    this.bean = ManagementFactory.getThreadMXBean();
    this.targetThreadId = targetThread.getId();

    for (int i = 0; i < MAX_SAMPLES; i++) {
      ring[i] = new Sample();
    }
  }

  public void start() {
    Thread t = new Thread(this::runSampler, "StackSampler");
    t.setDaemon(true);
    t.setPriority(Thread.MAX_PRIORITY);
    t.start();
  }

  private void runSampler() {
    while (running) {
      long ts = System.nanoTime();

      ThreadInfo info = bean.getThreadInfo(targetThreadId, MAX_DEPTH);

      if (info != null && writeIndex.get() != -1) {
        int idx = writeIndex.getAndIncrement() % MAX_SAMPLES;
        Sample s = ring[idx];

        s.timestampNs = ts;
        s.state = info.getThreadState();
        s.stack = info.getStackTrace();
      }

      // 5ms sleep
      try {
        Thread.sleep(SAMPLE_INTERVAL_MS);
      } catch (InterruptedException ignored) {
      }
    }
  }

  public void printLog(double overrunAmount) {
    /*
    int samplesToPrint = (int) (overrunAmount / SAMPLE_INTERVAL_MS);
    int end = writeIndex.get();
    int start = (end - samplesToPrint) % MAX_SAMPLES;
    */
    // Don't want writing the trace out to be counted in the timestamp logs :(
    writeIndex.set(-1);
    for (int i = 0; i < MAX_SAMPLES; i++) {
      Sample s = ring[i];
      if (s == null || s.stack == null) {
        continue;
      }

      System.out.println(s.timestampNs + "," + s.state);

      for (StackTraceElement e : s.stack) {
        System.out.println("    at " + e);
      }
      System.out.println();
    }

    writeIndex.set(0);
  }

  /*
  public void dumpToFile(String filename) throws IOException {
    try (BufferedWriter w = new BufferedWriter(new FileWriter(filename))) {
      int end = writeIndex.get();
      int start = Math.max(0, end - MAX_SAMPLES);

      for (int i = start; i < end; i++) {
        Sample s = ring[i % MAX_SAMPLES];

        w.write(s.timestampNs + "," + s.state + "\n");

        for (StackTraceElement e : s.stack) {
          w.write("    at " + e + "\n");
        }
        w.write("\n");
      }
    }
  }
    */

  public void stop() {
    running = false;
  }
}
