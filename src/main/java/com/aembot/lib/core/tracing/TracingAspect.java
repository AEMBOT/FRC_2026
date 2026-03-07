package com.aembot.lib.core.tracing;

import org.aspectj.lang.ProceedingJoinPoint;
import org.aspectj.lang.annotation.Around;
import org.aspectj.lang.annotation.Aspect;

/**
 * AspectJ aspect that intercepts methods annotated with @Traced and automatically measures their
 * execution time.
 *
 * <p>This is woven into the bytecode at compile time by the AspectJ compiler, so there's no
 * reflection overhead at runtime.
 */
@Aspect
public class TracingAspect {

  /**
   * Intercept all methods annotated with @Traced and wrap them with timing code.
   *
   * @param joinPoint The join point representing the method call
   * @param traced The @Traced annotation
   * @return The result of the method call
   * @throws Throwable If the method throws an exception
   */
  @Around("@annotation(traced) && execution(* *(..))")
  public Object traceMethod(ProceedingJoinPoint joinPoint, Traced traced) throws Throwable {
    // Fast path when tracing is disabled
    if (!Tracer.isEnabled()) {
      return joinPoint.proceed();
    }

    // Determine the trace name
    String name;
    if (traced.value().isEmpty()) {
      // Default: ClassName.methodName
      name =
          joinPoint.getSignature().getDeclaringType().getSimpleName()
              + "."
              + joinPoint.getSignature().getName();
    } else {
      // Custom name from annotation
      name = traced.value();
    }

    // Begin the span
    int spanIndex = Tracer.beginSpan(name);

    try {
      // Execute the actual method
      return joinPoint.proceed();
    } finally {
      // End the span (even if method throws)
      Tracer.endSpan(spanIndex);
    }
  }
}
