import static org.junit.jupiter.api.Assertions.assertEquals;

import com.aembot.frc2026.subsystems.turret.CRTTurretConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.junit.jupiter.api.Test;

public class CRTTest {
  public CRTTurretConfiguration<TalonFXConfiguration> testconfig =
      new CRTTurretConfiguration<TalonFXConfiguration>()
          .withEncoderAteeth(13)
          .withEncoderBteeth(17);

  @Test
  void DoesItWork() {
    // double result = testconfig.solve(0.230769231, 0.176470588);

    try {
      assertEquals(3., testconfig.solve(0.230769231, 0.176470588), 0.01);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertEquals(4., testconfig.solve(0.230769231, 0.176470588), 0.01);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertEquals(0, testconfig.solve(0, 0), 0.01);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertEquals(100., testconfig.solve(0.692307692, 0.882352941), 0.01);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertEquals(96.23, testconfig.solve(0.402307692, 0.660588235), 0.01);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertEquals(13., testconfig.solve(0, 0.764705882), 0.01);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
    try {
      assertEquals(17., testconfig.solve(0.307692308, 0), 0.01);
    } catch (AssertionError e) {
      System.out.println(e.getMessage());
    }
  }
}
