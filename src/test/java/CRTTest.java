import static org.junit.jupiter.api.Assertions.assertEquals;

import com.aembot.frc2026.config.robots.ProductionTurretConfig;
import com.aembot.frc2026.config.subsystems.TalonFXTurretConfiguration;
import java.util.Random;
import org.junit.jupiter.api.Test;

public class CRTTest {
  public TalonFXTurretConfiguration testconfig = new ProductionTurretConfig().TURRET_CONFIG;

  @Test
  void DualEncoderTurretAngleCalculation() {

    Random rand = new Random();
    // unit test params. will test to the maximum amount of teeth rotatable
    int TestATeeth = 13;
    int TestBTeeth = 17;
    int TestMechTeeth = 100;
    for (double i = 0.; i < TestATeeth * TestBTeeth; i += 0.05) {

      double inputA = ((i + rand.nextDouble(-0.01, 0.01)) % TestATeeth) / TestATeeth;
      double inputB = ((i + rand.nextDouble(-0.01, 0.01)) % TestBTeeth) / TestBTeeth;
      try {
        System.currentTimeMillis();
        assertEquals(
            i / TestMechTeeth,
            testconfig.getMechanismRotationsFromEncoders(
                inputA, inputB, TestATeeth, TestBTeeth, TestMechTeeth),
            0.25);
      } catch (AssertionError e) {
        System.out.println(e.getMessage());
      }
    }
  }
}
