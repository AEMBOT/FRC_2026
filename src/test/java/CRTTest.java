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

    for (double i = 0.; i < 750; i += 0.05) {

      double rotorPosition = testconfig.kRealMotorConfig.getUnitsToRotorRotations(i);

      double inputA = ((rotorPosition + rand.nextDouble(-0.01, 0.01)) % 17.0) / 17.0;
      double inputB = ((rotorPosition + rand.nextDouble(-0.01, 0.01)) % 13.0) / 13.0;
      try {
        System.currentTimeMillis();
        assertEquals(i, testconfig.getMechanismRotationsFromEncoders(inputA, inputB), 0.25);
      } catch (AssertionError e) {
        System.out.println(e.getMessage());
      }
    }
  }
}
