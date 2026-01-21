/*package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armCode extends SubsystemBase implements Sendable{

    public double m_deadband = 0.1;
    public void armCode(double armExtend, double armRotate, double armAngled, boolean squareInputs) {

    armAngled = MathUtil.applyDeadband(armRotate, m_deadband);
    armExtend = MathUtil.applyDeadband(armExtend, m_deadband);
    armRotate = MathUtil.applyDeadband(armRotate, m_deadband);

    var speeds = arcadeDriveSpeeds(armAngled, armExtend, armRotate, squareInputs);

    armA_Output = speeds.armAngle;
    armE_Output = speeds.armExtend; 
    armR_Output = speeds.armRotate; 

    armAngle.accept(armA_Output);
    armExtension.accept(armE_Output);
    armRotation.accept(armR_Output);

    feed();
  }
  public static speeds arcadeDriveSpeeds(double armExtend, double armRotate, double armAngled, boolean squareInputs){
    armExtend = MathUtil.clamp(armExtend, -1.0, 1.0);
    armRotate = MathUtil.clamp(armRotate, -1.0, 1.0);
    armAngled = MathUtil.clamp(armAngled, -1.0, 1.0);

    if (squareInputs) {
      armExtend = MathUtil.copyDirectionPow(armExtend, 2);
      armRotate = MathUtil.copyDirectionPow(armRotate, 2);
      armAngled = MathUtil.copyDirectionPow(armAngled, 2);
    }


    return new speeds(armExtend, armRotate, armAngled);
  }
}*/
