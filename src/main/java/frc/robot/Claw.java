package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private final SparkMax clawRotationMotor = new SparkMax(7, MotorType.kBrushless);
    private final RelativeEncoder wristEncoder;
    private final PIDController wristPid = new PIDController(.25, 0, 0);
    private double clawSetpoint = 0;

    public Claw() {
        SparkMaxConfig clawRotationConfig = new SparkMaxConfig();
        clawRotationMotor.configure(clawRotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wristEncoder = clawRotationMotor.getEncoder();
    }

    // public void runClaw(boolean up, boolean down) {
    //     if (up == true) {
    //         clawRotationMotor.setVoltage(1);
    //     } else if (down == true) {
    //         clawRotationMotor.setVoltage(-1);
    //     } else {
    //         clawRotationMotor.setVoltage();
    //     }
    // }
    public void runClaw(boolean up, boolean down) {
      if (up == true) {
        clawSetpoint += 1;
      }
      else if (down == true) {
        clawSetpoint -= 1;
      } 
      ClawGoTo(clawSetpoint);
    } 

    public void ClawGoTo(double position){
        clawSetpoint = MathUtil.clamp(position, -62, 37);
        wristPid.setSetpoint(clawSetpoint);
    }
    
    @Override
    public void periodic() {
       SmartDashboard.putNumber("wristAngle", wristEncoder.getPosition());

       if(RobotState.isEnabled()) {
            double clawVoltage = wristPid.calculate(wristEncoder.getPosition());
            clawRotationMotor.setVoltage(clawVoltage);
       }
    }
    
}
