package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private final SparkMax clawRotationMotor = new SparkMax(7, MotorType.kBrushless);

    public Wrist() {
        SparkMaxConfig clawRotationConfig = new SparkMaxConfig();

        clawRotationMotor.configure(clawRotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void runWrist(boolean up, boolean down){
        if (up == true){
            clawRotationMotor.setVoltage(1);
        } else if (down == true){
            clawRotationMotor.setVoltage(-1);
        } else {
            clawRotationMotor.setVoltage(0);
        }
    }
}
