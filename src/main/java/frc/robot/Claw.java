package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private final SparkMax clawRotationMotor = new SparkMax(7, MotorType.kBrushless);

    Claw() {
        SparkMaxConfig clawRotationConfig = new SparkMaxConfig();

        clawRotationMotor.configure(clawRotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }
}
