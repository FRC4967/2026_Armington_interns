package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    // private final SparkMax armAngle = new SparkMax(, MotorType.kBrushless);
    private final SparkMax armExtension = new SparkMax(6, MotorType.kBrushless);
    private final SparkMax armAngle = new SparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder armExtensionEncoder;
    private final AnalogPotentiometer armAngleEncoder = new AnalogPotentiometer(0,325.7, -167.2);
    // private final SparkMax armRotation = new SparkMax(, MotorType.kBrushless);
    PIDController extensionPID = new PIDController(1, 0, 0);
    PIDController anglePID = new PIDController(1, 0, 0);

    public Arm() {
        SparkMaxConfig armConfig = new SparkMaxConfig();
        SparkMaxConfig armAngleConfig = new SparkMaxConfig();
        armConfig.idleMode(IdleMode.kCoast);
        armAngleConfig.idleMode(IdleMode.kCoast);
        armAngle.configure(armAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armExtension.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armExtensionEncoder = armExtension.getEncoder();
        extensionPID.setSetpoint(0);
        anglePID.setSetpoint(0);
    }

    @Override
    public void periodic() {
        double rawAngle = armAngleEncoder.get() * 6;
        SmartDashboard.putNumber("Ext. Position", armExtensionEncoder.getPosition());
        SmartDashboard.putNumber("Raw Angle", rawAngle);
        if (RobotState.isEnabled()) {
            // double elevatorFeedForward = feedforward.calculate(0);
            double armFeedback = extensionPID.calculate(armExtensionEncoder.getPosition());
            double armVoltage = MathUtil.clamp(armFeedback, -4, 4);
            armExtension.setVoltage(armVoltage);
        }
    }
    public void extendTo(double percentage){
        percentage += 1;
        double extensionSetpoint = percentage*250;
        if(extensionSetpoint > 250){
            extensionSetpoint =250;
        }
        extensionPID.setSetpoint(extensionSetpoint);
    }
    public void tiltTo(double percentage){
        percentage += 1;
        
        //anglePID.setSetpoint(angleSetpoint);
    }
    private double potTicsToAngle(double tics) {
        return (tics * 360) / 64;
    }
}
/*
 * Arm(){
 * SendableRegistry.addChild(m_robotArm, armAngle);
 * SendableRegistry.addChild(m_robotArm, armExtension);
 * SendableRegistry.addChild(m_robotArm, armRotation);
 * 
 * SparkMaxConfig armAngleConfig = new SparkMaxConfig();
 * SparkMaxConfig armExtensionConfig = new SparkMaxConfig();
 * SparkMaxConfig armRotationConfig = new SparkMaxConfig();
 * 
 * 
 * armAngle.configure(armAngleConfig, ResetMode.kResetSafeParameters,
 * PersistMode.kPersistParameters);
 * armExtension.configure(armExtensionConfig, ResetMode.kResetSafeParameters,
 * PersistMode.kPersistParameters);
 * armRotation.configure(armRotationConfig, ResetMode.kResetSafeParameters,
 * PersistMode.kPersistParameters);
 * }
 * PIDController.
 */
