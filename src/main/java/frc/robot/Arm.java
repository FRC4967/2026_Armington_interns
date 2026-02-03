package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    // double angleSetpoint = 0;
    // private final SparkMax armAngle = new SparkMax(, MotorType.kBrushless);
    private final SparkMax armExtension = new SparkMax(6, MotorType.kBrushless);
    private final SparkMax armAngle = new SparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder armExtensionEncoder;
    private final SparkLimitSwitch armExtensionRetractedSwitch;
    private final AnalogPotentiometer armAngleEncoder = new AnalogPotentiometer(0, 325.7, -167.7);
    // private final SparkMax armRotation = new SparkMax(, MotorType.kBrushless);
    PIDController extensionPID = new PIDController(1, 0, 0);
    PIDController anglePID = new PIDController(.45, 0, 0);
    private final ArmFeedforward armFeedforward = new ArmFeedforward(0, .25, 0);
    private boolean isHoming = false;
    private double extensionSetpoint = 0;

    public Arm() {
        SparkMaxConfig armConfig = new SparkMaxConfig();
        SparkMaxConfig armAngleConfig = new SparkMaxConfig();
        armConfig.idleMode(IdleMode.kCoast);
        armAngleConfig.idleMode(IdleMode.kCoast);
        armAngle.configure(armAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armExtension.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armExtensionEncoder = armExtension.getEncoder();
        armExtensionRetractedSwitch = armExtension.getReverseLimitSwitch();
        extensionPID.setSetpoint(0);
        anglePID.setSetpoint(0);
    }

    @Override
    public void periodic() {
        double rawAngle = getArmAngle();
        SmartDashboard.putNumber("Ext. Position", armExtensionEncoder.getPosition());
        SmartDashboard.putNumber("Ext. Setpoint", extensionPID.getSetpoint());
        SmartDashboard.putNumber("Ext. Setpoint2", extensionSetpoint);
        SmartDashboard.putNumber("Raw Angle", rawAngle);
        SmartDashboard.putBoolean("Stop Switch", isArmRetracted());
        SmartDashboard.putNumber("Target Angle", anglePID.getSetpoint());
        if (RobotState.isEnabled()) {
            // double elevatorFeedForward = feedforward.calculate(0);
            double armFeedback = extensionPID.calculate(armExtensionEncoder.getPosition());
            double armVoltage = MathUtil.clamp(armFeedback, -4, 4);
            double armAngleFeedback = anglePID.calculate(rawAngle);
            double armGain = armFeedforward.calculate(Math.toRadians(rawAngle), 0);
            double armAngleVoltage = MathUtil.clamp(armAngleFeedback + armGain, -12, 12);
            armAngle.setVoltage(armAngleVoltage);
            if (isHoming) {
                if (armExtensionRetractedSwitch.isPressed()) {
                    armExtension.setVoltage(0);
                    isHoming = false;
                    armExtensionEncoder.setPosition(0);
                } else {
                    armExtension.setVoltage(-3);
                }
            } else {
                armExtension.setVoltage(armVoltage);
            }
        }
    }

    public void runExtension(boolean up, boolean down) {
        if (up) {
            extensionSetpoint += 1;
            extendTo(extensionSetpoint);
        } else if (down) {
            extensionSetpoint -= 1;
            extendTo(extensionSetpoint);
        }
    }

    public void extendTo(double position) {
        double extensionSetpoint = position;
        extensionSetpoint = MathUtil.clamp(extensionSetpoint, 0, 250);
        extensionPID.setSetpoint(extensionSetpoint);
    }

    public void setArmAngleRelative(double percentage) {
        double newAngleSetpoint = anglePID.getSetpoint() + percentage;
        setArmAngle(newAngleSetpoint);
    }

    public void setArmAngle(double degrees) {
        if (degrees > -179 || degrees < 175) {
            anglePID.setSetpoint(degrees);
        }
    }

    private double potTicsToAngle(double tics) {
        return (tics * 360) / 64;
    }

    public double getArmAngle() {
        double rawAngle = (armAngleEncoder.get() * 6);
        return rawAngle;
    }

    public boolean isArmRetracted() {
        return armExtensionRetractedSwitch.isPressed();
    }

    public void activateHomingProtocall() {
        isHoming = true;
    }
}