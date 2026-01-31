package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase{

  private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
  private final SparkMax leftMotor = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax leftMotorFollow = new SparkMax(4, MotorType.kBrushless);
  private final SparkMax rightMotorFollow = new SparkMax(2, MotorType.kBrushless);
    private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(leftMotor::set, rightMotor::set);

    public static final double TrackWidth = Units.inchesToMeters(22);
    public static final double WheelRadius = Units.inchesToMeters(3); // meters
    public static final double WheelCircumference = WheelRadius * 2 * Math.PI;
    public static final double GEAR_RATIO = 10.71;

    DriveTrain(){
        SendableRegistry.addChild(m_robotDrive, leftMotor);
    SendableRegistry.addChild(m_robotDrive, rightMotor);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowConfig = new SparkMaxConfig();

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotorFollow.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotorFollow.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = leftMotor.getEncoder();
    
    leftConfig.encoder.positionConversionFactor(WheelCircumference / GEAR_RATIO);
    rightConfig.encoder.positionConversionFactor(WheelCircumference / GEAR_RATIO);
    leftConfig.encoder.velocityConversionFactor(WheelCircumference / GEAR_RATIO / 60);
    rightConfig.encoder.velocityConversionFactor(WheelCircumference / GEAR_RATIO / 60);
    
    rightConfig.inverted(true);
    rightFollowConfig.inverted(true);

    leftFollowConfig.follow(leftMotor);
    rightFollowConfig.follow(rightMotor);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.


    }
    public void arcadeDrive(double forward_backwards, double right_left){
        m_robotDrive.arcadeDrive(forward_backwards, right_left);
    }
}
