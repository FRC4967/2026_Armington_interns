package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  public final DifferentialDriveOdometry odometry;
  private final SparkMax leftMotor = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax leftMotorFollow = new SparkMax(4, MotorType.kBrushless);
  private final SparkMax rightMotorFollow = new SparkMax(2, MotorType.kBrushless);
  private final AHRS gyro;
  private final Field2d field;
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotor::set, rightMotor::set);

  public static final double TrackWidth = Units.inchesToMeters(22);
  public static final double WheelRadius = Units.inchesToMeters(2.98); // meters
  public static final double WheelCircumference = WheelRadius * 2 * Math.PI;
  public static final double GEAR_RATIO = 10.71;

  DriveTrain() {
    SendableRegistry.addChild(m_robotDrive, leftMotor);
    SendableRegistry.addChild(m_robotDrive, rightMotor);

    field = new Field2d();

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowConfig = new SparkMaxConfig();

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftConfig.encoder.positionConversionFactor(WheelCircumference / GEAR_RATIO);
    rightConfig.encoder.positionConversionFactor(WheelCircumference / GEAR_RATIO);
    leftConfig.encoder.velocityConversionFactor(WheelCircumference / GEAR_RATIO / 60);
    rightConfig.encoder.velocityConversionFactor(WheelCircumference / GEAR_RATIO / 60);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotorFollow.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotorFollow.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightConfig.inverted(true);
    rightFollowConfig.inverted(true);

    leftFollowConfig.follow(leftMotor);
    rightFollowConfig.follow(rightMotor);

    gyro = new AHRS(NavXComType.kUSB1);
    gyro.zeroYaw();
    gyro.reset();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    odometry = new DifferentialDriveOdometry(new Rotation2d(gyro.getYaw()), leftEncoder.getPosition(),
        rightEncoder.getPosition());

    SmartDashboard.putData("Field", field);

  }

  public void arcadeDrive(double forward_backwards, double right_left) {
    m_robotDrive.arcadeDrive(forward_backwards, right_left);
  }

  @Override
  public void periodic() {
    updateOdometry();
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putNumber("gyro", gyro.getAngle());
    SmartDashboard.putNumber("Left Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", rightEncoder.getPosition());
    
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void resetPose(Pose2d newPose) {
    odometry.resetPose(newPose);
  }

  public Double getHeading() {
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getGyro() {
    return Rotation2d.fromDegrees(-getHeading());
  }

  public void updateOdometry() {
    odometry.update(
        getGyro(),
        leftEncoder.getPosition(),
        -rightEncoder.getPosition());
  }
}
