// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  private RobotContainer container = new RobotContainer();
  // private final SparkMax armAngle = new SparkMax(, MotorType.kBrushless);
  // private final SparkMax armExtension = new SparkMax(, MotorType.kBrushless);
  // private final SparkMax armRotation = new SparkMax(, MotorType.kBrushless);
  private final Joystick stick = new Joystick(0);

  /** Called once at the beginning of the robot program. */
  public Robot() {

  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    container.arcadeDrive(stick.getX(), stick.getY());
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  /*
   * @Override
   * public void autonomousPeriodic() {
   * // Nova's test auto class
   * 
   * SwerveControllerCommand swerveControllerCommand =
   * new SwerveControllerCommand(
   * goForward2,
   * m_robotDrive::getPose, // Functional interface to feed supplier
   * DriveConstants.kDriveKinematics,
   * 
   * // Position controllers
   * new PIDController(AutoConstants.kPXController, 0, 0),
   * new PIDController(AutoConstants.kPYController, 0, 0),
   * thetaController,
   * m_robotDrive::setModuleStates,
   * m_robotDrive);
   * new InstantCommand(() ->
   * m_robotDrive.resetOdometry(goForward2.getInitialPose())),
   * }
   */
}
