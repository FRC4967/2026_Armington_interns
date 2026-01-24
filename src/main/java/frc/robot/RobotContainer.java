package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
    public DriveTrain drivetrain = new DriveTrain();
    public Arm arm = new Arm();
    public Claw wrist = new Claw();
    private CommandJoystick joystick = new CommandJoystick(0);
    private CommandJoystick joystick2 = new CommandJoystick(1);

    public void arcadeDrive(double forwards, double left){
        drivetrain.arcadeDrive(forwards, left);
    }

    public RobotContainer() {
        arm.setDefaultCommand(new DefaultArmCommand(arm, () -> joystick2.getRawAxis(3), () -> joystick2.getRawAxis(1)));
        wrist.setDefaultCommand(new DefaultWristCommand(wrist, () -> joystick2.povUp().getAsBoolean(), () -> joystick2.povDown().getAsBoolean()));
    }
    
}
