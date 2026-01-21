package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
    public DriveTrain drivetrain = new DriveTrain();
    public Arm arm = new Arm();
    public Claw claw = new Claw();
    private CommandJoystick joystick = new CommandJoystick(0);

    public void arcadeDrive(double forwards, double left){
        drivetrain.arcadeDrive(forwards, left);
    }

    public RobotContainer() {
        arm.setDefaultCommand(new DefaultArmCommand(arm, () -> joystick.getRawAxis(3)));
    }
    
}
