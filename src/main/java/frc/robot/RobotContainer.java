package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
    public DriveTrain drivetrain = new DriveTrain();
    public Arm arm = new Arm();
    public Claw claw = new Claw();
    public Gripper gripper = new Gripper();
    private CommandJoystick joystick = new CommandJoystick(0);
    private CommandJoystick joystick2 = new CommandJoystick(1);

    public void arcadeDrive(double forwards, double left){
        drivetrain.arcadeDrive(forwards, left);
    }

    public RobotContainer() {
        arm.setDefaultCommand(new DefaultArmCommand(arm, () -> joystick2.getRawAxis(3), () -> joystick2.getRawAxis(1)) );
        gripper.setDefaultCommand(new DefaultGripperCommand(gripper, () -> joystick2.button(1).getAsBoolean()));
    }
    
}
