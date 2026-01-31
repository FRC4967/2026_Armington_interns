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
        arm.setDefaultCommand(new DefaultArmCommand(arm, () -> joystick2.button(7).getAsBoolean(), () -> joystick2.button(8).getAsBoolean(), () -> joystick2.getRawAxis(1)));
        claw.setDefaultCommand(new DefaultWristCommand(claw, () -> joystick2.povUp().getAsBoolean(), () -> joystick2.povDown().getAsBoolean()));
        gripper.setDefaultCommand(new DefaultGripperCommand(gripper, () -> joystick2.button(1).getAsBoolean()));
        configureBindings();
    }

    private void configureBindings(){
        joystick2.button(2).onTrue(new ResetButton(arm, claw));
        joystick2.button(3).onTrue(new pickUpButton(arm, claw));
        joystick2.button(4).onTrue(new PlaceButton(arm, claw));
        joystick.button(6).onTrue(new ExtensionHoming(arm));
    }
    
}
