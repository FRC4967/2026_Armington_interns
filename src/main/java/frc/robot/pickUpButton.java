package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class pickUpButton extends Command {

    private final Arm arm;
    private final Claw claw;

    public pickUpButton(Arm arm, Claw claw) {
        this.arm = arm;
        this.claw = claw;
        addRequirements(arm);
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.ClawGoTo(-27);
        arm.extendTo(73);
        arm.setArmAngle(150);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}