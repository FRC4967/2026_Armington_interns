package frc.robot;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetButton extends Command{
    private final Arm arm; 
    private final Claw claw;
    
    
    public ResetButton(Arm arm, Claw claw) {
        this.arm = arm;
        this.claw = claw;
        addRequirements(arm);
        addRequirements(claw);
    }


    @Override
    public void execute() {
        claw.ClawGoTo(0);
        arm.extendTo(-1);
        arm.shoulderToManualControl(0);
    }
}
