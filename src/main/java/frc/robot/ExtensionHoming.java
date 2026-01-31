package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class ExtensionHoming extends Command {
    private final Arm arm; 

    public ExtensionHoming(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

     @Override
    public void execute() {
        arm.activateHomingProtocall();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
