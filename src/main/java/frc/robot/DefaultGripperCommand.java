package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultGripperCommand extends Command{

    private final Gripper gripper; 
    
    private Supplier <Boolean> gripperSupplier;
    
    
    public DefaultGripperCommand(Gripper gripper, Supplier<Boolean> gripperSupplier) {
        this.gripper = gripper;
        this.gripperSupplier = gripperSupplier;
        addRequirements(gripper);
    }


    @Override
    public void execute() {
        gripper.grip(gripperSupplier.get());
    }

    

}