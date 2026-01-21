package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultArmCommand extends Command{

    private final Arm arm; 
    private Supplier <Double> extensionSupplier;
    
    
    public DefaultArmCommand(Arm arm, Supplier<Double> extensionSupplier) {
        this.arm = arm;
        this.extensionSupplier = extensionSupplier;
        addRequirements(arm);
    }


    @Override
    public void execute() {
        arm.extendTo(extensionSupplier.get());
    }

    

}
