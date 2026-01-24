package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultArmCommand extends Command{

    private final Arm arm; 
    private Supplier <Double> extensionSupplier;
    private Supplier <Double> angleSupplier;
    
    
    public DefaultArmCommand(Arm arm, Supplier<Double> extensionSupplier, Supplier<Double> angleSupplier) {
        this.arm = arm;
        this.extensionSupplier = extensionSupplier;
        this.angleSupplier = angleSupplier;
        addRequirements(arm);
    }


    @Override
    public void execute() {
        double angleDeadband = MathUtil.applyDeadband(angleSupplier.get(), .1); 
        arm.extendTo(-extensionSupplier.get());
        arm.shoulderToManualControl(-angleDeadband);
    }

    

}
