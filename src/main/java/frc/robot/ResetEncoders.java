package frc.robot;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetEncoders extends Command{
    private final DriveTrain driveTrain;
    
    
    public ResetEncoders(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }


    @Override
    public void execute() {
        driveTrain.resetEncoders();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
