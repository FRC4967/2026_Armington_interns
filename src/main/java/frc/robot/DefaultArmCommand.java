package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultArmCommand extends Command {

    private final Arm arm;
    private Supplier<Boolean> upSupplier;
    private Supplier<Boolean> downSupplier;
    private Supplier<Double> angleSupplier;

    public DefaultArmCommand(Arm arm, Supplier<Boolean> upSupplier, Supplier<Boolean> downSupplier,
            Supplier<Double> angleSupplier) {
        this.arm = arm;
        this.upSupplier = upSupplier;
        this.downSupplier = downSupplier;
        this.angleSupplier = angleSupplier;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double angleDeadband = MathUtil.applyDeadband(angleSupplier.get(), .1);
        arm.runExtension(upSupplier.get(), downSupplier.get());
        arm.setArmAngleRelative(-angleDeadband);
    }

}
