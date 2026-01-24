package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultWristCommand extends Command {

    private final Claw wrist;
    private Supplier<Boolean> upSupplier;
    private Supplier<Boolean> downSupplier;

    public DefaultWristCommand(Claw wrist, Supplier<Boolean> upSupplier, Supplier<Boolean> downSupplier) {
        this.wrist = wrist;
        this.upSupplier = upSupplier;
        this.downSupplier = downSupplier;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.runClaw(upSupplier.get(), downSupplier.get());

    }

}