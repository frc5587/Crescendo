package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ClimbWithAxis extends Command {
    private final DoubleSupplier axisSupplier;
    private final Arm arm;

    public ClimbWithAxis(DoubleSupplier axisSupplier, Arm arm) {
        this.axisSupplier = axisSupplier;
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.disable();
        arm.setManualMode(true);
    }
    
    @Override
    public void execute() {
        arm.set(-axisSupplier.getAsDouble());
        System.out.println(-axisSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        arm.enable();
    }
}
