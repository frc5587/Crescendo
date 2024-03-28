package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;

public class ClimbWithAxis extends Command {
    private final DoubleSupplier axisSupplier;
    private final Arm arm;
    private final Climb climb;

    public ClimbWithAxis(DoubleSupplier axisSupplier, Arm arm, Climb climb) {
        this.axisSupplier = axisSupplier;
        this.arm = arm;
        this.climb = climb;
    }

    @Override
    public void initialize() {
        // arm.disable();
        // arm.setManualMode(true);
        climb.disable();
    }
    
    @Override
    public void execute() {
        // arm.set(-axisSupplier.getAsDouble());
        climb.set(-axisSupplier.getAsDouble());
        // System.out.println(-axisSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // arm.enable();
        climb.enable();
    }
}
