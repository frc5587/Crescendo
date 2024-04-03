package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbWithAxis extends Command {
    private final DoubleSupplier axisSupplier;
    private final Climb climb;
    private final boolean reverse;

    public ClimbWithAxis(DoubleSupplier axisSupplier, Climb climb, boolean reverse) {
        this.axisSupplier = axisSupplier;
        this.climb = climb;
        this.reverse = reverse;
    }

    @Override
    public void initialize() {
        climb.disable();
    }
    
    @Override
    public void execute() {
        climb.set(axisSupplier.getAsDouble() * (reverse ? 1 : -1));
    }

    @Override
    public void end(boolean interrupted) {
        climb.enable();
    }
}
