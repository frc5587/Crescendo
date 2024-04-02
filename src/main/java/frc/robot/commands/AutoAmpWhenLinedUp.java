package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoAmpWhenLinedUp extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final BooleanSupplier readySupplier;

    public AutoAmpWhenLinedUp(Shooter shooter, Intake intake, BooleanSupplier readySupplier) {
        this.shooter = shooter;
        this.intake = intake;
        this.readySupplier = readySupplier;
    }

    @Override
    public void initialize() {
        shooter.pancake();
    }

    @Override
    public void execute() {
        if(readySupplier.getAsBoolean()) {
            intake.forward();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        shooter.enable();
        shooter.idleSpeed();
    }
}
