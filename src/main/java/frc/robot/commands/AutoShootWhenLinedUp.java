package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShootWhenLinedUp extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final BooleanSupplier readySupplier;

    public AutoShootWhenLinedUp(Shooter shooter, Intake intake, BooleanSupplier readySupplier) {
        this.shooter = shooter;
        this.intake = intake;
        this.readySupplier = readySupplier;
        SmartDashboard.putBoolean("AutoShoot Running", false);
    }

    @Override
    public void initialize() {
        shooter.forward();
        SmartDashboard.putBoolean("AutoShoot Running", true);
    }

    @Override
    public void execute() {
        if(readySupplier.getAsBoolean() && shooter.isSpunUp()) {
            intake.forward();
        }
        SmartDashboard.putBoolean("AutoShoot Running", true);
    }
    
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("AutoShoot Running", false);
        shooter.idleSpeed();
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return !intake.getLimitSwitch();
    }
}