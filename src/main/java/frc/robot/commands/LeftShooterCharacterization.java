package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class LeftShooterCharacterization extends CharacterizationBase {
    private final Shooter shooter;
    
    public LeftShooterCharacterization(Shooter shooter) {
        super(MechanismType.Linear, shooter);
        this.shooter = shooter;
    }

    @Override
    public void setVoltage(double volts) {
        shooter.setLeftVoltage(volts);
    }

    @Override
    public double getMotorVoltage() {
        return shooter.getLeftVoltage();
    }

    @Override
    public double getMechanismPosition() {
        return shooter.getPositionMeters();
    }

    @Override
    public double getMechanismVelocity() {
        return shooter.getLeftMPS();
    }

    @Override
    public double getMechanismAcceleration() {
        return 0.0;
    }

    @Override
    public Command preRoutine() {
        return new InstantCommand(() -> {
            shooter.disable();
            shooter.stopVoltage();
        }).andThen(new WaitCommand(1));
    }

    @Override
    public Command postRoutine() {
        return new InstantCommand(shooter::stopVoltage);
    }
    
}
