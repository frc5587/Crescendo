package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class ShooterCharacterization extends CharacterizationBase {
    private final Shooter shooter;
    
    public ShooterCharacterization(Shooter shooter) {
        super(MechanismType.Linear, shooter);
        this.shooter = shooter;
    }

    @Override
    public void setVoltage(double volts) {
        shooter.setVoltage(volts);
    }

    @Override
    public double getMotorVoltage() {
        return shooter.getVoltage();
    }

    @Override
    public double getMechanismPosition() {
        return shooter.getPositionMeters();
    }

    @Override
    public double getMechanismVelocity() {
        return shooter.getWheelSpeedsMPS();
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
