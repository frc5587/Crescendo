package frc.robot.commands;

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
    
}
