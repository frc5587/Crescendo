package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;

public class SwerveCharacterization extends CharacterizationBase {
    private final Swerve swerve;
    public SwerveCharacterization(Swerve swerve) {
        super(MechanismType.Linear, swerve);
        this.swerve = swerve;
    }

    @Override
    public void setVoltage(double volts) {
        for (SwerveModule module : swerve.swerveModules) {
            module.setDriveMotorVoltage(volts);
            module.setAngle(new Rotation2d());
        }
    }

    @Override
    public double getMotorVoltage() {
        return swerve.swerveModules[0].getDriveMotorVoltage();
    }

    @Override
    public double getMechanismPosition() {
        return swerve.getOdometryPose().getX();
    }

    @Override
    public double getMechanismVelocity() {
        return swerve.getLinearVelocity();
    }

    @Override
    public double getMechanismAcceleration() {
        return swerve.getLinearAcceleration();
    }
    
}
