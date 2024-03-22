package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class SwerveCharacterization extends CharacterizationBase {
    private final Swerve swerve;
    private final Arm arm;
    public SwerveCharacterization(Swerve swerve, Arm arm) {
        super(MechanismType.Linear, swerve);
        this.swerve = swerve;
        this.arm = arm;
    }

    @Override
    public void setVoltage(double volts) {
        swerve.setVoltage(volts);
    }

    @Override
    public double getMotorVoltage() {
        return swerve.swerveModules[0].getDriveMotorVoltage();
    }

    @Override
    public double getMechanismPosition() {
        return swerve.swerveModules[0].getPosition().distanceMeters; // serve.getOdometryPose().getX();
    }

    @Override
    public double getMechanismVelocity() {
        return swerve.swerveModules[0].getState().speedMetersPerSecond;//swerve.getLinearVelocity();
    }

    @Override
    public double getMechanismAcceleration() {
        return swerve.getLinearAcceleration();
    }
    
    @Override
    public Command preRoutine() {
        return arm.armTravelCommand().alongWith(new InstantCommand());
    }
    
    @Override
    public Command postRoutine() {
        return new InstantCommand(swerve::stop);
    }
}