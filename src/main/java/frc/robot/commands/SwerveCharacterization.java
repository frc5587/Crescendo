package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;

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
    
    @Override
    public Command preRoutine() {
        return new InstantCommand(arm::travelSetpoint);
    }
    
}
