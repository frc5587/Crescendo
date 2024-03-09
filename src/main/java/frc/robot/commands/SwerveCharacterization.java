package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
        return arm.armTravelCommand();
    }
    
}
