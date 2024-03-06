package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;

public class SwerveCharacterization extends CharacterizationBase {
    private final Swerve swerve;
    public SwerveCharacterization(Swerve swerve) {
        super(MechanismType.Linear, swerve);
        this.swerve = swerve;
        // SysIdRoutine routine = new SysIdRoutine(
        //     new SysIdRoutine.Config(),
        //     new SysIdRoutine.Mechanism(this::setVoltage, this::log, this)
        // );
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
        return swerve.getModuleStates()[0].speedMetersPerSecond;
    }

    @Override
    public double getMechanismAcceleration() {
        return swerve.getModuleStates()[0].speedMetersPerSecond;
    }
    
}
