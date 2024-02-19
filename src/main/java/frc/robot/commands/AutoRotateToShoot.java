package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve;

public class AutoRotateToShoot extends Command {
    private final Swerve swerve;
    private Rotation2d targetAngle;

    public AutoRotateToShoot(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        if(DriverStation.getAlliance().equals(Alliance.Blue)) {
            targetAngle = swerve.getPose().relativeTo(FieldConstants.BLUE_SUBWOOFER_FRONT_POSE).getTranslation().getAngle();
            if(swerve.getYaw().getDegrees() > targetAngle.getDegrees()) {
                swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, -0.05));
            }
            else {
                swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0.05));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getYaw().getDegrees() - targetAngle.getDegrees()) < 2;
    }
}
