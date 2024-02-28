package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve;

public class AutoRotateToShoot extends Command {
    private final Swerve swerve;
    private boolean isFinished = false;

    public AutoRotateToShoot(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();
        Rotation2d currentAngle = currentPose.getRotation().plus(Rotation2d.fromDegrees(180.));
        Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(
                currentPose.getY() - (DriverStation.getAlliance().get().equals(Alliance.Blue) ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION).getY(),
                currentPose.getX() - (DriverStation.getAlliance().get().equals(Alliance.Blue) ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION).getX()
            )).plus(Rotation2d.fromDegrees(180.));
            
            swerve.drive(new Translation2d(0, 0),
                            Math.copySign(1., currentAngle.getDegrees() - targetAngle.getDegrees()),
                            true, true);
            if(Math.abs(currentAngle.getDegrees() - targetAngle.getDegrees()) < 1.) {
                isFinished = true;
            }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
