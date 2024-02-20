package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SimSwerve;

public class SimLineUpToSpeaker extends Command {
    private final SimSwerve swerve;
    private boolean rotatingDone, strafingDone = false;

    public SimLineUpToSpeaker(SimSwerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();
        if(!rotatingDone) {
            if(DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(
                currentPose.getY() - FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY(),
                currentPose.getX() - FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
            ));
            if(currentPose.getRotation().getDegrees() > targetAngle.getDegrees()) {
                swerve.runVelocity(new ChassisSpeeds(0, 0, -1));
            }
            else {
                swerve.runVelocity(new ChassisSpeeds(0, 0, 1));
            }
            if(Math.abs(currentPose.getRotation().getDegrees() - targetAngle.getDegrees()) < 2) {
                rotatingDone = true;
            }
        }
        }
        else {
            if(DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                if (Math.abs(currentPose.getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()) > 0.25) {
                swerve.runVelocity(new ChassisSpeeds(Math.copySign(1., currentPose.getX() - FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()), 0, 0));
                }
                if (Math.abs(currentPose.getY() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY()) > 0.25) {
                    swerve.runVelocity(new ChassisSpeeds(0, -Math.copySign(1., currentPose.getY() - FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY()), 0));
                }
                if (Math.abs(currentPose.getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()) < 0.25 && Math.abs(currentPose.getY() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY()) < 0.25) {
                    strafingDone = true;
                }
            }
            else if(DriverStation.getAlliance().get().equals(Alliance.Red))
            {

                if (Math.abs(currentPose.getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()) > 0.25) {
                    swerve.runVelocity(new ChassisSpeeds(.25, 0, 0));
                    }

                if (Math.abs(currentPose.getY() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY()) > 0.25) {
                    swerve.runVelocity(new ChassisSpeeds(0, .25, 0));
                    }

            if (Math.abs(currentPose.getX() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getX()) < 0.25 && Math.abs(currentPose.getY() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getY()) < 0.25) {
                strafingDone = true; }   
                    }
                 }
    
    }


    @Override
    public boolean isFinished() {
        return rotatingDone && strafingDone;
    }
}
