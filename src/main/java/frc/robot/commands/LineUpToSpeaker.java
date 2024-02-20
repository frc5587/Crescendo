package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve;

public class LineUpToSpeaker extends Command {
    private final Swerve swerve;
    private boolean rotatingDone, strafingDone = false;

    public LineUpToSpeaker(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        if(!rotatingDone) {
            if (swerve.getPose().getRotation().getDegrees() < 0 ) {
        swerve.setChassisSpeeds(new ChassisSpeeds(0,0,1));
            }
            else if (swerve.getPose().getRotation().getDegrees() > 0) {
        swerve.setChassisSpeeds(new ChassisSpeeds(0,0,-1));
            }
            if(swerve.getPose().getRotation().equals(Rotation2d.fromDegrees(-90))) {
                rotatingDone = true;
            }
        }
        else {
            if(DriverStation.getAlliance().equals(Alliance.Blue)) {
                if (Math.abs(swerve.getPose().getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()) > 0.25) {
                swerve.setChassisSpeeds(new ChassisSpeeds(.25, 0, 0));
                }
                if (Math.abs(swerve.getPose().getY() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY()) > 0.25) {
                    swerve.setChassisSpeeds(new ChassisSpeeds(0, .25, 0));
                }
                if (Math.abs(swerve.getPose().getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()) < 0.25 && Math.abs(swerve.getPose().getY() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY()) < 0.25) {
                    strafingDone = true; }
            }
            else if(DriverStation.getAlliance().equals(Alliance.Red))
            {

                if (Math.abs(swerve.getPose().getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()) > 0.25) {
                    swerve.setChassisSpeeds(new ChassisSpeeds(.25, 0, 0));
                    }

                if (Math.abs(swerve.getPose().getY() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY()) > 0.25) {
                    swerve.setChassisSpeeds(new ChassisSpeeds(0, .25, 0));
                    }

            if (Math.abs(swerve.getPose().getX() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getX()) < 0.25 && Math.abs(swerve.getPose().getY() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getY()) < 0.25) {
                strafingDone = true; }   
                    }
                 }
    
    }


    @Override
    public boolean isFinished() {
        if (rotatingDone && strafingDone) {
        return true; //robot lined up
        }
        else {
        return false; 
        }
    }
}
