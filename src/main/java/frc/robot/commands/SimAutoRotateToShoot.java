package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SimSwerve;
import frc.robot.subsystems.Swerve;

public class SimAutoRotateToShoot extends Command {
    private final SimSwerve swerve;
    private Rotation2d targetAngle = new Rotation2d();
    private Field2d targetField = new Field2d();
    private Pose2d swervePose = new Pose2d();

    public SimAutoRotateToShoot(SimSwerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        Field2d speakerLoc = new Field2d();
        speakerLoc.setRobotPose(FieldConstants.BLUE_SUBWOOFER_FRONT_POSE);
        SmartDashboard.putData("Speaker Location", speakerLoc);
    }

    @Override
    public void execute() {
        if(DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            swervePose = swerve.getPose();
            targetAngle = Rotation2d.fromRadians(Math.atan2(
                swervePose.getY() - FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY(),
                swervePose.getX() - FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
            ));
            if(swerve.getPose().getRotation().getDegrees() > targetAngle.getDegrees()) {
                swerve.runVelocity(new ChassisSpeeds(0, 0, -1));
            }
            else {
                swerve.runVelocity(new ChassisSpeeds(0, 0, 1));
            }
            System.out.println("TARGET " + targetAngle);
            System.out.println("ANGLE " + swerve.getPose().getRotation());
            
        }
        targetField.setRobotPose(swerve.getPose().getX(), swerve.getPose().getY(), targetAngle);
        SmartDashboard.putData("Target", targetField);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getPose().getRotation().getDegrees() - targetAngle.getDegrees()) < 2;
    }
}
