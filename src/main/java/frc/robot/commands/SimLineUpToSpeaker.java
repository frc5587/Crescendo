package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public void initialize() {
        rotatingDone = false;
        strafingDone = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(
                    currentPose.getY() - FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY(),
                    currentPose.getX() - FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()));
                    System.out.println("TARGET " + targetAngle.getDegrees());
                    System.out.println("CURRENT " + currentPose.getRotation().getDegrees());
            if (Math.abs(currentPose.getRotation().getDegrees() - targetAngle.getDegrees()) < 2) {
                rotatingDone = true;
            }
            else {
                rotatingDone = false;
            }
            if (Math.abs(currentPose.getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()) < 0.1
                        && Math.abs(currentPose.getY() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY()) < 0.1) {
                    strafingDone = true;
            }
            else {
                strafingDone = false;
            }
            if ((Math.abs(currentPose.getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()) > 0.1)
                    && (Math.abs(currentPose.getY() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY()) > 0.1)) {
                swerve.drive(new Translation2d(
                        -Math.copySign(1., currentPose.getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()),
                        -Math.copySign(1., currentPose.getY() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY())),
                        rotatingDone ? 0 : -Math.copySign(1., currentPose.getRotation().getDegrees() - targetAngle.getDegrees()),
                        true, true);
            }
                else if (Math.abs(currentPose.getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()) > 0.1) {
                    swerve.drive(new Translation2d(
                            -Math.copySign(1., currentPose.getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getX()),
                            0),
                            rotatingDone ? 0 : -Math.copySign(1., currentPose.getRotation().getDegrees() - targetAngle.getDegrees()),
                            true, true);
                }
                else if (Math.abs(currentPose.getY() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY()) > 0.1) {
                    swerve.drive(new Translation2d(0, -Math.copySign(1.,
                            currentPose.getX() - FieldConstants.BLUE_SUBWOOFER_FRONT_POSE.getY())),
                            rotatingDone ? 0 : -Math.copySign(1., currentPose.getRotation().getDegrees() - targetAngle.getDegrees()),
                            true, true);
                }
                else if(!rotatingDone) {
                    swerve.drive(new Translation2d(0, 0),
                            -Math.copySign(1., currentPose.getRotation().getDegrees() - targetAngle.getDegrees()),
                            true, true);
                }
            }

        else if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(
                    currentPose.getY() - FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getY(),
                    currentPose.getX() - FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getX()));
                    System.out.println("TARGET " + targetAngle.getDegrees());
                    System.out.println("CURRENT " + currentPose.getRotation().getDegrees());
            if (Math.abs(currentPose.getRotation().getDegrees() - targetAngle.getDegrees()) < 2) {
                rotatingDone = true;
            }
            else {
                rotatingDone = false;
            }
            if (Math.abs(currentPose.getX() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getX()) < 0.1
                        && Math.abs(currentPose.getY() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getY()) < 0.1) {
                    strafingDone = true;
                }
                else {
                    strafingDone = false;
                }
            if ((Math.abs(currentPose.getX() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getX()) > 0.1)
                    && (Math.abs(currentPose.getY() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getY()) > 0.1)) {
                swerve.drive(new Translation2d(
                        -Math.copySign(1., currentPose.getX() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getX()),
                        -Math.copySign(1., currentPose.getY() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getY())),
                        -Math.copySign(1., currentPose.getRotation().getDegrees() - targetAngle.getDegrees()),
                        true, true);
            }
                else if (Math.abs(currentPose.getX() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getX()) > 0.1) {
                    swerve.drive(new Translation2d(
                            -Math.copySign(1.,
                                    currentPose.getX() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getX()),
                            0),
                            -Math.copySign(1., currentPose.getRotation().getDegrees() - targetAngle.getDegrees()),
                            true, true);
                }
                else if (Math.abs(currentPose.getY() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getY()) > 0.1) {
                    swerve.drive(new Translation2d(0, -Math.copySign(1.,
                            currentPose.getX() - FieldConstants.RED_SUBWOOFER_FRONT_POSE.getY())),
                            -Math.copySign(1., currentPose.getRotation().getDegrees() - targetAngle.getDegrees()),
                            true, true);
                }
                else if(!rotatingDone) {
                    swerve.drive(new Translation2d(0, 0),
                            -Math.copySign(1., currentPose.getRotation().getDegrees() - targetAngle.getDegrees()),
                            true, true);
                }
            }
        }
    

    @Override
    public boolean isFinished() {
        return rotatingDone && strafingDone;
    }
}
