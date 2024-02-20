package frc.robot.commands;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve;

public class AutoRotateToShoot extends Command {
    private final Swerve swerve;
    private Rotation2d targetAngle = new Rotation2d();
    private Field2d targetField = new Field2d();

    public AutoRotateToShoot(Swerve swerve) {
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
            targetAngle = swerve.getEstimatedPose().relativeTo(FieldConstants.BLUE_SUBWOOFER_FRONT_POSE).getTranslation().getAngle().times(-1);
            if(swerve.getYaw().getDegrees() > targetAngle.getDegrees()) {
                swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 1));
            }
            else {
                swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, -1));
            }
            System.out.println("TARGET " + targetAngle);
            System.out.println("ANGLE " + swerve.getYaw());
        }
        targetField.setRobotPose(swerve.getEstimatedPose().getX(), swerve.getEstimatedPose().getY(), targetAngle);
        SmartDashboard.putData("Target", targetField);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getYaw().getDegrees() - targetAngle.getDegrees()) < 2;
    }
}
