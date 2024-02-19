package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve;

public class LineUpToAmp extends Command {
    private final Swerve swerve;
    private boolean rotatingDone, strafingDone = false;

    public LineUpToAmp(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        if(!rotatingDone) {
            // code to rotatate
            if(swerve.getPose().getRotation().equals(Rotation2d.fromDegrees(-90))) {
                rotatingDone = true;
            }
        }
        else {
            if(Math.abs(swerve.getPose().getX() - FieldConstants.BLUE_AMP_POSE.getX()) < 0.25) {
                strafingDone = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (rotatingDone && strafingDone) {
        return true;
        }
        else {
        return false; //robot lined up
        }
    }
}
