package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;

import frc.robot.Constants.LimelightConstants;

public class Limelight extends LimelightBase {
    public Limelight() {
        super(LimelightConstants.MOUNT_ANGLE, LimelightConstants.LENS_HEIGHT, LimelightConstants.GOAL_HEIGHT, LimelightConstants.DISTANCE_OFFSET);
    }
}
