package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;

public class NoteDetector extends LimelightBase {
    public NoteDetector() {
        super(LimelightConstants.NOTE_DETECTOR_MOUNT_ANGLE.getDegrees(), LimelightConstants.INITIAL_LENS_HEIGHT, LimelightConstants.GOAL_HEIGHT, LimelightConstants.DISTANCE_OFFSET);
        this.limelightTable = NetworkTableInstance.getDefault().getTable("limelight-notes");
        this.ty = limelightTable.getEntry("ty");
        this.tx = limelightTable.getEntry("tx");
        this.tv = limelightTable.getEntry("tv");
        this.tl = limelightTable.getEntry("tl");
        this.ledMode = limelightTable.getEntry("ledMode");

        PortForwarder.add(5800, "limelight-note.local", 5800);
        PortForwarder.add(5801, "limelight-note.local", 5801);
        PortForwarder.add(5802, "limelight-note.local", 5802);
        PortForwarder.add(5803, "limelight-note.local", 5803);
        PortForwarder.add(5804, "limelight-note.local", 5804);
        PortForwarder.add(5805, "limelight-note.local", 5805);
    }

    public Rotation2d getRotationToNote() {
        return Rotation2d.fromDegrees(tx.getDouble(0.0));
    }

    public double getDistanceToNoteMeters() {
        return ty.getDouble(0.0) + mountAngle;
    }

    public double getDistanceToNoteMeters(Rotation2d armAngle) {
        return ty.getDouble(0.0) + mountAngle + armAngle.getDegrees();
    }

    public Rotation2d getRotationToNote(Rotation2d armAngle) {
        return Rotation2d.fromRadians(
                Math.toRadians(tx.getDouble(0.0))
                / Math.cos(Math.toRadians(ty.getDouble(0.0)) + LimelightConstants.NOTE_DETECTOR_MOUNT_ANGLE.plus(armAngle).getRadians()));
    }

    public double getLensHeight(Rotation2d armAngle) {
        return (LimelightConstants.DISTANCE_TO_ARM_PIVOT * Math.sin(armAngle.getRadians())) + LimelightConstants.INITIAL_LENS_HEIGHT;
    }

    @Override
    public void periodic() {
        super.periodic();
        if(hasTarget()) {
            SmartDashboard.putNumber("Calc Height", getLensHeight(new Rotation2d()));
            SmartDashboard.putNumber("Calc Distance", getDistanceToNoteMeters(new Rotation2d()));
            SmartDashboard.putNumber("Calc Angle", getRotationToNote(new Rotation2d()).getDegrees());
        }
    }
}
