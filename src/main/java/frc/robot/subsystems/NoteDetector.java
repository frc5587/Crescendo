package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NoteDetector extends LimelightBase {
    public NoteDetector() {
        super(-10., 0.2, 0., 0);
        this.limelightTable = NetworkTableInstance.getDefault().getTable("noteDetector");
        this.ty = limelightTable.getEntry("ty");
        this.tx = limelightTable.getEntry("tx");
        this.tv = limelightTable.getEntry("tv");
        this.tl = limelightTable.getEntry("tl");
        this.ledMode = limelightTable.getEntry("ledMode");

        PortForwarder.add(5800, "noteDetect.local", 5800);
        PortForwarder.add(5801, "noteDetect.local", 5801);
        PortForwarder.add(5802, "noteDetect.local", 5802);
        PortForwarder.add(5803, "noteDetect.local", 5803);
        PortForwarder.add(5804, "noteDetect.local", 5804);
        PortForwarder.add(5805, "noteDetect.local", 5805);
    }

    public Rotation2d getRotationToNote() {
        return Rotation2d.fromDegrees(tx.getDouble(0.0));
    }

    public double getDistanceToNoteMeters() {
        return ty.getDouble(0.0);
    }
}
