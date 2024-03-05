package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends LimelightBase {
    SendableChooser<LedValues> ledChooser = new SendableChooser<LedValues>();

    public Limelight() {
        super(0, Units.inchesToMeters(30), 0, 0);

        ledChooser.setDefaultOption("DEFAULT", LedValues.DEFAULT);
        ledChooser.addOption("ON", LedValues.ON);
        ledChooser.addOption("OFF", LedValues.OFF);
        ledChooser.addOption("BLINK", LedValues.BLINK);
        SmartDashboard.putData("Limelight LEDs", ledChooser);

    }
    
    public Pose2d getLimelightPose() {
        double[] limelightBotPose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pose2d(limelightBotPose[0], limelightBotPose[1], Rotation2d.fromDegrees(limelightBotPose[5]));
    }

    public Pose3d getTargetSpacePose() {
        double[] limelightBotPose = limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        return new Pose3d(limelightBotPose[0], limelightBotPose[1], Math.abs(limelightBotPose[2]), new Rotation3d(0, 0, Rotation2d.fromDegrees(limelightBotPose[4]).getRadians()));
    }

    public void periodic() {
        setLEDs(ledChooser.getSelected());
    }

}
