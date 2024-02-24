package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

public class Photon extends PhotonCamera {
    protected NetworkTable photonTable = NetworkTableInstance.getDefault().getTable("limelight");
    protected NetworkTableEntry ty = photonTable.getEntry("ty");
    protected NetworkTableEntry tx = photonTable.getEntry("tx");
    protected NetworkTableEntry tv = photonTable.getEntry("tv");
    protected NetworkTableEntry tl = photonTable.getEntry("tl");
    protected NetworkTableEntry ledMode = photonTable.getEntry("ledMode");

    protected double mountAngle = 0;
    protected double lensHeight = 0;
    protected double goalHeight = 0;
    protected double distanceOffset = 0;

    protected final double approximateNetworkDelaySeconds = 0.05;

    public Photon() {
        super("limelight");
        // Allow access to limelight when connected over USB
        PortForwarder.add(5800, "photonvision.local", 5800);
        PortForwarder.add(5801, "photonvision.local", 5801);
        PortForwarder.add(5802, "photonvision.local", 5802);
        PortForwarder.add(5803, "photonvision.local", 5803);
        PortForwarder.add(5804, "photonvision.local", 5804);
        PortForwarder.add(5805, "photonvision.local", 5805);
    }

    /**
     * Calculate the angle to the goal in degrees
     * 
     * @return The angle to the goal in degrees
     */
    public double angleToGoalDegrees() {
        return ty.getDouble(0.0) + mountAngle;
    }

    /**
     * Calculate the angle to the goal in radians using {@link #angleToGoalDegrees}
     * 
     * @return The angle to the goal in radians
     */
    public double angleToGoalRadians() {
        return Units.degreesToRadians(angleToGoalDegrees());
    }

    /**
     * Get the horizontal angle
     * 
     * @return
     */
    public double getHorizontalAngleRadians() {
        return Math.toRadians(tx.getDouble(0.0)) / Math.cos(Math.toRadians(ty.getDouble(0.0)) + Math.toRadians(mountAngle)); // should fix perspective shifts that occur on the sides
    }

    /**
     * Calculate the distance from the Limelight to the goal
     * 
     * @return The distance from the Limelight to goal in meters
     */
    public double calculateDistance() {
        return (goalHeight - lensHeight) / (Math.tan(angleToGoalRadians()) * Math.cos(Math.toRadians(tx.getDouble(0.0)))) + distanceOffset;
    }

    /**
     * Gets the MegaTag botpose relative to the blue driver station where (0, 0) is
     * at the bottom-left corner of a blue-left field map.
     * @return a Pose2d representing the robot's pose relative to the blue driver station
     */
    public Pose2d getWPIBlueBotpose() {
        double[] limelightBotPose = photonTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pose2d(limelightBotPose[0], limelightBotPose[1], Rotation2d.fromDegrees(limelightBotPose[5]));
    }
    
    /**
     * Gets the MegaTag botpose relative to the red driver station where (0, 0) is
     * at the top-right corner of a blue-left field map.
     * @return a Pose2d representing the robot's pose relative to the red driver station
     */
    public Pose2d getWPIRedBotpose() {
        double[] limelightBotPose = photonTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        return new Pose2d(limelightBotPose[0], limelightBotPose[1], Rotation2d.fromDegrees(limelightBotPose[5]));
    }

    /**
     * Gets the MegaTag botpose relative to the field where (0, 0) is at the center
     * of a field map.
     * @return a Pose2d representing the robot's pose relative to the red driver station
     */
    public Pose2d getFieldSpaceBotpose() {
        // double[] limelightBotPose = photonTable.getEntry("botpose").getDoubleArray(new double[6]);
        // return new Pose2d(limelightBotPose[0], limelightBotPose[1], Rotation2d.fromDegrees(limelightBotPose[5]));
        PhotonPipelineResult result = getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        // AprilTagFieldLayout layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getAbsolutePath() + "limelightFiles/FieldLayout.fmap");
        // PhotonPoseEstimator
        // Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), layout.getTagPose(target.getFiducialId()), cameraToRobot);
        return null;
    }

    /**
     * Gets the MegaTag botpose relative to the target where (0, 0) is at the center
     * of the target.
     * @return a Pose2d representing the robot's pose relative to the red driver station
     */
    public Pose2d getTargetSpaceBotpose() {
        double[] limelightBotPose = photonTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        return new Pose2d(limelightBotPose[0], limelightBotPose[1], Rotation2d.fromDegrees(limelightBotPose[5]));
    }

    /**
     * @return A value Whether the Limelight detects a target
     */
    public boolean hasTarget() {
        return tv.getNumber(0).intValue() == 1;
    }

    /**
     * 
     */
    public enum LedValues {
        DEFAULT,
        OFF,
        BLINK,
        ON
    }

    /**
     * Set the Limelight LEDs to a specific mode
     * 
     * @param value - DEFAULT, OFF, BLINK, ON
     */
    public void setLEDs(LedValues value) {
        switch (value) {
            case DEFAULT:
                ledMode.setNumber(0);
                break;
            case OFF:
                ledMode.setNumber(1);
                break;
            case BLINK:
                ledMode.setNumber(2);
                break;
            case ON:
                ledMode.setNumber(3);
                break;
            default:
                throw new RuntimeException("Invalid value (DEFAULT, OFF, BLINK, ON)");
        }
    }

    /**
     * Turn on Limelight LEDs
     */
    public void on() {
        setLEDs(LedValues.ON);
    }

    /**
     * Turn off Limelight LEDs
     */
    public void off() {
        setLEDs(LedValues.OFF);
    }

    /**
     * Check if the Limelight LEDs are on
     * 
     * @return A truthy value if the Limelight LEDs are on
     */
    public boolean isOn() {
        return ledMode.getNumber(0.0).doubleValue() >= 1;
    }

    /**
     * Gets the amount of the time the limelight took to calculate the frame, will
     * be at least 11 ms
     * 
     * @return latency in milliseconds
     */
    public double pipelineLatencyMS() {
        return tl.getDouble(0);
    }

    /**
     * Gets the (approximate) timestamp of when the frame was taken. This allows you
     * to predict where the robot was at this point in time, and minimize the
     * effects of latency from the network and from the limelight.
     * 
     * @return FPGA timestamp (seconds)
     */
    public double calculateFPGAFrameTimestamp() {
        return Timer.getFPGATimestamp() - totalSystemLatency();
    }

    /**
     * Gets the total delay between when the frame was taken, and when the data
     * arrived to the RIO.
     * 
     * @return delay in seconds
     */
    public double totalSystemLatency() {
        return (Units.millisecondsToSeconds(pipelineLatencyMS()) - approximateNetworkDelaySeconds);
    }
}
