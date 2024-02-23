package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldConstants;

public class Swerve extends SwerveBase {
     private static SwerveModule[] swerveModules = {
            new SwerveModule(DrivetrainConstants.Mod0.MODULE_CONSTANTS, new TalonFX(10, "canivore"),
                    new TalonFX(15, "canivore"), new CANcoder(50, "canivore"), DrivetrainConstants.Mod0.ANGLE_OFFSET),
            new SwerveModule(DrivetrainConstants.Mod1.MODULE_CONSTANTS, new TalonFX(11, "canivore"),
                    new TalonFX(16, "canivore"), new CANcoder(51, "canivore"), DrivetrainConstants.Mod1.ANGLE_OFFSET),
            new SwerveModule(DrivetrainConstants.Mod2.MODULE_CONSTANTS, new TalonFX(12, "canivore"),
                    new TalonFX(17, "canivore"), new CANcoder(52, "canivore"), DrivetrainConstants.Mod2.ANGLE_OFFSET),
            new SwerveModule(DrivetrainConstants.Mod3.MODULE_CONSTANTS, new TalonFX(13, "canivore"),
                    new TalonFX(18, "canivore"), new CANcoder(53, "canivore"), DrivetrainConstants.Mod3.ANGLE_OFFSET)
    };

   private Limelight limelight;
   private Field2d limelightField = new Field2d();
   private PathPlannerPath ampPath = PathPlannerPath.fromPathFile("ampPath"); // used for alternate ampLineUp command
   private PathPlannerPath subwooferPath = PathPlannerPath.fromPathFile("subwooferPath");

    public Swerve(Limelight limelight) {
        super(DrivetrainConstants.SWERVE_CONSTANTS, swerveModules);
        this.limelight = limelight;
        this.limelightField.setRobotPose(limelight.getLimelightPose());
        ReplanningConfig replanningConfig = new ReplanningConfig(true, true);
        // Auto Config
        
            AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(AutoConstants.TRANSLATION_KP, AutoConstants.TRANSLATION_KI, AutoConstants.TRANSLATION_KD), // Translation PID constants TODO set
                        new PIDConstants(AutoConstants.ROTATION_KP, AutoConstants.ROTATION_KI, AutoConstants.ROTATION_KD), // Rotation PID constants TODO set
                        AutoConstants.MAX_SPEED_MPS, // Max module speed, in m/s
                        AutoConstants.DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                        replanningConfig // Default path replanning config. See the API for the options here
                ),
                () -> {return DriverStation.getAlliance().get().equals(Alliance.Red);},
                this // Reference to this subsystem to set requirements
            );
            SmartDashboard.putBoolean("Swerve Debug On?", false);
    }

    public Command ampLineUp() {
        return AutoBuilder.pathfindToPose(FieldConstants.BLUE_AMP_POSE, AutoConstants.CONSTRAINTS, 0.0,/*m/s*/ 0.0/*meters*/);
        /* alternate ampLineUp command in case first one does not work
        return AutoBuilder.pathfindThenFollowPath(ampPath, AutoConstants.CONSTRAINTS, 0);
        */
    }

    public Command subwooferLineUp() {
        return AutoBuilder.pathfindToPose(FieldConstants.BLUE_SUBWOOFER_FRONT_POSE, AutoConstants.CONSTRAINTS, 0, 0);
        /* alternate subwooferLineUp command in case first one does not work
        return AutoBuilder.pathfindThenFollowPath(subwooferPath, AutoConstants.CONSTRAINTS, 0);
        */
    }
    
    @Override
    public void periodic() {
        super.periodic();
        if(SmartDashboard.getBoolean("Swerve Debug On?", false)) {
            SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw().getDegrees());
            SmartDashboard.putNumber("Yaw Offset", gyro.getYawZeroOffset().getDegrees());
            if(SmartDashboard.getBoolean("Zero Yaw", false)) {
                gyro.zeroYaw();
            }
            SmartDashboard.putBoolean("Zero Yaw", false);
    
            for(int i = 0; i < swerveModules.length; i++) {
                SmartDashboard.putNumber("M"+i+" Raw CANCoder", swerveModules[i].getNonZeroedAbsoluteEncoderValue().getDegrees());
                SmartDashboard.putNumber("M" + i + " Adjusted CANCoder", swerveModules[i].getAbsoluteEncoderValue().getDegrees());
                SmartDashboard.putNumber("M" + i + " Relative", swerveModules[i].getAngle().getDegrees());
            }
        }

        SmartDashboard.putData("Field", field);
        this.limelightField.setRobotPose(limelight.getLimelightPose());

        SmartDashboard.putData("LimelightField", limelightField);
        
        if(limelight.hasTarget() && (limelight.getTargetSpacePose().getX() <= 1. && limelight.getTargetSpacePose().getX() >= -1.)) { // if the target is super close, we can set the pose to the limelight pose
            resetOdometry(limelight.getLimelightPose());
            gyro.setYawZeroOffset(gyro.getUnZeroedYaw().plus(limelight.getLimelightPose().getRotation()));
        }
        if(limelight.hasTarget()) {
            poseEstimator.addVisionMeasurement(getEstimatedPose(), 0);
            poseEstimator.update(getYaw(), getModulePositions());
        }
    }
    /**
     * Sets the module states based on chassis speeds.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        setModuleStates(kinematics.toSwerveModuleStates(speeds), true);
    }

    /**
     * Returns the linear velocity of this swerve in meters per second.
     * @return
     */
    public double getLinearVelocity() {
        return Math.atan2(getChassisSpeeds().vyMetersPerSecond, getChassisSpeeds().vxMetersPerSecond);
    }
}