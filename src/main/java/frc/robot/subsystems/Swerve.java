package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private SwerveModule[] swerveModules;
    private Limelight limelight;
    private Field2d limelightField = new Field2d();
    private boolean brakeModeEnabled = true;

    public Swerve(SwerveModule[] swerveModules, Limelight limelight) {
        super(DrivetrainConstants.SWERVE_CONSTANTS, swerveModules);
        this.swerveModules = swerveModules;
        this.limelight = limelight;
        this.limelightField.setRobotPose(limelight.getWPIBlueBotpose());
        ReplanningConfig replanningConfig = new ReplanningConfig(true, true);
        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            getYaw(), 
            getModulePositions(), 
            DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? FieldConstants.BLUE_SUBWOOFER_FRONT_POSE : FieldConstants.RED_SUBWOOFER_FRONT_POSE,
            MatBuilder.fill(Nat.N3(), Nat.N1(), 0.05, 0.05, 0.05), // Odometry standard deviations. Smaller number = more trust. PoseX, PoseY, Rotation
            MatBuilder.fill(Nat.N3(), Nat.N1(), .7, .7, 999.) // Vision standard deviations.
            );
        // Auto Config
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometryWithYaw, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(AutoConstants.TRANSLATION_KP, AutoConstants.TRANSLATION_KI, AutoConstants.TRANSLATION_KD), // Translation PID constants
                new PIDConstants(AutoConstants.ROTATION_KP, AutoConstants.ROTATION_KI, AutoConstants.ROTATION_KD), // Rotation PID constants
                AutoConstants.MAX_SPEED_MPS, // Max module speed, in m/s
                AutoConstants.DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                replanningConfig // Default path replanning config. See the API for the options here
            ),
            () -> {return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Red);},
            this // Reference to this subsystem to set requirements
            );
        SmartDashboard.putBoolean("Swerve Debug On?", false);
        SmartDashboard.putBoolean("Swerve Brake Mode", brakeModeEnabled);
        SmartDashboard.putBoolean("Reset to Limelight Pose", false);
        resetOdometry((DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? FieldConstants.BLUE_SUBWOOFER_FRONT_POSE : FieldConstants.RED_SUBWOOFER_FRONT_POSE));
    }

    public Swerve(Limelight limelight) {
        this(new SwerveModule[]{
            new SwerveModule(DrivetrainConstants.Mod0.MODULE_CONSTANTS, new TalonFX(10, "canivore"),
                    new TalonFX(15, "canivore"), new CANcoder(50, "canivore"), DrivetrainConstants.Mod0.ANGLE_OFFSET),
            new SwerveModule(DrivetrainConstants.Mod1.MODULE_CONSTANTS, new TalonFX(11, "canivore"),
                    new TalonFX(16, "canivore"), new CANcoder(51, "canivore"), DrivetrainConstants.Mod1.ANGLE_OFFSET),
            new SwerveModule(DrivetrainConstants.Mod2.MODULE_CONSTANTS, new TalonFX(12, "canivore"),
                    new TalonFX(17, "canivore"), new CANcoder(52, "canivore"), DrivetrainConstants.Mod2.ANGLE_OFFSET),
            new SwerveModule(DrivetrainConstants.Mod3.MODULE_CONSTANTS, new TalonFX(13, "canivore"),
                    new TalonFX(18, "canivore"), new CANcoder(53, "canivore"), DrivetrainConstants.Mod3.ANGLE_OFFSET)
        }, limelight);
    }

    public Command ampLineUp() {
        return AutoBuilder.pathfindToPose(DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? FieldConstants.BLUE_AMP_POSE : FieldConstants.RED_AMP_POSE, AutoConstants.PATHFIND_CONSTRAINTS, 0.0,/*m/s*/ 0.0/*meters*/);

    }

    public Command subwooferLineUp() {
        return AutoBuilder.pathfindToPose(DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? FieldConstants.BLUE_SUBWOOFER_FRONT_POSE : FieldConstants.RED_SUBWOOFER_FRONT_POSE, AutoConstants.PATHFIND_CONSTRAINTS, 0, 0);
    }

    public void resetOdometryWithYaw(Pose2d pose) {
        resetOdometry(pose);
        gyro.setYaw(pose.getRotation());
    }

    @Override
    public void periodic() {
        super.periodic();
                this.limelightField.setRobotPose(limelight.getWPIBlueBotpose());

        if(SmartDashboard.getBoolean("Swerve Debug On?", false)) {
            SmartDashboard.putNumber("Yaw Offset", gyro.getYawZeroOffset().getDegrees()); 
            for(int i = 0; i < swerveModules.length; i++) {
                SmartDashboard.putNumber("M" + i + " Adjusted CANCoder", swerveModules[i].getAbsoluteEncoderValue().getDegrees());
                SmartDashboard.putNumber("M" + i + " Relative", swerveModules[i].getAngle().getDegrees());
            }
        }
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw().getDegrees());
        
        if(SmartDashboard.getBoolean("Zero Yaw", false)) {
            gyro.zeroYaw();
        }
        SmartDashboard.putBoolean("Zero Yaw", false);

        if(SmartDashboard.getBoolean("Swerve Brake Mode", true) != brakeModeEnabled) {
            this.brakeModeEnabled = SmartDashboard.getBoolean("Swerve Break Mode", true);
            for(SwerveModule mod : swerveModules) {
                mod.setBrakeMode(brakeModeEnabled);
            }
        }

        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("LimelightField", limelightField);
        
        if(limelight.hasTarget() && (limelight.get3DTargetSpacePose().getZ() <= 1.5) && SmartDashboard.getBoolean("Reset to Limelight Pose", false)) {// if the target is super close, we can set the pose to the limelight pose
            odometry.resetPosition(getYaw(), getModulePositions(), limelight.getWPIBlueBotpose());
            poseEstimator.resetPosition(getYaw(), getModulePositions(), limelight.getWPIBlueBotpose());
            SmartDashboard.putBoolean("Reset to Limelight Pose", false);
        }

        if(limelight.hasTarget() && (limelight.get3DTargetSpacePose().getZ() <= 2.5) && !DriverStation.isAutonomousEnabled()) {
            poseEstimator.addVisionMeasurement(limelight.getWPIBlueBotpose(), limelight.calculateFPGAFrameTimestamp());
        }
    }
    /**
     * Sets the module states based on chassis speeds.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
        setModuleStates(kinematics.toSwerveModuleStates(speeds), false);
    }
    
    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
        speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        setModuleStates(kinematics.toSwerveModuleStates(speeds), isOpenLoop);
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
    }

    /**
     * Returns the linear velocity of this swerve in meters per second.
     * @return
     */
    public double getLinearVelocity() {
        return Math.atan2(getChassisSpeeds().vyMetersPerSecond, getChassisSpeeds().vxMetersPerSecond);
    }

    public void standYourGround() {
        for(int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setAngle(i == 2 || i == 1 ? Rotation2d.fromDegrees(135) : Rotation2d.fromDegrees(45));
        }
    }
}