package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
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
     public SwerveModule[] swerveModules;
     private Limelight limelight;
   private Field2d limelightField = new Field2d();
   private PathPlannerPath ampPath = PathPlannerPath.fromPathFile("ampPath"); // used for alternate ampLineUp command
   private PathPlannerPath subwooferPath = PathPlannerPath.fromPathFile("subwooferPath");
    private boolean brakeModeEnabled = true;
    private final TimeInterpolatableBuffer<Double> velocityBuffer = TimeInterpolatableBuffer.createDoubleBuffer(1.);

    public Swerve(Limelight limelight) {
        this(new SwerveModule[] {
                new SwerveModule(DrivetrainConstants.Mod0.MODULE_CONSTANTS, new TalonFX(10, "canivore"),
                        new TalonFX(15, "canivore"), new CANcoder(50, "canivore"),
                        DrivetrainConstants.Mod0.ANGLE_OFFSET),
                new SwerveModule(DrivetrainConstants.Mod1.MODULE_CONSTANTS, new TalonFX(11, "canivore"),
                        new TalonFX(16, "canivore"), new CANcoder(51, "canivore"),
                        DrivetrainConstants.Mod1.ANGLE_OFFSET),
                new SwerveModule(DrivetrainConstants.Mod2.MODULE_CONSTANTS, new TalonFX(12, "canivore"),
                        new TalonFX(17, "canivore"), new CANcoder(52, "canivore"),
                        DrivetrainConstants.Mod2.ANGLE_OFFSET),
                new SwerveModule(DrivetrainConstants.Mod3.MODULE_CONSTANTS, new TalonFX(13, "canivore"),
                        new TalonFX(18, "canivore"), new CANcoder(53, "canivore"),
                        DrivetrainConstants.Mod3.ANGLE_OFFSET)
        }, limelight);
    }
   

    public Swerve(SwerveModule[] swerveModules, Limelight limelight) {
        super(DrivetrainConstants.SWERVE_CONSTANTS, swerveModules);
        this.limelight = limelight;
        this.swerveModules = swerveModules;
        this.limelightField.setRobotPose(limelight.getLimelightPose());
        ReplanningConfig replanningConfig = new ReplanningConfig(true, true);
        // Auto Config
        
            AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometryWithYaw, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(AutoConstants.TRANSLATION_KP, AutoConstants.TRANSLATION_KI, AutoConstants.TRANSLATION_KD), // Translation PID constants TODO set
                        new PIDConstants(AutoConstants.ROTATION_KP, AutoConstants.ROTATION_KI, AutoConstants.ROTATION_KD), // Rotation PID constants TODO set
                        AutoConstants.MAX_SPEED_MPS, // Max module speed, in m/s
                        AutoConstants.DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                        replanningConfig // Default path replanning config. See the API for the options here
                ),
                () -> {return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Red);},
                this // Reference to this subsystem to set requirements
            );
            SmartDashboard.putBoolean("Swerve Debug On?", false);
            SmartDashboard.putBoolean("Swerve Brake Mode", brakeModeEnabled);
    }

    public Command ampLineUp() {
        return AutoBuilder.pathfindToPose(DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? FieldConstants.BLUE_AMP_POSE : FieldConstants.RED_AMP_POSE, AutoConstants.CONSTRAINTS, 0.0,/*m/s*/ 0.0/*meters*/);
        /* alternate ampLineUp command in case first one does not work
        return AutoBuilder.pathfindThenFollowPath(ampPath, AutoConstants.CONSTRAINTS, 0);
        */
    }

    public Command subwooferLineUp() {
        return AutoBuilder.pathfindToPose(DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? FieldConstants.BLUE_SUBWOOFER_FRONT_POSE : FieldConstants.RED_SUBWOOFER_FRONT_POSE, AutoConstants.CONSTRAINTS, 0, 0);
        // alternate subwooferLineUp command in case first one does not work
        // return AutoBuilder.pathfindThenFollowPath(subwooferPath, AutoConstants.CONSTRAINTS, 0);
        //
    }
    
    @Override
    public void periodic() {
        super.periodic();
        if(SmartDashboard.getBoolean("Swerve Debug On?", false)) {
            SmartDashboard.putNumber("Yaw Offset", gyro.getYawZeroOffset().getDegrees());
            
    
            for(int i = 0; i < swerveModules.length; i++) {
                SmartDashboard.putNumber("M"+i+" Raw CANCoder", swerveModules[i].getNonZeroedAbsoluteEncoderValue().getDegrees());
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
        this.limelightField.setRobotPose(limelight.getLimelightPose());

        SmartDashboard.putData("LimelightField", limelightField);
        
        if(limelight.hasTarget() && (limelight.getTargetSpacePose().getZ() <= 2.) && !DriverStation.isAutonomousEnabled()) {// && limelight.getTargetSpacePose().getZ() >= -1.)) { // if the target is super close, we can set the pose to the limelight pose
            // resetOdometry(limelight.getLimelightPose());
            odometry.resetPosition(getYaw(), getModulePositions(), limelight.getLimelightPose());
            poseEstimator.resetPosition(getYaw(), getModulePositions(), limelight.getLimelightPose());
            // gyro.setYaw(limelight.getLimelightPose().getRotation().plus(DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? new Rotation2d() : Rotation2d.fromDegrees(180.)));
            SmartDashboard.putBoolean("Within Range", true);
        }
        else {
            SmartDashboard.putBoolean("Within Range", false);
        }
        if(limelight.hasTarget() && (limelight.getTargetSpacePose().getZ() <= 2.5) && !DriverStation.isAutonomousEnabled()) {
            poseEstimator.addVisionMeasurement(limelight.getWPIBlueBotpose(), limelight.calculateFPGAFrameTimestamp());
            poseEstimator.updateWithTime(limelight.calculateFPGAFrameTimestamp(), getYaw(), getModulePositions());
        }
        
        velocityBuffer.addSample(MathSharedStore.getTimestamp(), getLinearVelocity());
    }
    /**
     * Sets the module states based on chassis speeds.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
        setModuleStates(kinematics.toSwerveModuleStates(speeds), true);
    }

    /**
     * Returns the linear velocity of this swerve in meters per second.
     * @return
     */
    public double getLinearVelocity() {
        return Math.atan2(getChassisSpeeds().vyMetersPerSecond, getChassisSpeeds().vxMetersPerSecond);
    }

    public double getLinearAcceleration() {
        //(v - u) / t = a
        // doing samples of 3 periods to reduce noise in period length
        return (velocityBuffer.getSample(MathSharedStore.getTimestamp()).orElseGet(() -> {return 0.;}) - velocityBuffer.getSample(MathSharedStore.getTimestamp() - 0.06).orElseGet(() -> {return 0.;})) / 0.06;
    }

    public void setVoltage(double voltage) {
        for(SwerveModule module : swerveModules) {
            module.setDriveMotorVoltage(-voltage);
            module.setAngle(new Rotation2d());
        }
    }

    public void resetOdometryWithYaw(Pose2d pose) {
        resetOdometry(pose);
        gyro.setYaw(pose.getRotation());
    }
}