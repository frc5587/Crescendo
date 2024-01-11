package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    public Limelight limelight;
    public ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();
    public SwerveDriveOdometry odometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveKinematics kinematics;
    public TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5); 
    
    public Field2d field = new Field2d();
    public Double lockedHeading = null;
    // private SlewRateLimiter slew = new SlewRateLimiter(SwerveConstants.SLEW_RATE);

    public Swerve(Limelight limelight) {
        this.gyro = new AHRS();
        this.limelight = limelight;
        this.kinematics = SwerveConstants.SWERVE_KINEMATICS;

        this.mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.MODULECONSTANTS),
            new SwerveModule(1, SwerveConstants.Mod1.MODULECONSTANTS),
            new SwerveModule(2, SwerveConstants.Mod2.MODULECONSTANTS),
            new SwerveModule(3, SwerveConstants.Mod3.MODULECONSTANTS)
        };
        zeroGyro();
        
        Timer.delay(1.0);
        resetModulesToAbsolute();

        this.odometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_KINEMATICS, getYaw(), getModulePositions());
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), getOdometryPose()); // Vision standard deviations.

        SmartDashboard.putData("Swerve Pose Field", field);
        SmartDashboard.putBoolean("Swerve Brake Mode", true);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), //TODO invert getX??????
                        translation.getY(), 
                        rotation, 
                        getYaw())
                    : new ChassisSpeeds(
                        translation.getX(), // -translation.getX(), //TODO invert getX????
                        translation.getY(), 
                        rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        desiredChassisSpeeds = speeds;
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
        // return getEstimatedPose(); // ! TODO If this isnt working, uncomment above
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        gyro.zeroYaw();
        gyro.setAngleAdjustment(pose.getRotation().getDegrees());

        odometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void zeroOdometry() {
        resetOdometry(new Pose2d());
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        //TODO invert states??????
        return states;
    }

    public SwerveModuleState[] getDesiredStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.desiredState;
        }
        //TODO invert states??????
        return states;
    }

    // public SwerveModulePosition[] getDesiredPositions() {
    //     SwerveModulePosition[] positions = new SwerveModulePosition[4];
    //     for(SwerveModule mod : mSwerveMods){
    //         positions[mod.moduleNumber] = mod.desiredState;
    //     }
    //     //TODO invert states??????
    //     return positions;
    // }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        lockedHeading = null;
        gyro.zeroYaw();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.INVERT_GYRO) ? Rotation2d.fromDegrees(-gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getYawFromOdom() {
        return odometry.getPoseMeters().getRotation();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    
    @Override
    public void periodic() {
        // if the target is within 2.5 meters of the robot, factor it into pose data
        // if (limelight.hasTarget() && limelight.calculateDistance() < 2.5) { // TODO: tune distance requirement
        //     poseEstimator.addVisionMeasurement(limelight.getLimelightPose(), Timer.getFPGATimestamp());
        // }

        // if (DriverStation.isDisabled()) {
        //     // TODO Check if robot is outside of community. If so, set motors to coast mode with 0.1 second left
        //     // if(!inCommunity()) {
        //     //     SmartDashboard.putBoolean("Swerve Brake Mode", true);
        //     // }
        //     // resetModulesToAbsolute(); // TODO Maybe remove to fix random wheel positions
        // }

        odometry.update(getYaw(), getModulePositions());
        // odometry.update(getYaw(), kinematics.toSwerveModuleStates(desiredChassisSpeeds));
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions()); // ! If this is wrong, its probably a problem with getYaw()
        poseHistory.addSample(Timer.getFPGATimestamp(), getPose());
        field.setRobotPose(getPose());
        // if(Robot.m_debugMode) {
        //     // DEBUGGING VALUES
        //     for (int i = 0; i < mSwerveMods.length; i++) {
        //         SmartDashboard.putNumber("mod " + i + "degrees", mSwerveMods[i].getCanCoder().getDegrees());
        //         SmartDashboard.putNumber("Adjusted " + i, mSwerveMods[i].getPosition().angle.getDegrees());
        //     }
        // }

        //     SmartDashboard.putNumber("Roll", getRoll());
            // SmartDashboard.putNumber("Pitch", getPitch());
        //     SmartDashboard.putNumber("Yaw", getYaw().getDegrees());
        // }

        // if(SmartDashboard.getBoolean("Swerve Brake Mode", true)) {
        //     for(SwerveModule mod : mSwerveMods) {
        //         mod.mAngleMotor.setNeutralMode(NeutralMode.Brake);
        //         mod.mDriveMotor.setNeutralMode(NeutralMode.Brake);
        //     }
        // } else {
        //     for(SwerveModule mod : mSwerveMods) {
        //         mod.mAngleMotor.setNeutralMode(NeutralMode.Coast);
        //         mod.mDriveMotor.setNeutralMode(NeutralMode.Coast);
        //     }
        // }
    }
}