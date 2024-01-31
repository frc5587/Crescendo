package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class SimSwerve extends SubsystemBase {
    private static final double maxCoastVelocityMetersPerSec = 0.05; // Need to be under this to
                                                                     // switch to coast when disabling

    private final SimGyro gyroSim;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SimSwerveModule[] moduleIOs = new SimSwerveModule[4]; // FL, FR, BL, BR
    private final ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged() };

    private final double maxLinearSpeed = DrivetrainConstants.MAX_SPEED;
    private final double maxAngularSpeed = DrivetrainConstants.MAX_ANGULAR_VELOCITY;
    private final double wheelRadius = DrivetrainConstants.CHOSEN_MODULE.wheelDiameter / 2;
    private final double trackWidthX = DrivetrainConstants.TRACK_WIDTH;
    private final double trackWidthY = DrivetrainConstants.WHEEL_BASE;

    private final double driveKp = DrivetrainConstants.DRIVE_FPID.kP;
    private final double driveKd = DrivetrainConstants.DRIVE_FPID.kD;
    private final double driveKs = DrivetrainConstants.DRIVE_KS;
    private final double driveKv = DrivetrainConstants.DRIVE_KV;

    private final double turnKp = DrivetrainConstants.ANGLE_FPID.kP;
    private final double turnKd = DrivetrainConstants.ANGLE_FPID.kD;

    private final SwerveDriveKinematics kinematics = DrivetrainConstants.SWERVE_KINEMATICS;
    private SimpleMotorFeedforward driveFeedforward;
    private final PIDController[] driveFeedback = new PIDController[4];
    private final PIDController[] turnFeedback = new PIDController[4];

    private Pose2d odometryPose = new Pose2d();
    private Translation2d fieldVelocity = new Translation2d();
    private double[] lastModulePositionsRad = new double[] { 0.0, 0.0, 0.0, 0.0 };
    private double lastGyroPosRad = 0.0;
    private boolean brakeMode = false;

    private DriveMode driveMode = DriveMode.NORMAL;
    private ChassisSpeeds closedLoopSetpoint = new ChassisSpeeds();
    private double characterizationVoltage = 0.0;

    /** Creates a new Drive. */
    public SimSwerve(SimGyro gyroSim, SimSwerveModule flModuleIO, SimSwerveModule frModuleIO,
            SimSwerveModule blModuleIO, SimSwerveModule brModuleIO) {
        this.gyroSim = gyroSim;
        moduleIOs[0] = flModuleIO;
        moduleIOs[1] = frModuleIO;
        moduleIOs[2] = blModuleIO;
        moduleIOs[3] = brModuleIO;

        driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv);
        for (int i = 0; i < 4; i++) {
            driveFeedback[i] = new PIDController(driveKp, 0.0, driveKd,
                    0.02);
            turnFeedback[i] = new PIDController(turnKp, 0.0, turnKd,
                    0.02);
            turnFeedback[i].enableContinuousInput(-Math.PI, Math.PI);
        }
    }

    @Override
    public void periodic() {
        if(SmartDashboard.getBoolean("Zero SimGyro?", false)) {
            gyroInputs.positionRad = 0;
        }
        SmartDashboard.putBoolean("Zero SimGyro?", false);
        gyroSim.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("Drive/Module" + Integer.toString(i),
                    moduleInputs[i]);
        }

        // Update angle measurements
        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            turnPositions[i] = new Rotation2d(moduleInputs[i].turnAbsolutePositionRad);
        }

        if (DriverStation.isDisabled()) {
            // Disable output while disabled
            for (int i = 0; i < 4; i++) {
                moduleIOs[i].setTurnVoltage(0.0);
                moduleIOs[i].setDriveVoltage(0.0);
            }
        } else {
            switch (driveMode) {
                case NORMAL:
                    // In normal mode, run the controllers for turning and driving based on the
                    // current
                    // setpoint
                    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(closedLoopSetpoint);
                    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,
                            maxLinearSpeed);

                    // If stationary, go to last state
                    boolean isStationary = Math.abs(closedLoopSetpoint.vxMetersPerSecond) < 1e-3
                            && Math.abs(closedLoopSetpoint.vyMetersPerSecond) < 1e-3
                            && Math.abs(closedLoopSetpoint.omegaRadiansPerSecond) < 1e-3;

                    SwerveModuleState[] setpointStatesOptimized = new SwerveModuleState[] { null, null, null, null };
                    for (int i = 0; i < 4; i++) {
                        // Run turn controller
                        setpointStatesOptimized[i] = SwerveModuleState.optimize(setpointStates[i], turnPositions[i]);
                        if (isStationary) {
                            moduleIOs[i].setTurnVoltage(0.0);
                        } else {
                            moduleIOs[i].setTurnVoltage(
                                    turnFeedback[i].calculate(turnPositions[i].getRadians(),
                                            setpointStatesOptimized[i].angle.getRadians()));
                        }

                        // Update velocity based on turn error
                        setpointStatesOptimized[i].speedMetersPerSecond *= Math.cos(turnFeedback[i].getPositionError());

                        // Run drive controller
                        double velocityRadPerSec = setpointStatesOptimized[i].speedMetersPerSecond / wheelRadius;
                        moduleIOs[i].setDriveVoltage(
                                driveFeedforward.calculate(velocityRadPerSec) + driveFeedback[i]
                                        .calculate(moduleInputs[i].driveVelocityRadPerSec,
                                                velocityRadPerSec));

                        // Log individual setpoints
                        Logger.recordOutput(
                                "SwerveSetpointValues/Drive/" + Integer.toString(i),
                                velocityRadPerSec);
                        Logger.recordOutput(
                                "SwerveSetpointValues/Turn/" + Integer.toString(i),
                                setpointStatesOptimized[i].angle.getRadians());
                    }

                    // Log all module setpoints
                    Logger.recordOutput("SwerveModuleStates/Setpoints",
                            setpointStates);
                    Logger.recordOutput(
                            "SwerveModuleStates/SetpointsOptimized", setpointStatesOptimized);
                    break;

                case CHARACTERIZATION:
                    // In characterization mode, drive at the specified voltage (and turn to zero
                    // degrees)
                    for (int i = 0; i < 4; i++) {
                        moduleIOs[i].setTurnVoltage(
                                turnFeedback[i].calculate(turnPositions[i].getRadians(), 0.0));
                        moduleIOs[i].setDriveVoltage(characterizationVoltage);
                    }
                    break;

                case X:
                    for (int i = 0; i < 4; i++) {
                        Rotation2d targetRotation = new Rotation2d(getModuleTranslations()[i].getX(), getModuleTranslations()[i].getY());
                        Rotation2d currentRotation = turnPositions[i];
                        if (Math.abs(
                                targetRotation.minus(currentRotation).getDegrees()) > 90.0) {
                            targetRotation = targetRotation.minus(Rotation2d.fromDegrees(180.0));
                        }
                        moduleIOs[i].setTurnVoltage(turnFeedback[i].calculate(
                                currentRotation.getRadians(), targetRotation.getRadians()));
                        moduleIOs[i].setDriveVoltage(0.0);
                    }
                    break;
            }
        }

        // Update odometry
        SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStatesDiff[i] = new SwerveModuleState(
                    (moduleInputs[i].drivePositionRad - lastModulePositionsRad[i])
                            * wheelRadius,
                    turnPositions[i]);
            lastModulePositionsRad[i] = moduleInputs[i].drivePositionRad;
        }
        ChassisSpeeds chassisStateDiff = kinematics.toChassisSpeeds(measuredStatesDiff);
        if (gyroInputs.connected) { // Use gyro for angular change when connected
            odometryPose = odometryPose.exp(new Twist2d(chassisStateDiff.vxMetersPerSecond,
                    chassisStateDiff.vyMetersPerSecond,
                    gyroInputs.positionRad - lastGyroPosRad));
        } else { // Fall back to using angular velocity (disconnected or sim)
            odometryPose = odometryPose.exp(new Twist2d(chassisStateDiff.vxMetersPerSecond,
                    chassisStateDiff.vyMetersPerSecond,
                    chassisStateDiff.omegaRadiansPerSecond));
        }
        lastGyroPosRad = gyroInputs.positionRad;

        // Update field velocity
        SwerveModuleState[] measuredStates = new SwerveModuleState[] { null, null, null, null };
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = new SwerveModuleState(
                    moduleInputs[i].driveVelocityRadPerSec * wheelRadius,
                    turnPositions[i]);
        }
        ChassisSpeeds chassisState = kinematics.toChassisSpeeds(measuredStates);
        fieldVelocity = new Translation2d(chassisState.vxMetersPerSecond,
                chassisState.vyMetersPerSecond).rotateBy(getRotation());

        // Log measured states
        Logger.recordOutput("SwerveModuleStates/Measured",
                measuredStates);

        // Log odometry pose
        Logger.recordOutput("Odometry/Robot", odometryPose);

        // Enable/disable brake mode
        if (DriverStation.isEnabled()) {
        }
        else {
            boolean stillMoving = false;
            for (int i = 0; i < 4; i++) {
                if (Math.abs(moduleInputs[i].driveVelocityRadPerSec
                        * wheelRadius) > maxCoastVelocityMetersPerSec) {
                    stillMoving = true;
                }
            }
        }
    }

    /**
     * Runs the drive at the desired velocity.
     * 
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        driveMode = DriveMode.NORMAL;
        closedLoopSetpoint = speeds;
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void goToX() {
        driveMode = DriveMode.X;
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxLinearSpeed;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return maxAngularSpeed;
    }

    /** Returns the current odometry pose. */
    public Pose2d getPose() {
        return odometryPose;
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return odometryPose.getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        odometryPose = pose;
    }

    public Translation2d getFieldVelocity() {
        return fieldVelocity;
    }

    /** Returns an array of module translations. */
    public Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
                new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
                new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
                new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0) };
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(double volts) {
        driveMode = DriveMode.CHARACTERIZATION;
        characterizationVoltage = volts;
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (int i = 0; i < 4; i++) {
            driveVelocityAverage += moduleInputs[i].driveVelocityRadPerSec;
        }
        return driveVelocityAverage / 4.0;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.closedLoopSetpoint = fieldRelative ?
        ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getRotation()) 
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    private static enum DriveMode {
        NORMAL, X, CHARACTERIZATION
    }
}