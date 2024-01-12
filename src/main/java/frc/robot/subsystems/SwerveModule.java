package frc.robot.subsystems;



import org.frc5587.lib.control.TitanDrive.ControlMode;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.swervelib.math.Conversions;
import frc.robot.util.swervelib.util.CTREConfigs;
import frc.robot.util.swervelib.util.CTREModuleState;
import frc.robot.util.swervelib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    public Rotation2d angleOffset;
    private Rotation2d lastAngle;
    public static CTREConfigs ctreConfigs;
    public SwerveModuleState desiredState = new SwerveModuleState();

    public TalonFX mAngleMotor;
    public TalonFX mDriveMotor;

    private final FlywheelSim sAngleMotor = new FlywheelSim(
            // LinearSystemId.identifyVelocitySystem(kvVoltSecondsPerRadian,
            // kaVoltSecondsSquaredPerRadian),
            // LinearSystemId.identifyVelocitySystem(1.47, 0.0348),
            LinearSystemId.identifyVelocitySystem(0.16, 0.0917),
            DCMotor.getFalcon500(1),
            SwerveConstants.ANGLE_GEAR_RATIO);

    private final FlywheelSim sDriveMotor = new FlywheelSim(
            // LinearSystemId.identifyVelocitySystem(Constants.DriveConstants.kvVoltSecondsPerMeter,
            // Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            LinearSystemId.identifyVelocitySystem(2, 1.24),
            DCMotor.getFalcon500(1),
            SwerveConstants.DRIVE_GEAR_RATIO);

    public CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = SwerveConstants.DRIVE_FF;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        ctreConfigs = new CTREConfigs();

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "canivore");
        // angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "canivore");
        // mAngleMotor = new TalonFX(moduleConstants.angleMotorID);

        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "canivore");
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        this.desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(this.desiredState);
        setSpeed(this.desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
            // TODOD set ControlModeValue.VoltageOut
            mDriveMotor.set(percentOutput * 12);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    SwerveConstants.WHEEL_CIRCUMFERENCE_METERS, SwerveConstants.DRIVE_GEAR_RATIO);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_SPEED * 0.05))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 5%. Prevents Jittering.

        mAngleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.ANGLE_GEAR_RATIO));
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveConstants.ANGLE_GEAR_RATIO));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(),
                SwerveConstants.ANGLE_GEAR_RATIO);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(SwerveConstants.ANGLE_MOTOR_INVERTED);
        mAngleMotor.setNeutralMode(SwerveConstants.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(SwerveConstants.DRIVE_MOTOR_INVERTED);
        mDriveMotor.setNeutralMode(SwerveConstants.DRIVE_NEUTRAL_MODE);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                -Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
                        SwerveConstants.WHEEL_CIRCUMFERENCE_METERS, SwerveConstants.DRIVE_GEAR_RATIO), // TODO Remove -
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                -Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(),
                        SwerveConstants.WHEEL_CIRCUMFERENCE_METERS, SwerveConstants.DRIVE_GEAR_RATIO), // TODO Remove -
                getAngle());
    }
}