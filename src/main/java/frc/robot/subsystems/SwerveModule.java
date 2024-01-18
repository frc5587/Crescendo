package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveModuleBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule extends SwerveModuleBase {
    private TalonFX talonFX;
    private CANcoder angleEncoder;
    public SwerveModule(SwerveModuleConstants moduleConstants, TalonFX driveMotor, TalonFX angleMotor, CANcoder angleEncoder) {
        super(moduleConstants, angleMotor, driveMotor);
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.angleEncoder = angleEncoder;
        configureAngleEncoder();
        configureAngleMotor();
        configureDriveMotor();
    }

    @Override
    public Rotation2d getRawAbsoluteEncoderValue() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    protected void setAngleMotorPosition(Rotation2d position) {
        angleMotor.getPIDController().setReference(position.getRotations(), ControlType.kPosition);
    }

    @Override
    protected void setAngleMotorEncoderPosition(Rotation2d position) {
        angleMotor.getEncoder().setPosition(position.getRotations());
    }

    @Override
    protected void setDriveMotorEncoderPosition(Rotation2d position) {
        driveMotor.getEncoder().setPosition(position.getDegrees());
    }

    @Override
    protected void setDriveMotorVelocity(double velocityMPS) {
        driveMotor.getPIDController().setReference(velocityMPS, ControlType.kVelocity);
    }

    @Override
    protected void configureAngleEncoder() {
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(DrivetrainConstants.CANCODER_CONFIG);
    }

    @Override
    protected void configureAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.getPIDController().setP(DrivetrainConstants.ANGLE_FPID.kP);
        angleMotor.getPIDController().setI(DrivetrainConstants.ANGLE_FPID.kI);
        angleMotor.getPIDController().setD(DrivetrainConstants.ANGLE_FPID.kD);
        angleMotor.getPIDController().setFF(DrivetrainConstants.ANGLE_FPID.kF);
        angleMotor.setSmartCurrentLimit(DrivetrainConstants.ANGLE_CONT_LIMIT);
        angleMotor.setSecondaryCurrentLimit(DrivetrainConstants.ANGLE_PEAK_LIMIT);
        // angleMotor.getEncoder().setPositionConversionFactor(360 / DrivetrainConstants.ANGLE_GEAR_RATIO);
        angleMotor.getEncoder().setPositionConversionFactor(-1.);
        angleMotor.setInverted(DrivetrainConstants.ANGLE_MOTOR_INVERTED);
        angleMotor.setIdleMode(IdleMode.kCoast);
        angleMotor.getPIDController().setPositionPIDWrappingEnabled(true);
        angleMotor.getPIDController().setOutputRange(-1, 1, 0);
        angleMotor.getPIDController().setIZone(0, 0);
        angleMotor.burnFlash();
    }

    @Override
    protected void configureDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(DrivetrainConstants.DRIVE_MOTOR_INVERTED);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.getPIDController().setP(DrivetrainConstants.DRIVE_FPID.kP);
        driveMotor.getPIDController().setI(DrivetrainConstants.DRIVE_FPID.kI);
        driveMotor.getPIDController().setD(DrivetrainConstants.DRIVE_FPID.kD);
        driveMotor.getPIDController().setFF(DrivetrainConstants.DRIVE_FPID.kF);
        driveMotor.setOpenLoopRampRate(DrivetrainConstants.OPEN_LOOP_RAMP);
        driveMotor.setClosedLoopRampRate(DrivetrainConstants.CLOSED_LOOP_RAMP);
        driveMotor.setSmartCurrentLimit(DrivetrainConstants.DRIVE_PEAK_LIMIT, DrivetrainConstants.DRIVE_CONT_LIMIT);
        driveMotor.getEncoder().setPositionConversionFactor(DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS / DrivetrainConstants.DRIVE_GEAR_RATIO);
        driveMotor.getEncoder().setVelocityConversionFactor((DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS / DrivetrainConstants.DRIVE_GEAR_RATIO) / 60.);
        driveMotor.getEncoder().setPosition(0);
        driveMotor.burnFlash();
    }

    @Override
    protected Rotation2d getAngleMotorEncoderPosition() {
        return Rotation2d.fromRotations(angleMotor.getEncoder().getPosition());
    }

    @Override
    protected Rotation2d getDriveMotorEncoderPosition() {
        return Rotation2d.fromRotations(driveMotor.getEncoder().getPosition());
    }

    @Override
    protected double getDriveMotorEncoderVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }
}
}
