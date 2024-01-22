package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveBase.SwerveConstants;
import org.frc5587.lib.subsystems.SwerveModuleBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.util.swervelib.util.CTREConfigs;

public class SwerveModule extends SwerveModuleBase {
    private TalonFX angleMotor, driveMotor;
    private CANcoder angleEncoder;
    public static CTREConfigs ctreConfigs;
    public SwerveModule(SwerveModuleConstants moduleConstants, TalonFX driveMotor, TalonFX angleMotor, CANcoder angleEncoder) {
        super(moduleConstants, angleMotor, driveMotor);
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.angleEncoder = angleEncoder;
        configureAngleEncoder();
        configureAngleMotor();
        configureDriveMotor();

        ctreConfigs = new CTREConfigs();
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
        angleMotor.setPosition(position.getRotations());
    }

    @Override
    protected void setDriveMotorEncoderPosition(Rotation2d position) {
        driveMotor.setPosition(position.getDegrees());
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
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        angleMotor.getConfigurator().apply(ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(DrivetrainConstants.ANGLE_MOTOR_INVERTED);
        angleMotor.setNeutralMode(DrivetrainConstants.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    @Override
    protected void configureDriveMotor() {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(DrivetrainConstants.DRIVE_MOTOR_INVERTED);
        driveMotor.setNeutralMode(DrivetrainConstants.DRIVE_NEUTRAL_MODE);
        driveMotor.setPosition(0);
    }

    @Override
    protected Rotation2d getAngleMotorEncoderPosition() {
        return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    }

    @Override
    protected Rotation2d getDriveMotorEncoderPosition() {
        return Rotation2d.fromRotations(driveMotor.getPosition().getValueAsDouble());
    }

    @Override
    protected double getDriveMotorEncoderVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }
}
