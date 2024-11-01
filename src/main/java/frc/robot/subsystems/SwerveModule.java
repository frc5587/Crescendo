package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveModuleBase;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.util.swervelib.util.CTREConfigs;

public class SwerveModule extends SwerveModuleBase {
    private TalonFX angleMotor, driveMotor;
    private CANcoder angleEncoder;
    private Rotation2d canCoderOffset;
    public static CTREConfigs ctreConfigs = new CTREConfigs();
    public SwerveModule(SwerveModuleConstants moduleConstants, TalonFX driveMotor, TalonFX angleMotor, CANcoder angleEncoder, Rotation2d canCoderOffset) {
        super(moduleConstants, angleMotor, driveMotor);
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.angleEncoder = angleEncoder;
        this.canCoderOffset = canCoderOffset;
        configureAngleEncoder();
        configureAngleMotor();
        configureDriveMotor();
    }

    public void setBrakeMode(boolean enabled) {
        angleMotor.setNeutralMode(enabled ? NeutralModeValue.Brake: NeutralModeValue.Coast);
        driveMotor.setNeutralMode(enabled ? NeutralModeValue.Brake: NeutralModeValue.Coast);
    }

    @Override
    public Rotation2d getAbsoluteEncoderValue() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    protected void setAngleMotorPosition(Rotation2d position) {
        angleMotor.setControl(new PositionVoltage(position.getRotations()));
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
        driveMotor.setControl(new VelocityVoltage(mpsToRotationsPerSecond(velocityMPS)));
    }

    @Override
    protected void configureAngleEncoder() {
        angleEncoder.getConfigurator().apply(ctreConfigs.swerveCanCoderConfig.MagnetSensor.withMagnetOffset(canCoderOffset.getRotations()));
    }

    @Override
    protected void configureAngleMotor() {
        angleMotor.getConfigurator().apply(ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(DrivetrainConstants.ANGLE_MOTOR_INVERTED);
        angleMotor.setNeutralMode(DrivetrainConstants.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    @Override
    protected void configureDriveMotor() {
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
        return Rotation2d.fromRotations(-driveMotor.getPosition().getValueAsDouble());
    }

    @Override
    protected double getDriveMotorEncoderVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }
}
