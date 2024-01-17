package frc.robot.util.swervelib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        // SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.SwerveConstants.ANGLE_LIMIT_ENABLED, 
        //     Constants.SwerveConstants.ANGLE_CONT_LIMIT, 
        //     Constants.SwerveConstants.ANGLE_PEAK_LIMIT, 
        //     Constants.SwerveConstants.ANGLE_PEAK_DURATION);

        // swerveAngleFXConfig.slot0.kP = Constants.SwerveConstants.ANGLE_FPID.kP;
        // swerveAngleFXConfig.slot0.kI = Constants.SwerveConstants.ANGLE_FPID.kI;
        // swerveAngleFXConfig.slot0.kD = Constants.SwerveConstants.ANGLE_FPID.kD;
        // swerveAngleFXConfig.slot0.kF = Constants.SwerveConstants.ANGLE_FPID.kF;
        // swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.SwerveConstants.DRIVE_LIMIT_ENABLED, 
        //     Constants.SwerveConstants.DRIVE_CONT_LIMIT, 
        //     Constants.SwerveConstants.DRIVE_PEAK_LIMIT, 
        //     Constants.SwerveConstants.DRIVE_PEAK_DURATION);

        // swerveDriveFXConfig.slot0.kP = Constants.SwerveConstants.DRIVE_FPID.kP;
        // swerveDriveFXConfig.slot0.kI = Constants.SwerveConstants.DRIVE_FPID.kI;
        // swerveDriveFXConfig.slot0.kD = Constants.SwerveConstants.DRIVE_FPID.kD;
        // swerveDriveFXConfig.slot0.kF = Constants.SwerveConstants.DRIVE_FPID.kF;
        // swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        // swerveDriveFXConfig.openloopRamp = Constants.SwerveConstants.OPEN_LOOP_RAMP;
        // swerveDriveFXConfig.closedloopRamp = Constants.SwerveConstants.CLOSED_LOOP_RAMP;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.DrivetrainConstants.CANCODER_INVERTED ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        // swerveCanCoderConfig.MagnetSensor. = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.MagnetSensor. = SensorTimeBase.PerSecond;
    }
}