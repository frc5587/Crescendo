package frc.robot.util.swervelib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;
    public Slot0Configs slot0Configs;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();
       
        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleCurrLimit = new CurrentLimitsConfigs();
        

        angleCurrLimit.SupplyCurrentLimit = Constants.SwerveConstants.ANGLE_CONT_LIMIT;
        angleCurrLimit.StatorCurrentLimit = Constants.SwerveConstants.ANGLE_PEAK_LIMIT;
        angleCurrLimit.SupplyCurrentLimitEnable = Constants.SwerveConstants.ANGLE_LIMIT_ENABLED;
        angleCurrLimit.StatorCurrentLimitEnable = Constants.SwerveConstants.ANGLE_PEAK_LIMIT_ENABLED;
        angleCurrLimit.SupplyTimeThreshold = Constants.SwerveConstants.ANGLE_PEAK_DURATION;

        slot0Configs.kP = Constants.SwerveConstants.ANGLE_FPID.kP;
        slot0Configs.kI = Constants.SwerveConstants.ANGLE_FPID.kI;
        slot0Configs.kD = Constants.SwerveConstants.ANGLE_FPID.kD;
        // TODO add kF limits????
        swerveAngleFXConfig.CurrentLimits = angleCurrLimit;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveCurrLimit = new CurrentLimitsConfigs();

        driveCurrLimit.SupplyCurrentLimit = Constants.SwerveConstants.DRIVE_CONT_LIMIT;
        driveCurrLimit.StatorCurrentLimit = Constants.SwerveConstants.DRIVE_PEAK_LIMIT;
        driveCurrLimit.SupplyCurrentLimitEnable = Constants.SwerveConstants.DRIVE_LIMIT_ENABLED;
        driveCurrLimit.StatorCurrentLimitEnable = Constants.SwerveConstants.DRIVE_PEAK_LIMIT_ENABLED;
        driveCurrLimit.SupplyTimeThreshold = Constants.SwerveConstants.DRIVE_PEAK_DURATION;

        slot0Configs.kP = Constants.SwerveConstants.DRIVE_FPID.kP;
        slot0Configs.kI = Constants.SwerveConstants.DRIVE_FPID.kI;
        slot0Configs.kD = Constants.SwerveConstants.DRIVE_FPID.kD;
        // TODO add kF limits????
        swerveDriveFXConfig.CurrentLimits = driveCurrLimit;
        swerveDriveFXConfig.OpenLoopRamps = Constants.SwerveConstants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps = Constants.SwerveConstants.CLOSED_LOOP_RAMP;
        
        /* Swerve CANCoder Configuration */ 
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // TODO fix above if broken????
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.CANCODER_INVERTED;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}