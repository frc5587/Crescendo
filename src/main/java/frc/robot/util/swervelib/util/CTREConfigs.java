package frc.robot.util.swervelib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;
    public Slot0Configs angleSlot0Configs = new Slot0Configs();
    public Slot0Configs driveSlot0Configs = new Slot0Configs();

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();
       
        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleCurrLimit = new CurrentLimitsConfigs();
        

        angleCurrLimit.SupplyCurrentLimit = Constants.DrivetrainConstants.ANGLE_CONT_LIMIT;
        angleCurrLimit.StatorCurrentLimit = Constants.DrivetrainConstants.ANGLE_PEAK_LIMIT;
        angleCurrLimit.SupplyCurrentLimitEnable = Constants.DrivetrainConstants.ANGLE_LIMIT_ENABLED;
        angleCurrLimit.StatorCurrentLimitEnable = Constants.DrivetrainConstants.ANGLE_PEAK_LIMIT_ENABLED;
        angleCurrLimit.SupplyTimeThreshold = Constants.DrivetrainConstants.ANGLE_PEAK_DURATION;

        angleSlot0Configs.kP = Constants.DrivetrainConstants.ANGLE_FPID.kP;
        angleSlot0Configs.kI = Constants.DrivetrainConstants.ANGLE_FPID.kI;
        angleSlot0Configs.kD = Constants.DrivetrainConstants.ANGLE_FPID.kD;
        // TODO add kF limits????
        swerveAngleFXConfig.CurrentLimits = angleCurrLimit;
        swerveAngleFXConfig.Slot0 = angleSlot0Configs;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveCurrLimit = new CurrentLimitsConfigs();

        driveCurrLimit.SupplyCurrentLimit = Constants.DrivetrainConstants.DRIVE_CONT_LIMIT;
        driveCurrLimit.StatorCurrentLimit = Constants.DrivetrainConstants.DRIVE_PEAK_LIMIT;
        driveCurrLimit.SupplyCurrentLimitEnable = Constants.DrivetrainConstants.DRIVE_LIMIT_ENABLED;
        driveCurrLimit.StatorCurrentLimitEnable = Constants.DrivetrainConstants.DRIVE_PEAK_LIMIT_ENABLED;
        driveCurrLimit.SupplyTimeThreshold = Constants.DrivetrainConstants.DRIVE_PEAK_DURATION;

        driveSlot0Configs.kP = Constants.DrivetrainConstants.DRIVE_FPID.kP;
        driveSlot0Configs.kI = Constants.DrivetrainConstants.DRIVE_FPID.kI;
        driveSlot0Configs.kD = Constants.DrivetrainConstants.DRIVE_FPID.kD;
        // TODO add kF limits????
        swerveDriveFXConfig.CurrentLimits = driveCurrLimit;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.DrivetrainConstants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.DrivetrainConstants.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.Slot0 = driveSlot0Configs;
        
        /* Swerve CANCoder Configuration */ 
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // TODO fix above if broken????
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.DrivetrainConstants.CANCODER_INVERTED ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        // TODO fix above
    }
}