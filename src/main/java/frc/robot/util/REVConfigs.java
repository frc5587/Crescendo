
package frc.robot.util;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public final class REVConfigs {
    // initiate two motor configurations (motor settings)
    public SparkBaseConfig leftShooterConfig; 
    public SparkBaseConfig rightShooterConfig;
    public SparkBaseConfig intakeConfig;
    public SparkBaseConfig leftClimbConfig;
    public SparkBaseConfig rightClimbConfig;
    
    public REVConfigs(){
        // set each setting for the motor configuration
        leftShooterConfig.inverted(ShooterConstants.LEFT_MOTOR_INVERTED);
        leftShooterConfig.idleMode(IdleMode.kCoast);
        leftShooterConfig.smartCurrentLimit(ShooterConstants.STALL_LIMIT, ShooterConstants.FREE_LIMIT);

        rightShooterConfig.apply(leftShooterConfig); // apply left shooter's configurations to the right shooter
        // change any settings that are different between the motors
        rightShooterConfig.inverted(ShooterConstants.RIGHT_MOTOR_INVERTED);

        intakeConfig.inverted(IntakeConstants.MOTOR_INVERTED);
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeConfig.smartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);

        leftClimbConfig.inverted(ClimbConstants.LEFT_MOTOR_INVERTED);
        leftClimbConfig.idleMode(IdleMode.kBrake);
        leftClimbConfig.smartCurrentLimit(ClimbConstants.STALL_LIMIT, ClimbConstants.FREE_LIMIT);
        rightClimbConfig.apply(leftClimbConfig);
        rightClimbConfig.inverted(ClimbConstants.RIGHT_MOTOR_INVERTED);
        rightClimbConfig.follow(ClimbConstants.LEFT_MOTOR_ID);
    }
}
