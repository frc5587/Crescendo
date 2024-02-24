package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SimpleMotorBase {
    private static CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    
    public Shooter() {
        super(leftMotor, ShooterConstants.FORWARD_THROTTLE, ShooterConstants.REVERSE_THROTTLE);
        configureMotors();
        idleSpeed();
    }

    @Override
    public void configureMotors() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setInverted(ShooterConstants.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(ShooterConstants.RIGHT_MOTOR_INVERTED);
        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setSmartCurrentLimit(ShooterConstants.STALL_LIMIT, ShooterConstants.FREE_LIMIT);
        rightMotor.setSmartCurrentLimit(ShooterConstants.STALL_LIMIT, ShooterConstants.FREE_LIMIT);
        rightMotor.follow(leftMotor);

    }

    public void idleSpeed() {
        motors.set(forwardThrottle / 4);
    }

    public double getMotorSpeeds() {
        return leftMotor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", getMotorSpeeds());
    }
}
