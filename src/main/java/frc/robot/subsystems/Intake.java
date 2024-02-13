package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SimpleMotorBase {
    private static CANSparkMax motor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
 
    
    public Intake() {
        super(motor, ShooterConstants.FORWARD_THROTTLE, ShooterConstants.REVERSE_THROTTLE);
    }

    @Override
    public void configureMotors() {
        motor.restoreFactoryDefaults();
        motor.setInverted(IntakeConstants.MOTOR_INVERTED);
        motor.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);
    }
}