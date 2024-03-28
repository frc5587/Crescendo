package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ClimbConstants;

public class Climb extends ProfiledPIDSubsystem {

    private static CANSparkMax leftMotor = new CANSparkMax(ClimbConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(ClimbConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private static SimpleMotorFeedforward ff = ClimbConstants.FF;

    public Climb() {
        super(ClimbConstants.PID);
        configureMotors();
        enable();
    }

     public void configureMotors() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setInverted(ClimbConstants.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(ClimbConstants.RIGHT_MOTOR_INVERTED);
        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setSmartCurrentLimit(ClimbConstants.STALL_LIMIT, ClimbConstants.FREE_LIMIT);
        rightMotor.setSmartCurrentLimit(ClimbConstants.STALL_LIMIT, ClimbConstants.FREE_LIMIT);
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
        rightMotor.follow(leftMotor);
        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    public void up() {
        setGoal(ClimbConstants.TOP_POSITION);
    }

    public void down() {
        setGoal(ClimbConstants.BOTTOM_POSITION);
    }
    @Override
    protected void useOutput(double output, State setpoint) {
        SmartDashboard.putNumber("Climb Output", output);
        leftMotor.setVoltage(output + ff.calculate(setpoint.position));
    }
    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
    }
}
