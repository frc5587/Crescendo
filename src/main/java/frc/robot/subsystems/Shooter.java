package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends ProfiledPIDSubsystem {
    private static CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private static SimpleMotorFeedforward ff = ShooterConstants.FF;

    public Shooter() {
        super(ShooterConstants.PID);
        configureMotors();
        // idleSpeed();
    }

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
        leftMotor.set(ShooterConstants.IDLE_SPEED);
    }

    public double getMotorSpeeds() {
        return leftMotor.get();
    }

    public double getMeasuredMotorSpeedsRPS() {
        return leftMotor.getEncoder().getVelocity() / 60.;
    }

    public double getWheelSpeedsMPS() {
        return getMeasuredMotorSpeedsRPS() * ShooterConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getPositionMeters() {
        return leftMotor.getEncoder().getPosition() * ShooterConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getMeasuredMotorSpeedsAsPercentage() {
        return leftMotor.getEncoder().getVelocity() / 60. / ShooterConstants.MAX_MOTOR_SPEED_RPS;
    }

    public void forward() {
        setGoal(0.5);
    }

    public void backward() {
        setGoal(0.5);
    }

    public void stop() {
        setGoal(0.);
    }

    public boolean isSpunUp() {
        return ShooterConstants.FORWARD_THROTTLE - getMeasuredMotorSpeedsAsPercentage() <= 0.075;
    }

    public void spinUpToAmp() {
        leftMotor.set(ShooterConstants.AMP_THROTTLE);
    }

    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    public double getVoltage() {
        return leftMotor.get() * leftMotor.getBusVoltage();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Shooter Set Speed", getMotorSpeeds());
        // SmartDashboard.putNumber("Shooter Measured Speed", getMeasuredMotorSpeeds());
        // SmartDashboard.putNumber("Shooter Measured Percentage", getMeasuredMotorSpeedsAsPercentage());
        // SmartDashboard.putBoolean("Shooter Spun Up", isSpunUp());
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        leftMotor.set(output + ff.calculate(setpoint.position));
    }

    @Override
    protected double getMeasurement() {
        return getWheelSpeedsMPS();
    }
}
