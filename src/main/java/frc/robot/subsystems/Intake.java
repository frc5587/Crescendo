package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.IntakeConstants;

public class Intake extends PIDSubsystem {
    private CANSparkMax motor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(1);
    private final DoubleSupplier shooterSpeedSupplier, swerveSpeedSupplier;
    
    public Intake(DoubleSupplier shooterSpeedSupplier, DoubleSupplier swerveSpeedSupplier) {
        super(IntakeConstants.PID);
        this.shooterSpeedSupplier = shooterSpeedSupplier;
        this.swerveSpeedSupplier = swerveSpeedSupplier;
        configureMotors();
    }

    public void configureMotors() {
        resetEncoders();
        motor.restoreFactoryDefaults();
        motor.setInverted(IntakeConstants.MOTOR_INVERTED);
        motor.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);
        motor.setIdleMode(IdleMode.kBrake);
        this.enable();
    }

    public void resetEncoders() {
        motor.getEncoder().setPosition(0);
    }

    public void setVelocity(double velocity) {
        setSetpoint(velocity);
    }

    public double getMeasurement() {
        return ((motor.getEncoder().getVelocity() / 60) * (2*Math.PI) * (IntakeConstants.WHEEL_RADIUS / IntakeConstants.GEARING));
    }

    public void forward() {
        motor.set(IntakeConstants.FORWARD_THROTTLE);
        // double swerveSpeed = Math.abs(swerveSpeedSupplier.getAsDouble());
        // setVelocity(swerveSpeed * 15. < IntakeConstants.MINIMUM_VELOCITY ? IntakeConstants.MINIMUM_VELOCITY : swerveSpeed * IntakeConstants.MINIMUM_VELOCITY);
    }

    public void backward() {
        // setVelocity(-IntakeConstants.REVERSE_THROTTLE);
        motor.set(-IntakeConstants.REVERSE_THROTTLE);
    }

    public void stop() {
        // setVelocity(0);
        motor.set(0.);
    }
    
    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        motor.setVoltage(IntakeConstants.FF.calculate(setpoint) + output);
        SmartDashboard.putNumber("Intake Output", output);
    }

    @Override
    public void periodic() {
        // super.periodic();
        // motor.setVoltage(IntakeConstants.FF.calculate(setpoint) - IntakeConstants.PID.calculate(setpoint - getMeasurement()));
        SmartDashboard.putNumber("Intake Setpoint", getSetpoint());
        SmartDashboard.putNumber("Intake Measurement", getMeasurement());
        SmartDashboard.putBoolean("Intake Limit Switch", getLimitSwitch());
        if(getLimitSwitch() && shooterSpeedSupplier.getAsDouble() < 0.55) {
            stop();
        }
    }

}