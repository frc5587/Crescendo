package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
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
        setVelocity(swerveSpeedSupplier.getAsDouble() * 2.);
    }

    public void backward() {
        setVelocity(-IntakeConstants.REVERSE_THROTTLE);
    }

    public void stop() {
        setVelocity(0);
    }
    
    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }
    @Override
    protected void useOutput(double output, double setpoint) {
        motor.setVoltage(IntakeConstants.FF.calculate(setpoint) + output);
    }

    @Override
    public void periodic() {
        if(getLimitSwitch() && shooterSpeedSupplier.getAsDouble() == 0) {
            stop();
        }
    }

}