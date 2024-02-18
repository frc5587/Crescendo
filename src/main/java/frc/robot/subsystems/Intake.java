package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private static CANSparkMax motor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
    private static RelativeEncoder encoder = motor.getEncoder();
    private final I2C.Port i2cPort = I2C.Port.kMXP;
    private double setpoint;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private final DoubleSupplier shooterSpeedSupplier;
    
    public Intake(DoubleSupplier shooterSpeedSupplier) {
        this.shooterSpeedSupplier = shooterSpeedSupplier;
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
        encoder.setPosition(0);
    }

    public void setVelocity(double velocity) {
        setpoint = velocity;
    }

    public double getMeasurement() {
        return ((encoder.getVelocity() / 60) * (2*Math.PI) * (IntakeConstants.WHEEL_RADIUS / IntakeConstants.GEARING));
    }

    public void stop() {
        setVelocity(0);
    }
    
    @Override
    public void periodic() {
        motor.setVoltage(IntakeConstants.FF.calculate(setpoint) - IntakeConstants.PID.calculate(setpoint - getMeasurement()));

        if(colorSensor.getProximity() > 250 && shooterSpeedSupplier.getAsDouble() == 0) {
            stop();
        }
    }
}