package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SimpleMotorBase {
    private static CANSparkMax motor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    
    public Intake() {
        super(motor, ShooterConstants.FORWARD_THROTTLE, ShooterConstants.REVERSE_THROTTLE);
    }

    @Override
    public void configureMotors() {
        motor.restoreFactoryDefaults();
        motor.setInverted(IntakeConstants.MOTOR_INVERTED);
        motor.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Color Sensor Proximity", colorSensor.getProximity());

        while (colorSensor.getProximity() > 1000) /*placeholder value*/{
            if (.get() != 0) {
                break;
            }
            stop();
        }
    }
}