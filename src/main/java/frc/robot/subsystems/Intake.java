package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SimpleMotorBase {
    private static CANSparkMax motor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
    private final I2C.Port i2cPort = I2C.Port.kMXP;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private final DoubleSupplier shooterSpeedSupplier;
    private final DigitalInput limitSwitch = new DigitalInput(1);
    
    public Intake(DoubleSupplier shooterSpeedSupplier) {
        super(motor, ShooterConstants.FORWARD_THROTTLE, ShooterConstants.REVERSE_THROTTLE);
        this.shooterSpeedSupplier = shooterSpeedSupplier;
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
        SmartDashboard.putBoolean("Color Sensor Connected?", colorSensor.isConnected());
        SmartDashboard.putBoolean("Limit Switch", !limitSwitch.get());

        if(!limitSwitch.get() && shooterSpeedSupplier.getAsDouble() == 0) { //colorSensor.getProximity() > 250 && 
            stop();
        }
    }
}