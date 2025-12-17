package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.REVConfigs;

public class Intake extends PIDSubsystem {
    private SparkMax motor = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(2);
    private final BooleanSupplier shooterSpunUpSupplier, spunUpOverrideSupplier;
    private final DoubleConsumer rumbleConsumer;
    private static SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private double rumbleTimerEndTime = Timer.getFPGATimestamp() + 1;
    private boolean switchTimeHasBeenSet, shotIsConfirmed = false;
    
    public Intake(BooleanSupplier shooterSpunUpSupplier, BooleanSupplier spunUpOverrideSupplier, DoubleSupplier swerveSpeedSupplier, DoubleConsumer rumbleConsumer) {
        super(IntakeConstants.PID);
        this.shooterSpunUpSupplier = shooterSpunUpSupplier;
        this.spunUpOverrideSupplier = spunUpOverrideSupplier;
        this.rumbleConsumer = rumbleConsumer;
        configureMotors();
        this.enable();
    }

    public void configureMotors() {
        intakeConfig.inverted(IntakeConstants.MOTOR_INVERTED);
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeConfig.smartCurrentLimit(IntakeConstants.STALL_LIMIT, IntakeConstants.FREE_LIMIT);
        
        motor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    }

    public void backward() {
        motor.set(-IntakeConstants.REVERSE_THROTTLE);
    }

    public void stop() {
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

    public double getMotorSetSpeed() {
        return motor.get();
    }

    public void confirmShot() {
        shotIsConfirmed = true;
    }

    public void denyShot() {
        shotIsConfirmed = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Setpoint", motor.get());
        SmartDashboard.putNumber("Intake Measurement", getMeasurement());
        SmartDashboard.putBoolean("Intake Limit Switch", getLimitSwitch());
        if(DriverStation.isAutonomousEnabled()) {
            forward();
        }
        if(getLimitSwitch() && switchTimeHasBeenSet && Timer.getFPGATimestamp() < rumbleTimerEndTime) {
            rumbleConsumer.accept(1.);
        }
        else if(getLimitSwitch() && !switchTimeHasBeenSet) {
            rumbleTimerEndTime = Timer.getFPGATimestamp() + .6;

            switchTimeHasBeenSet = true;
            rumbleConsumer.accept(1.);
        }
        else {
            rumbleConsumer.accept(0.);
        }
        if(!getLimitSwitch()) {
            switchTimeHasBeenSet = false;
        }
    }
}
