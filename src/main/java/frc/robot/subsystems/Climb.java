package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;

public class Climb extends ProfiledPIDSubsystem {

    public static CANSparkMax leftMotor = new CANSparkMax(ClimbConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    public static CANSparkMax rightMotor = new CANSparkMax(ClimbConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private ElevatorFeedforward ff = ClimbConstants.FF;
    private boolean wasManuallyDisabled = false;
    private boolean brakeModeEnabled = true;

    public Climb() {
        super(ClimbConstants.PID);
        configureMotors();
        enable();
        SmartDashboard.putBoolean("Climb Enabled", isEnabled());
        SmartDashboard.putBoolean("Climb Brake Mode", brakeModeEnabled);
    }

    public void configureMotors() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setInverted(ClimbConstants.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(ClimbConstants.RIGHT_MOTOR_INVERTED);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
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

    public void set(double percentOutput) {
        leftMotor.set(percentOutput);
        rightMotor.set(percentOutput);
    }

    public void setVoltage(double volts) {
        leftMotor.set(volts / RobotController.getBatteryVoltage());
        rightMotor.set(volts / RobotController.getBatteryVoltage());
    }

    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Climb Goal", this.getController().getGoal().position);
        SmartDashboard.putNumber("Raw Climb Position", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Climb Position", getMeasurement());
        if(SmartDashboard.getBoolean("Reset Climb Encoders", false)) {
            resetEncoders();
        }
        SmartDashboard.putBoolean("Reset Climb Encoders", false);
        if(SmartDashboard.getBoolean("Climb Enabled", true) && wasManuallyDisabled) {
            this.enable();
            this.wasManuallyDisabled = false;
        }
        else if(!SmartDashboard.getBoolean("Climb Enabled", true) && !wasManuallyDisabled) {
            wasManuallyDisabled = true;
            this.disable();
        }

        if(SmartDashboard.getBoolean("Climb Brake Mode", true) != brakeModeEnabled) {
            this.brakeModeEnabled = SmartDashboard.getBoolean("Climb Brake Mode", true);
            leftMotor.setIdleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
            rightMotor.setIdleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
        }
    }
    
    @Override
    protected void useOutput(double output, State setpoint) {
        SmartDashboard.putNumber("Climb Output", output);
        if ((getMeasurement() < ArmConstants.SOFT_LIMITS[0].getRadians() && output < 0.)
                || (getMeasurement() > ArmConstants.SOFT_LIMITS[1].getRadians() && output > 0.)) {
            setVoltage(0);
        }
        else {
            leftMotor.setVoltage(output + ff.calculate(setpoint.position));
        }
    }

    public double getVoltage() {
        return leftMotor.get() * RobotController.getBatteryVoltage();
    }

    public double getLinearVelocity() {
        return ((leftMotor.getEncoder().getVelocity() / 60) * ClimbConstants.SPOOL_CIRCUMFERENCE_METERS) / ClimbConstants.GEARING;
    }

    @Override
    public double getMeasurement() {
        return (leftMotor.getEncoder().getPosition() * ClimbConstants.SPOOL_CIRCUMFERENCE_METERS) / ClimbConstants.GEARING;
    }
}
