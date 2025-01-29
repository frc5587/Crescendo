package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.REVConfigs;

public class Climb extends ProfiledPIDSubsystem {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private boolean brakeModeEnabled = true;
    private REVConfigs motorConfigs;

    public Climb(SparkMax leftMotor, SparkMax rightMotor) {
        super(ClimbConstants.PID);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        configureMotors();
        enable();
        getController().setTolerance(Units.inchesToMeters(0.5));
        SmartDashboard.putBoolean("Climb Enabled", isEnabled());
        SmartDashboard.putBoolean("Climb Brake Mode", brakeModeEnabled);
    }
    
    public Climb() {
        this(new SparkMax(ClimbConstants.LEFT_MOTOR_ID, MotorType.kBrushless),
        new SparkMax(ClimbConstants.RIGHT_MOTOR_ID, MotorType.kBrushless)
        );
    }

    public void configureMotors() {
        leftMotor.configure(motorConfigs.leftClimbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(motorConfigs.rightClimbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetEncoders();
    }

    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }

    public void hookTop() {
        setGoal(ClimbConstants.TOP_POSITION);
    }

    public void hookMiddle() {
        setGoal(ClimbConstants.MIDDLE_POSITION);
    }

    public void hookBottom() {
        setGoal(ClimbConstants.BOTTOM_POSITION);
    }

    public Command hookTopCommand() {
        return new InstantCommand(this::hookTop);
    }

    public Command hookMiddleCommand() {
        return new InstantCommand(this::hookMiddle);
    }

    public Command hookBottomCommand() {
        return new InstantCommand(this::hookBottom);
    }

    public void set(double percentOutput) {
        leftMotor.set(percentOutput);
        rightMotor.set(percentOutput);
    }

    public void setVoltage(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
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
        if(SmartDashboard.getBoolean("Climb Enabled", true) && !isEnabled()) {
            this.enable();
        }
        else if(!SmartDashboard.getBoolean("Climb Enabled", true) && isEnabled()) {
            this.disable();
        }

        if(SmartDashboard.getBoolean("Climb Brake Mode", true) != brakeModeEnabled) {
            this.brakeModeEnabled = SmartDashboard.getBoolean("Climb Brake Mode", true);
            motorConfigs.leftClimbConfig.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
            motorConfigs.rightClimbConfig.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
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
            leftMotor.setVoltage(output);
        }
    }

    protected double getHookHeightMeters() {
        return ((leftMotor.getEncoder().getPosition() * ClimbConstants.SPOOL_CIRCUMFERENCE_METERS) / ClimbConstants.GEARING);
    }
    @Override
    protected double getMeasurement() {
        return getHookHeightMeters();
    }
}
