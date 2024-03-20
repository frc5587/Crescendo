package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends ProfiledPIDSubsystem {
    private static CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private static CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private static SimpleMotorFeedforward ff = ShooterConstants.FF;
    private Supplier<Pose2d> poseSupplier;

    public Shooter(Supplier<Pose2d> poseSupplier) {
        super(ShooterConstants.PID);
        configureMotors();
        enable();
        this.poseSupplier = poseSupplier;
        getController().setTolerance(0.2);
        idleSpeed();
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
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
        rightMotor.follow(leftMotor);
    }

    public void idleSpeed() {
        // leftMotor.set(ShooterConstants.IDLE_SPEED);
        setGoal(5);
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
        // setGoal(20);
        setGoal(poseDepenantShooterSpeed(poseSupplier.get()));
    }

    public double poseDepenantShooterSpeed(Pose2d pose) {
        double distance = Math.sqrt(
                        Math.pow(
                                pose.getX() - (DriverStation.getAlliance().get().equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getX()), 2) +
                        Math.pow((pose.getY()
                                - (DriverStation.getAlliance().get().equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getY())),
                                2));

        // return (2.8 * distance) + 9.86;
        // https://www.desmos.com/calculator/bplceypa5r
        return MathUtil.clamp((2.8 * distance) + 9.86, 12., 21.0);
    }

    public void backward() {
        setGoal(5);
    }

    public void stop() {
        setGoal(0.);
    }

    public void stopVoltage() {
        leftMotor.set(0);
    }

    public boolean isSpunUp() {
        // return ShooterConstants.FORWARD_THROTTLE - getMeasuredMotorSpeedsAsPercentage() <= 0.075;
        return getController().atGoal() && getController().getSetpoint().position > 8;
    }

    public void spinUpToAmp() {
        setGoal(ShooterConstants.AMP_THROTTLE);
    }

    public void setVoltage(double voltage) {
        leftMotor.set(voltage / RobotController.getBatteryVoltage());
    }

    public double getVoltage() {
        return leftMotor.get() * RobotController.getBatteryVoltage();
    }

    @Override
    public void periodic() {
        super.periodic();
        // SmartDashboard.putNumber("Shooter Set Speed", getMotorSpeeds());
        SmartDashboard.putData(getController());
        SmartDashboard.putNumber("Shooter Measured Speed", getWheelSpeedsMPS());
        SmartDashboard.putNumber("Shooter Position", getPositionMeters());
        SmartDashboard.putNumber("Shooter Volts", getVoltage());
        SmartDashboard.putBoolean("Shooter IsSpunUp", isSpunUp());
        // SmartDashboard.putNumber("Shooter Measured Percentage", getMeasuredMotorSpeedsAsPercentage());
        // SmartDashboard.putBoolean("Shooter Spun Up", isSpunUp());
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        SmartDashboard.putNumber("Shooter Output", output);
        leftMotor.setVoltage(output + ff.calculate(setpoint.position));
    }

    @Override
    protected double getMeasurement() {
        return getWheelSpeedsMPS();
    }
}
