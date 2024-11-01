package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
    // private static CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    // private static CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private SimpleMotorFeedforward ff, leftFF, rightFF;
    private Supplier<Pose2d> poseSupplier;
    private final ProfiledPIDController leftPID, rightPID;

    public Shooter(CANSparkMax leftMotor, CANSparkMax rightMotor, Supplier<Pose2d> poseSupplier) {
        super(ShooterConstants.PID);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.ff = ShooterConstants.FF;
        this.leftFF = ShooterConstants.LEFT_FF;
        this.rightFF = ShooterConstants.RIGHT_FF;
        this.poseSupplier = poseSupplier;
        this.leftPID = ShooterConstants.LEFT_PID;
        this.rightPID = ShooterConstants.RIGHT_PID;
        configureMotors();
        disable();
        getController().setTolerance(0.2);
        leftPID.setTolerance(0.3);
        rightPID.setTolerance(0.3);
        SmartDashboard.putNumber("Desired Speed", 0.0);
    }

    public Shooter(Supplier<Pose2d> poseSupplier) {
        this(new CANSparkMax(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless),
        new CANSparkMax(ShooterConstants.RIGHT_MOTOR_ID, MotorType.kBrushless),
        poseSupplier
        );
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
        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    public void idleSpeed() {
        setLeftSpeed(6);
        setRightSpeed(6);
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

    public double getLeftMPS() {
        return (leftMotor.getEncoder().getVelocity() / 60.) * ShooterConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getRightMPS() {
        return (rightMotor.getEncoder().getVelocity() / 60.) * ShooterConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public void setLeftSpeed(double speedMPS) {
        leftMotor.setVoltage(leftPID.calculate(getLeftMPS(), speedMPS) + leftFF.calculate(speedMPS));
    }

    public void setRightSpeed(double speedMPS) {
        rightMotor.setVoltage(rightPID.calculate(getRightMPS(), -speedMPS) + rightFF.calculate(-speedMPS));
    }

    public double getPositionMeters() {
        return leftMotor.getEncoder().getPosition() * ShooterConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getMeasuredMotorSpeedsAsPercentage() {
        return leftMotor.getEncoder().getVelocity() / 60. / ShooterConstants.MAX_MOTOR_SPEED_RPS;
    }

    public void forward() {
        setLeftSpeed(poseDepenantShooterSpeed(poseSupplier.get()));
        setRightSpeed(poseDepenantShooterSpeed(poseSupplier.get()));
    }

    public double poseDepenantShooterSpeed(Pose2d pose) {
        double distance = Math.sqrt(
                        Math.pow(
                                pose.getX() - (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getX()), 2) +
                        Math.pow((pose.getY()
                                - (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getY())),
                                2));

        // https://www.desmos.com/calculator/bplceypa5r
        return MathUtil.clamp((2 * distance) + 13.32, 12., 23.0);
    }

    public void backward() {
        setLeftSpeed(-5);
        setRightSpeed(-5);
    }

    public void stop() {
        setLeftSpeed(0.);
        setRightSpeed(0.);
    }

    public void stopVoltage() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public boolean isSpunUp() {
        return (getController().atGoal() && getController().getSetpoint().position > 8) || (!isEnabled() && leftPID.atGoal() && rightPID.atGoal());
    }

    public void spinUpToAmp() {
        setGoal(ShooterConstants.AMP_THROTTLE);
    }
    public void pancake() {
        this.disable();
        leftMotor.setVoltage(5.75);
        rightMotor.setVoltage(-0.85);
    }

    public void setVoltage(double voltage) {
        leftMotor.set(voltage / RobotController.getBatteryVoltage());
        rightMotor.set(-(voltage / RobotController.getBatteryVoltage()));
    }

    public double getVoltage() {
        return leftMotor.get() * RobotController.getBatteryVoltage();
    }

    @Override
    public void periodic() {
        double distance = Math.sqrt(
                        Math.pow(
                                poseSupplier.get().getX() - (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getX()), 2) +
                        Math.pow((poseSupplier.get().getY()
                                - (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getY())),
                                2));
        SmartDashboard.putData(getController());
        SmartDashboard.putNumber("Shooter Measured Speed", getWheelSpeedsMPS());
        SmartDashboard.putNumber("Shooter Volts", getVoltage());
        SmartDashboard.putBoolean("Shooter IsSpunUp", isSpunUp());
        SmartDashboard.putNumber("Left Speed", getLeftMPS());
        SmartDashboard.putNumber("Right Speed", getRightMPS());
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("Shooter Goal", poseDepenantShooterSpeed(poseSupplier.get()));
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        SmartDashboard.putNumber("Shooter Output", output);
        leftMotor.setVoltage(output + ff.calculate(setpoint.position));
        rightMotor.setVoltage(- (output + ff.calculate(setpoint.position)));
    }

    @Override
    protected double getMeasurement() {
        return getWheelSpeedsMPS();
    }
}
