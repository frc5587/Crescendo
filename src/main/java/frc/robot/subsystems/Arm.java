package frc.robot.subsystems;

import org.frc5587.lib.subsystems.PivotingArmBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;

public class Arm extends PivotingArmBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DutyCycleEncoder throughBore = new DutyCycleEncoder(0);

    public static PivotingArmConstants constants = new PivotingArmConstants(
            ArmConstants.GEARING_MOTOR_TO_ARM,
            1,
            0,
            ArmConstants.SOFT_LIMITS,
            (int) ArmConstants.ZERO_OFFSET,
            ArmConstants.ENCODER_CPR,
            ArmConstants.PID,
            ArmConstants.FF
    );

    public Arm(TalonFX leftMotor, TalonFX rightMotor) {
        super("arm", constants, leftMotor);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        resetToAbsolute();
        enable();
        throughBore.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0); // placeholder values
    }

    public Arm() {
        this(new TalonFX(ArmConstants.LEFT_MOTOR_ID), new TalonFX(ArmConstants.RIGHT_MOTOR_ID));
    }

    @Override
    public double getEncoderPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getEncoderVelocity() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setEncoderPosition(double position) {
        leftMotor.setPosition(position);
    }

    @Override
    public void configureMotors() {
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(ArmConstants.STALL_LIMIT)
                .withSupplyCurrentLimit(ArmConstants.FREE_LIMIT));
        rightMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(ArmConstants.STALL_LIMIT)
                .withSupplyCurrentLimit(ArmConstants.FREE_LIMIT));

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(),
                ArmConstants.LEFT_MOTOR_INVERTED != ArmConstants.RIGHT_MOTOR_INVERTED));
    }

    public void armSpeaker() {
        setGoal(ArmConstants.SPEAKER_SETPOINT);
    }

    public void armAmp() {
        setGoal(ArmConstants.AMP_SETPOINT);
    }

    public void armRest() {
        setGoal(ArmConstants.RESTING_SETPOINT);
    }

    public Rotation2d poseDependantArmAngle(Pose2d pose) {
        return Rotation2d.fromRadians(-Math.atan(FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getZ() / Math.sqrt(
                        Math.pow(
                                pose.getX() - (DriverStation.getAlliance().get().equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getX()), 2) +
                        Math.pow((pose.getY()
                                - (DriverStation.getAlliance().get().equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getY())),
                                2))) + Math.toRadians(50));
    }

    public void armDistanceSetpoint(Pose2d pose) throws Exception {
        setGoal(poseDependantArmAngle(pose).getRotations()); // TODO: Don't implement this until PivotingArmBase uses rotations/Rotation2d
        throw new Exception("DO NOT IMPLEMENT THIS METHOD UNLESS PivotingArmBase USES ROTATION2D or ROTATIONS!");
    }

    @Override
    public void periodic() {
        super.periodic();
        if (SmartDashboard.getBoolean("Reset Encoders", false)) {
            resetEncoders();
        }
        SmartDashboard.putBoolean("Reset Encoders", false);

        if (!throughBore.isConnected()) {
            this.disable();
            this.stop();
        }
    }

    public double getRawAbsolutePosition() {
        return throughBore.get();
    }

    public void zeroThroughBore() {
        double absolutePosition = getRawAbsolutePosition();
        throughBore.setPositionOffset(absolutePosition);
    }

    public double getArmAbsolutePosition() {
        return getRawAbsolutePosition() / ArmConstants.GEARING_ARM_TO_THROUGHBORE;
    }

    public double throughBoreToMotor(double throughBoreRotations) {
        return throughBoreRotations * ArmConstants.GEARING_THROUGHBORE_TO_MOTOR;
    }

    public void resetToAbsolute() {
        setEncoderPosition(throughBoreToMotor(getRawAbsolutePosition()));
    }
}
