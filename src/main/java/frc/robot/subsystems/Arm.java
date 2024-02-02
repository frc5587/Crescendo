package frc.robot.subsystems;

import org.frc5587.lib.subsystems.PivotingArmBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

public class Arm extends PivotingArmBase {
    private static TalonFX rightMotor = new TalonFX(ArmConstants.RIGHT_MOTOR_ID);
    private static TalonFX leftMotor = new TalonFX(ArmConstants.LEFT_MOTOR_ID);
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
        enable();
        resetEncoders();
        setGoal(0);
        throughBore.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0); // placeholder values

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

    public void ArmSpeaker() {
        setGoal(ArmConstants.SPEAKER_SETPOINT);
    }

    public void ArmAmp() {
        setGoal(ArmConstants.AMP_SETPOINT);
    }

    public void ArmRest() {
        setGoal(ArmConstants.RESTING_SETPOINT);
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
