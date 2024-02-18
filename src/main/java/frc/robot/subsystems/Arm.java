package frc.robot.subsystems;

import java.util.function.Supplier;

import org.frc5587.lib.subsystems.PivotingArmBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;

public class Arm extends PivotingArmBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final Supplier<Pose2d> poseSupplier;
    private final DutyCycleEncoder throughBore = new DutyCycleEncoder(0);
    private boolean wasManuallyDisabled = false;
    private boolean manualMode = true;

    public static PivotingArmConstants constants = new PivotingArmConstants(ArmConstants.GEARING_MOTOR_TO_ARM,
            new Rotation2d(), ArmConstants.SOFT_LIMITS, ArmConstants.ZERO_OFFSET, ArmConstants.PID, ArmConstants.FF);

    public Arm(TalonFX leftMotor, TalonFX rightMotor, Supplier<Pose2d> poseSupplier) {
        super(constants, leftMotor);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.poseSupplier = poseSupplier;
        throughBore.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
        resetToAbsolute();
        enable();
        SmartDashboard.putBoolean("Arm Enabled", isEnabled());
        
    }

    public Arm(Supplier<Pose2d> poseSupplier) {
        this(new TalonFX(ArmConstants.LEFT_MOTOR_ID, "canivore"), new TalonFX(ArmConstants.RIGHT_MOTOR_ID, "canivore"), poseSupplier); 
    }

    @Override
    public Rotation2d getEncoderPosition() {
        return Rotation2d.fromRotations(leftMotor.getPosition().getValueAsDouble());
    }

    @Override
    public double getEncoderVelocity() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setEncoderPosition(Rotation2d position) {
        leftMotor.setPosition(position.getRotations());
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
        return Rotation2d.fromRadians(-Math.atan(FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getZ() /
                pose.getTranslation().getDistance((DriverStation.getAlliance().get().equals(Alliance.Blue))
                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.toTranslation2d()
                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.toTranslation2d())
                + Math.toRadians(56)));
    }

    public void armDistanceSetpoint(Pose2d pose) {
        setGoal(poseDependantArmAngle(pose).getRadians());
    }

    @Override
    public void resetEncoders() {
        zeroThroughBore();
        resetToAbsolute();
    }

    @Override
    public void periodic() {
        super.periodic();
        if (SmartDashboard.getBoolean("Reset Arm Encoders", false)) {
            resetEncoders();
        }
        SmartDashboard.putBoolean("Reset Arm Encoders", false);
        SmartDashboard.putBoolean("ThroughBore Is Connected", throughBore.isConnected());
        if (!throughBore.isConnected() || !SmartDashboard.getBoolean("Arm Enabled", true)) {
            this.disable();
            this.stop();
            this.wasManuallyDisabled = true;
            SmartDashboard.putBoolean("Arm Enabled", false);
        }

        if(SmartDashboard.getBoolean("Arm Enabled", true) && wasManuallyDisabled) {
            this.enable();
            this.wasManuallyDisabled = false;
        }

        SmartDashboard.putNumber("Arm Absolute Pos", getArmAbsolutePosition().getDegrees());
        SmartDashboard.putNumber("Arm Relative Pos", getAngleDegrees());

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(),
                ArmConstants.LEFT_MOTOR_INVERTED != ArmConstants.RIGHT_MOTOR_INVERTED));

        SmartDashboard.putNumber("Arm Goal Degrees", Units.radiansToDegrees(this.getController().getGoal().position));

        if(!manualMode) {
            armDistanceSetpoint(poseSupplier.get());
        }

        SmartDashboard.putData("Arm PID", this.getController());
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("Old", poseDependantArmAngle(poseSupplier.get()).getDegrees());
        SmartDashboard.putNumber("New", newPoseDependantArmAngle(poseSupplier.get()).getDegrees());
    }

    public void setManualMode(boolean manualMode) {
        this.manualMode = manualMode;
    }

    public Rotation2d getRawAbsolutePosition() {
        return Rotation2d.fromRotations(throughBore.getAbsolutePosition());
    }

    public Rotation2d getZeroedArmAbsolutePosition() {
        return getRawAbsolutePosition().minus(Rotation2d.fromRotations(throughBore.getPositionOffset()));
    }

    public void zeroThroughBore() {
        throughBore.setPositionOffset(getRawAbsolutePosition().getRotations());
    }

    public Rotation2d getArmAbsolutePosition() {
        return getZeroedArmAbsolutePosition().div(ArmConstants.GEARING_ARM_TO_THROUGHBORE);
    }

    public Rotation2d throughBoreToMotor(Rotation2d throughBoreRotations) {
        return throughBoreRotations.times(ArmConstants.GEARING_THROUGHBORE_TO_MOTOR);
    }

    public void resetToAbsolute() {
        setEncoderPosition(throughBoreToMotor(getRawAbsolutePosition()));
    }
}
