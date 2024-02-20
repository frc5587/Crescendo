package frc.robot.subsystems;

import java.util.function.Supplier;

import org.frc5587.lib.subsystems.PivotingArmBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;

public class Arm extends PivotingArmBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final Supplier<Pose2d> poseSupplier;
    private final DutyCycleEncoder throughBore = new DutyCycleEncoder(0);
    private boolean wasManuallyDisabled = false;
    private boolean manualMode = true;
    private boolean breakModeEnabled = true;

    public static PivotingArmConstants constants = new PivotingArmConstants(ArmConstants.GEARING_MOTOR_TO_ARM,
            new Rotation2d(), ArmConstants.SOFT_LIMITS, ArmConstants.ZERO_OFFSET, ArmConstants.PID, ArmConstants.FF);
        

    public Arm(TalonFX leftMotor, TalonFX rightMotor, Supplier<Pose2d> poseSupplier) {
        super(constants, leftMotor);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.poseSupplier = poseSupplier;
        throughBore.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
        resetToAbsolute();
        getController().setTolerance(Units.degreesToRadians(1));
        enable();
        SmartDashboard.putBoolean("Arm Enabled", isEnabled());
        SmartDashboard.putBoolean("Arm Debug On?", false);
        SmartDashboard.putBoolean("Break Mode Enabled", breakModeEnabled);
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

    public InstantCommand armAmpCommand() {
        return new InstantCommand(() -> {
            setManualMode(true);
            armAmp();
        });
    }

    public void armRest() {
        setGoal(ArmConstants.RESTING_SETPOINT);
    }

    public InstantCommand armRestCommand() {
        return new InstantCommand(() -> {
            setManualMode(true);
            armRest();
        });
    }

    // public Rotation2d poseDependantArmAngle(Pose2d pose) {
    //     return Rotation2d.fromRadians(-Math.atan(FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getZ() /
    //             pose.getTranslation().getDistance((DriverStation.getAlliance().get().equals(Alliance.Blue))
    //                     ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.toTranslation2d()
    //                     : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.toTranslation2d())
    //             + Math.toRadians(56)));
    // }

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
                                2))) + Math.toRadians(56)).times(1.05);
    }

    public void armDistanceSetpoint(Pose2d pose) {
        setGoal(poseDependantArmAngle(pose).getRadians());
    }

    @Override
    public void resetEncoders() {
        zeroThroughBore();
        resetToAbsolute();
    }

    public void chinUp() {
        //  = new ProfiledPIDController(6., 0., 0., new TrapezoidProfile.Constraints(Math.PI, Math.PI));
        setGoal(0);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        /** SOFT LIMITS */
        /** output should be feedforward + calculated PID. */
        /** if the arm is below the limit and is powered to move downward, set the voltage to 0 */
        if(getMeasurement() < ArmConstants.SOFT_LIMITS[0].getRadians() && output < 0) {
            setVoltage(0);
        }

        /** if the arm is above the limit and is powered to move upward, set the voltage to 0 */
        else if(getMeasurement() > ArmConstants.SOFT_LIMITS[1].getRadians() && output > 0) {
            setVoltage(0);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        if(SmartDashboard.getBoolean("Arm Debug On?", false)) {
            if (SmartDashboard.getBoolean("Reset Arm Encoders", false)) {
                resetEncoders();
            }
            SmartDashboard.putBoolean("Reset Arm Encoders", false);
            SmartDashboard.putBoolean("ThroughBore Is Connected", throughBore.isConnected());

            SmartDashboard.putNumber("Arm Goal Degrees", Units.radiansToDegrees(this.getController().getGoal().position));
        
            SmartDashboard.putData("Arm PID", this.getController());

            if(SmartDashboard.getBoolean("Break Mode Enabled", true) != breakModeEnabled) {
                this.breakModeEnabled = SmartDashboard.getBoolean("Break Mode Enabled", true);
                leftMotor.setNeutralMode(breakModeEnabled ? NeutralModeValue.Brake: NeutralModeValue.Coast);
                rightMotor.setNeutralMode(breakModeEnabled ? NeutralModeValue.Brake: NeutralModeValue.Coast);
            }
        }
        
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

        if(SmartDashboard.getBoolean("Break Mode Enabled", true) != breakModeEnabled) {
            this.breakModeEnabled = SmartDashboard.getBoolean("Break Mode Enabled", true);
            leftMotor.setNeutralMode(breakModeEnabled ? NeutralModeValue.Brake: NeutralModeValue.Coast);
            rightMotor.setNeutralMode(breakModeEnabled ? NeutralModeValue.Brake: NeutralModeValue.Coast);
        }

        SmartDashboard.putNumber("Arm Absolute Pos", getArmAbsolutePosition().getDegrees());
        SmartDashboard.putNumber("Arm Relative Pos", getAngleDegrees());

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(),
                ArmConstants.LEFT_MOTOR_INVERTED != ArmConstants.RIGHT_MOTOR_INVERTED));

        if(!manualMode) {
            armDistanceSetpoint(poseSupplier.get());
        }
        SmartDashboard.putData("Arm PID", this.getController());
    }

    public void setManualMode(boolean manualMode) {
        this.manualMode = manualMode;
    }

    public InstantCommand enableManualMode() {
        return new InstantCommand(() -> this.setManualMode(true));
    }

    public InstantCommand disableManualMode() {
        return new InstantCommand(() -> this.setManualMode(false));
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
