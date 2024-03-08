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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;

public class Arm extends PivotingArmBase {
    public final TalonFX leftMotor;
    public final TalonFX rightMotor;
    private final Supplier<Pose2d> poseSupplier;
    private final DutyCycleEncoder throughBore = new DutyCycleEncoder(0);
    private final DigitalInput magLimitSwitch = new DigitalInput(2);
    private boolean wasManuallyDisabled = false;
    private boolean manualMode = true;
    private boolean brakeModeEnabled = true;

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
        SmartDashboard.putBoolean("Arm Brake Mode", brakeModeEnabled);
        SmartDashboard.putBoolean("Arm Debug On?", false);
        configureMotors();
        // setGoal(poseDependantArmAngle(poseSupplier.get()).getRadians());
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

    public double getEncoderVelocityRadPerSec() {
        return Units.rotationsToRadians(leftMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public void setEncoderPosition(Rotation2d position) {
        leftMotor.setPosition(position.getRotations());
    }

    @Override
    public void configureMotors() {
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        leftMotor.setInverted(false);
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

    public void armStage() {
        setGoal(ArmConstants.STAGE_SETPOINT);
    }
    
    public InstantCommand armStageCommand() {
        return new InstantCommand(() -> {
            setManualMode(true);
            armStage();
        });
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
    //             pose.getTranslation().getDistance((DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue))
    //                     ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.toTranslation2d()
    //                     : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.toTranslation2d())
    //             + Math.toRadians(56)));
    // }

    public Rotation2d poseDependantArmAngle(Pose2d pose) {
        return Rotation2d.fromRadians(-Math.atan2(FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getZ(), Math.sqrt(
                        Math.pow(
                                pose.getX() - (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getX()), 2) +
                        Math.pow(
                                pose.getY() - (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getY()), 2)))
                         + Math.toRadians(72)).times(1.04);
    }

    public void armToDistanceSetpoint(Pose2d pose) {
        setGoal(poseDependantArmAngle(pose).getRadians());
    }

    @Override
    public void resetEncoders() {
        zeroThroughBore();
        resetToAbsolute();
    }

    public boolean getLimitSwitch() {
        return !magLimitSwitch.get();
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        /** SOFT LIMITS */
        /** output should be feedforward + calculated PID. */
        /** if the arm is below the limit and is powered to move downward, set the voltage to 0 */
        if((getMeasurement() < ArmConstants.SOFT_LIMITS[0].getRadians() && output < 0.) || (getMeasurement() > ArmConstants.SOFT_LIMITS[1].getRadians() && output > 0.) || (getLimitSwitch() && output < 0)) {
            setVoltage(0);
        }
        else {
            super.useOutput(output, setpoint);
        }
    }

    public Command chinUp() {
        return new InstantCommand(() -> {
            getController().setConstraints(ArmConstants.CLIMB_CONSTRAINTS);
            armRest();
        }, this);
    }

    @Override
    public void periodic() {
        if(!manualMode) {
            armToDistanceSetpoint(poseSupplier.get());
        }

        super.periodic(); // This is after the manual mode check because we want the new setpoint to be set before output is calculated + used.

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(),
                ArmConstants.LEFT_MOTOR_INVERTED != ArmConstants.RIGHT_MOTOR_INVERTED));

        if(SmartDashboard.getBoolean("Arm Debug On?", false)) {
            SmartDashboard.putBoolean("ThroughBore Is Connected", throughBore.isConnected());
            SmartDashboard.putNumber("Throughbore Offset", throughBore.getPositionOffset());
            SmartDashboard.putBoolean("Arm Limit Switch", getLimitSwitch());

            SmartDashboard.putNumber("Arm Absolute Pos", getArmAbsolutePosition().getDegrees());
        
            SmartDashboard.putData("Arm PID", this.getController());

            
            if(SmartDashboard.getBoolean("Reset Constraints", false) && getController().getConstraints().equals(ArmConstants.CLIMB_CONSTRAINTS)) {
                getController().setConstraints(ArmConstants.DEFAULT_CONSTRAINTS);
            }
            SmartDashboard.putBoolean("Reset Constraints", false);
        }

        if(SmartDashboard.getBoolean("Reset Arm Encoders", false)) {
            resetEncoders();
        }
        SmartDashboard.putBoolean("Reset Arm Encoders", false);

        SmartDashboard.putNumber("Arm Relative Pos", getAngleDegrees());
        SmartDashboard.putNumber("Arm Goal Degrees", Units.radiansToDegrees(this.getController().getGoal().position));
        
        if(SmartDashboard.getBoolean("Arm Brake Mode", true) != brakeModeEnabled) {
            this.brakeModeEnabled = SmartDashboard.getBoolean("Arm Brake Mode", true);
            leftMotor.setNeutralMode(brakeModeEnabled ? NeutralModeValue.Brake: NeutralModeValue.Coast);
            rightMotor.setNeutralMode(brakeModeEnabled ? NeutralModeValue.Brake: NeutralModeValue.Coast);
        }
        
        if(getLimitSwitch()) {
            setEncoderPosition(new Rotation2d());
        }

        // if (!throughBore.isConnected() || !SmartDashboard.getBoolean("Arm Enabled", true)) {
        //     this.disable();
        //     this.stop();
        //     this.wasManuallyDisabled = true;
        //     SmartDashboard.putBoolean("Arm Enabled", false);
        // }

        if(SmartDashboard.getBoolean("Arm Enabled", true) && wasManuallyDisabled) {
            this.enable();
            this.wasManuallyDisabled = false;
        }
        else if(!SmartDashboard.getBoolean("Arm Enabled", true) && !wasManuallyDisabled) {
            wasManuallyDisabled = true;
            this.disable();
        }
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(),
                ArmConstants.LEFT_MOTOR_INVERTED != ArmConstants.RIGHT_MOTOR_INVERTED));
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

    public boolean inAutoAimSpace(Pose2d pose) {
        boolean withinX;
        boolean withinY;
        if(DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Red)) {
            withinX = pose.getX() > FieldConstants.RED_AUTO_TRACK_BOUNDS[0].getX() && pose.getX() < FieldConstants.RED_AUTO_TRACK_BOUNDS[1].getX();
            withinY = pose.getY() > FieldConstants.RED_AUTO_TRACK_BOUNDS[0].getY() && pose.getY() < FieldConstants.RED_AUTO_TRACK_BOUNDS[1].getY();
        }
        else {
            withinX = pose.getX() > FieldConstants.BLUE_AUTO_TRACK_BOUNDS[0].getX() && pose.getX() < FieldConstants.BLUE_AUTO_TRACK_BOUNDS[1].getX();
            withinY = pose.getY() > FieldConstants.BLUE_AUTO_TRACK_BOUNDS[0].getY() && pose.getY() < FieldConstants.BLUE_AUTO_TRACK_BOUNDS[1].getY();
        }
        return withinX && withinY;
    }
    
    public void travelSetpoint() {
        this.setGoal(Units.degreesToRadians(0));
    }

    public Command travelSetpointCommand() {
        return enableManualMode().andThen(new InstantCommand(() -> travelSetpoint()));
    }

    public Rotation2d getRawAbsolutePosition() {
        double abs = throughBore.getAbsolutePosition();
        return Rotation2d.fromRotations(abs);
    }

    public Rotation2d getZeroedArmAbsolutePosition() {
        Rotation2d abs = getRawAbsolutePosition().minus(Rotation2d.fromRotations(throughBore.getPositionOffset()));
        SmartDashboard.putNumber("Abs in Rotations", abs.getRotations());
    
        return abs;//abs.plus(Rotation2d.fromRotations(abs.getRotations() < 0 ? abs.getRotations() + 0.25 : abs.getRotations()));
    }

    public void zeroThroughBore() {
        throughBore.setPositionOffset(getRawAbsolutePosition().getRotations());
    }

    public Rotation2d getArmAbsolutePosition() {
        return getZeroedArmAbsolutePosition().times(ArmConstants.GEARING_ARM_TO_THROUGHBORE);
    }

    public Rotation2d throughBoreToMotor(Rotation2d throughBoreRotations) {
        return throughBoreRotations.times(ArmConstants.GEARING_THROUGHBORE_TO_MOTOR);
    }

    public void resetToAbsolute() {
        setEncoderPosition(throughBoreToMotor(getRawAbsolutePosition()));
    }
}
