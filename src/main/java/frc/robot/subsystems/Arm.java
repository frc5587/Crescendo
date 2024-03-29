package frc.robot.subsystems;

import java.time.Instant;
import java.util.Hashtable;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.frc5587.lib.subsystems.PivotingArmBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

public class Arm extends PivotingArmBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final Supplier<Pose2d> poseSupplier;
    private BooleanSupplier limitSwitchSupplier;
    private final DutyCycleEncoder throughBore = new DutyCycleEncoder(3);
    private final DigitalInput magLimitSwitch = new DigitalInput(0);
    private boolean wasManuallyDisabled = false;
    private boolean manualMode = true;
    private boolean brakeModeEnabled = true;
    private TimeInterpolatableBuffer<Double> voltageBuffer = TimeInterpolatableBuffer.createDoubleBuffer(0.2);
    private TimeInterpolatableBuffer<Double> positionBuffer = TimeInterpolatableBuffer.createDoubleBuffer(0.2);
    private double timeStopped = -1;
    private Hashtable<Double, Rotation2d> distanceToAngleTable = new Hashtable<Double, Rotation2d>();

    public static PivotingArmConstants constants = new PivotingArmConstants(ArmConstants.GEARING_MOTOR_TO_ARM,
            new Rotation2d(), ArmConstants.SOFT_LIMITS, ArmConstants.ZERO_OFFSET, ArmConstants.PID, ArmConstants.FF);
        

    public Arm(TalonFX leftMotor, TalonFX rightMotor, Supplier<Pose2d> poseSupplier, BooleanSupplier limitSwitchSupplier) {
        super(constants, leftMotor);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.poseSupplier = poseSupplier;
        this.limitSwitchSupplier = limitSwitchSupplier;
        throughBore.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
        resetToAbsolute();
        getController().setTolerance(Units.degreesToRadians(1));
        enable();
        armTravel();
        SmartDashboard.putBoolean("Arm Enabled", isEnabled());
        SmartDashboard.putBoolean("Arm Brake Mode", brakeModeEnabled);
        SmartDashboard.putBoolean("Arm Debug On?", false);
        configureMotors();
        SmartDashboard.putNumber("RadiansPerMeeter", ShooterConstants.RadiansPerMeter);
        // setGoal(poseDependantArmAngle(poseSupplier.get()).getRadians());
        SmartDashboard.putNumber("Manual Arm Angle", 0.0);
        distanceToAngleTable.put(1.3, Rotation2d.fromDegrees(7.25));
        distanceToAngleTable.put(1.8, Rotation2d.fromDegrees(14.0));
        distanceToAngleTable.put(2.3, Rotation2d.fromDegrees(22.3));
        distanceToAngleTable.put(2.8, Rotation2d.fromDegrees(26.7));
        distanceToAngleTable.put(3.3, Rotation2d.fromDegrees(30.4));
    }

    public Arm(Supplier<Pose2d> poseSupplier, BooleanSupplier limitSwitchSupplier) {
        this(new TalonFX(ArmConstants.LEFT_MOTOR_ID, "canivore"), new TalonFX(ArmConstants.RIGHT_MOTOR_ID, "canivore"), poseSupplier, limitSwitchSupplier); 
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

    public void armFerry() {
        setGoal(ArmConstants.FERRY_SETPOINT);
    }

    public InstantCommand armFerryCommand() {
        return new InstantCommand(() -> {
            setManualMode(true);
            armFerry();
        });
    }

    // public Rotation2d poseDependantArmAngle(Pose2d pose) {
    //     return Rotation2d.fromRadians(-Math.atan(FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getZ() /
    //             pose.getTranslation().getDistance((DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue))
    //                     ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.toTranslation2d()
    //                     : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.toTranslation2d())
    //             + Math.toRadians(56)));
    // }

    public double getShooterHeightMeters() {
        return (ArmConstants.ARM_LENGTH_METERS * Math.sin(getAngleRadians())) + ArmConstants.SHOOTER_HEIGHT_METERS;
    }

    public Rotation2d poseDependantArmAngle(Pose2d pose) {
        double distance = Math.sqrt(
                        Math.pow(
                                pose.getX() - (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getX()), 2) +
                        Math.pow(
                                pose.getY() - (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getY()),
                                2));

        SmartDashboard.putNumber("Arm Distance", distance);

        return Rotation2d.fromRadians(
                -Math.atan2(FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getZ() - getShooterHeightMeters(), distance) + Math.toRadians(72))
                // .times(1.04)[]\
                
                .minus(new Rotation2d((distance-1.3) * SmartDashboard.getNumber("RadiansPerMeeter", ShooterConstants.RadiansPerMeter)));
    }

    public Rotation2d logBasedArmAngle(Pose2d pose) {
        double distance = Math.sqrt(
                Math.pow(
                        pose.getX() - (DriverStation.getAlliance().get().equals(Alliance.Blue)
                                ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
                                : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getX()), 2) +
                Math.pow(
                        pose.getY() - (DriverStation.getAlliance().get().equals(Alliance.Blue)
                                ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY()
                                : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getY()), 2)
        );
        SmartDashboard.putNumber("Arm Distance", distance);

        // https://www.desmos.com/calculator/rqgtniidqa
        return Rotation2d.fromRadians(1.03433 * Math.log10(distance) - Units.degreesToRadians(2.5));
    }

    public Rotation2d lineBasedArmAngle(Pose2d pose) {
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

        return Rotation2d.fromRadians((0.205949 * distance) - 0.122348);
    }

    public Rotation2d interpolationArmAngle(Pose2d pose) {
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
        
        return Rotation2d.fromRadians(1.03433 * Math.log10(distance));
    }

    public void armToDistanceSetpoint(Pose2d pose) {
        // setGoal(poseDependantArmAngle(pose).getRadians());
        setGoal(logBasedArmAngle(pose).getRadians());
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
        if ((getMeasurement() < ArmConstants.SOFT_LIMITS[0].getRadians() && output < 0.)
                || (getMeasurement() > ArmConstants.SOFT_LIMITS[1].getRadians() && output > 0.)
                || (getLimitSwitch() && output < 0)) {
            setVoltage(0);
        }
        // else if(Math.abs(positionBuffer.getSample(Timer.getFPGATimestamp() - 0.18).orElseGet(() -> 0.0).doubleValue()
        //         - positionBuffer.getSample(Timer.getFPGATimestamp() - 0.02).orElseGet(() -> 0.0).doubleValue()) <= Units.degreesToRadians(0.3) // if the position 1 period ago and the position 9 periods ago are roughly the same,
        //         && positionBuffer.getSample(Timer.getFPGATimestamp() - 0.02).orElseGet(() -> 0.0).doubleValue() <= Units.degreesToRadians(0.3) // if the position is less than 0.3 degrees,
        //         && (Math.abs(voltageBuffer.getSample(Timer.getFPGATimestamp() - 0.18).orElseGet(() -> 0.0).doubleValue()) > 0.5) // if the robot has tried to move the arm in the last 9 periods,
        //         && getLimitSwitch()) { // and if the limit switch is pressed,
        //     setVoltage(0);
        // }
        else {
            super.useOutput(output, setpoint);
        }
        voltageBuffer.addSample(Timer.getFPGATimestamp(), output);
    }

    public Command chinUp() {
        return new InstantCommand(() -> {
            getController().setConstraints(ArmConstants.CLIMB_CONSTRAINTS);
            armStage();
        }, this);
    }

    @Override
    public void periodic() {
        if(!manualMode) {
            armToDistanceSetpoint(poseSupplier.get());
        }
        SmartDashboard.putNumber("Shooter Height (m)", getShooterHeightMeters());
        // if(!limitSwitchSupplier && MathSharedStore.getTimestamp() > limitSwitchDelayTime) {
        //     travelSetpointCommand();
        // }

        super.periodic(); // This is after the manual mode check because we want the new setpoint to be set before output is calculated + used.

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(),
                ArmConstants.LEFT_MOTOR_INVERTED != ArmConstants.RIGHT_MOTOR_INVERTED));
        
        if(DriverStation.isEnabled()) {
            positionBuffer.addSample(Timer.getFPGATimestamp(), getMeasurement());
            SmartDashboard.putNumber("Left Volts", leftMotor.getMotorVoltage().getValue());
        }

        else {
            positionBuffer.clear();
            voltageBuffer.clear();
        }

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

        SmartDashboard.putBoolean("In Restricted Zone?", isPoseWithinTriangle(poseSupplier.get()));

        if(SmartDashboard.getBoolean("Reset Arm Encoders", false)) {
            resetEncoders();
        }
        SmartDashboard.putBoolean("Reset Arm Encoders", false);

        SmartDashboard.putNumber("Arm Relative Pos", getAngleDegrees());
        SmartDashboard.putNumber("Arm Goal Degrees", Units.radiansToDegrees(this.getController().getGoal().position));
        SmartDashboard.putBoolean("At Goal", getController().atGoal());
        
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

    public boolean inRestrictedSpace(Pose2d pose) {
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

    // Method to check if pose is within the equilateral triangle
    public boolean isPoseWithinTriangle(Pose2d currentPose) {
        Translation2d t1 = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? FieldConstants.BLUE_RESTRICTED_SPACE_BOUNDS[0] : FieldConstants.RED_RESTRICTED_SPACE_BOUNDS[0];
        Translation2d t2 = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? FieldConstants.BLUE_RESTRICTED_SPACE_BOUNDS[1] : FieldConstants.RED_RESTRICTED_SPACE_BOUNDS[1];
        Translation2d t3 = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? FieldConstants.BLUE_RESTRICTED_SPACE_BOUNDS[2] : FieldConstants.RED_RESTRICTED_SPACE_BOUNDS[2];
        Translation2d currentTranslation = currentPose.getTranslation();
        double side1 = Math.abs(crossProduct(t2.minus(t1), currentTranslation.minus(t1)));
        double side2 = Math.abs(crossProduct(t3.minus(t2), currentTranslation.minus(t2)));
        double side3 = Math.abs(crossProduct(t1.minus(t3), currentTranslation.minus(t3)));

        // If the sum of the areas of the three triangles formed by the current pose and the triangle's vertices equals the area of the equilateral triangle, it's inside.
        double triangleArea = Math.abs(crossProduct(t2.minus(t1), t3.minus(t1))) / 2.0;
        double currentArea = side1 + side2 + side3;
        return currentArea <= triangleArea; // Tolerance for double comparison
    }

    // Helper method to calculate the cross product of two 2D vectors
    private double crossProduct(Translation2d a, Translation2d b) {
        return a.getX() * b.getY() - a.getY() * b.getX();
    }
    
    public void armTravel() {
        this.setGoal(Units.degreesToRadians(6));
    }

    public Command armTravelCommand() {
        return enableManualMode().andThen(new InstantCommand(() -> armTravel()));
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
