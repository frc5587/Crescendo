package frc.robot.subsystems;

import java.util.function.Supplier;

import org.frc5587.lib.subsystems.PivotingArmBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;

public class Arm extends PivotingArmBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final Supplier<Pose2d> poseSupplier;
    private final DigitalInput magLimitSwitch = new DigitalInput(0);
    private boolean manualMode = true;
    private boolean brakeModeEnabled = true;

    public static PivotingArmConstants constants = new PivotingArmConstants(ArmConstants.GEARING_MOTOR_TO_ARM,
            new Rotation2d(), ArmConstants.SOFT_LIMITS, ArmConstants.ZERO_OFFSET, ArmConstants.PID, ArmConstants.FF);
        

    public Arm(TalonFX leftMotor, TalonFX rightMotor, Supplier<Pose2d> poseSupplier) {
        super(constants, leftMotor);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.poseSupplier = poseSupplier;
        configureMotors();
        resetEncoders();
        getController().setTolerance(Units.degreesToRadians(0.1));
        enable();
        armTravel();
        SmartDashboard.putBoolean("Arm Enabled", isEnabled());
        SmartDashboard.putBoolean("Arm Brake Mode", brakeModeEnabled);
        SmartDashboard.putBoolean("Arm Debug On?", false);
        SmartDashboard.putNumber("Manual Arm Angle Degrees", 15.0);
        // if(DriverStation.isAutonomous()) {
        //     setGoal(Units.degreesToRadians(75.)); // TODO if using w/FMS, comment out conditional
        // }
    }

    public Arm(Supplier<Pose2d> poseSupplier) {
        this(new TalonFX(ArmConstants.LEFT_MOTOR_ID, "canivore"), new TalonFX(ArmConstants.RIGHT_MOTOR_ID, "canivore"), poseSupplier); 
    }

    public double getShuffleboardArmAngleRadians() {
        double clampedArmAngle = MathUtil.clamp(SmartDashboard.getNumber("Manual Arm Angle Degrees", ArmConstants.BOTTOM_LIMIT), ArmConstants.BOTTOM_LIMIT, ArmConstants.TOP_LIMIT);
        return Units.degreesToRadians(clampedArmAngle);
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

    public void setManualMode(boolean manualMode) {
        this.manualMode = manualMode;
    }

    public InstantCommand enableManualMode() {
        return new InstantCommand(() -> this.setManualMode(true));
    }

    public InstantCommand disableManualMode() {
        return new InstantCommand(() -> this.setManualMode(false));
    }
    
    public void armTravel() {
        setGoal(ArmConstants.TRAVEL_SETPOINT);
    }

    public Command armTravelCommand() {
        return enableManualMode().andThen(new InstantCommand(() -> armTravel()));
    }

    public void armStage() {
        setGoal(ArmConstants.CLIMB_SETPOINT);
    }
    
    public Command armStageCommand() {
        return enableManualMode().andThen(new InstantCommand(() -> armStage()));
    }

    public void armAmp() {
        setGoal(ArmConstants.AMP_SETPOINT);
    }
    
    public Command armAmpCommand() {
        return enableManualMode().andThen(new InstantCommand(() -> armAmp()));
    }

    public void armBottom() {
        setGoal(ArmConstants.BOTTOM_SETPOINT);
    }

    public Command armBottomCommand() {
        return enableManualMode().andThen(new InstantCommand(() -> armBottom()));
    }

    public void armFerry() {
        setGoal(ArmConstants.FERRY_SETPOINT);
    }

    public Command armFerryCommand() {
        return enableManualMode().andThen(new InstantCommand(() -> armFerry()));
    }

    public InstantCommand shuffleBoardArmCommand() {
        return new InstantCommand(() -> {
            setManualMode(true);
            setGoal(getShuffleboardArmAngleRadians());
        });
    }

    public Rotation2d logBasedArmAngle(Pose2d pose) {
        double distance = Math.sqrt(
                Math.pow(
                        pose.getX() - (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                                ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
                                : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getX()), 2) +
                Math.pow(
                        pose.getY() - (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue)
                                ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY()
                                : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getY()), 2)
        );

        // https://www.desmos.com/calculator/rqgtniidqa
        return Rotation2d.fromRadians(1.03433 * Math.log10(distance) - Units.degreesToRadians(2.5));
    }

    public void armToDistanceSetpoint(Pose2d pose) {
        setGoal(logBasedArmAngle(pose).getRadians());
    }

    @Override
    public void resetEncoders() {
        setEncoderPosition(new Rotation2d());
    }

    public boolean getLimitSwitch() {
        return !magLimitSwitch.get();
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        /** SOFT LIMITS */
        /** output should be feedforward + calculated PID. */
        /** if the arm is below the limit and is powered to move downward, set the voltage to 0 */
        /** if the arm is above the limit and is powered to move upward, set the voltage to 0 */
        if ((getMeasurement() < ArmConstants.SOFT_LIMITS[0].getRadians() && output < 0.)
                || (getMeasurement() > ArmConstants.SOFT_LIMITS[1].getRadians() && output > 0.)
                || (getLimitSwitch() && output < 0)) {
            setVoltage(0);
        }
        else {
            super.useOutput(output, setpoint);
        }
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
            SmartDashboard.putData("Arm PID", this.getController());
            
            if(SmartDashboard.getBoolean("Reset Constraints", false) && getController().getConstraints().equals(ArmConstants.CLIMB_CONSTRAINTS)) {
                getController().setConstraints(ArmConstants.DEFAULT_CONSTRAINTS);
            }
            SmartDashboard.putBoolean("Reset Constraints", false);
        }
                
        SmartDashboard.putBoolean("Arm Limit Switch", getLimitSwitch());

        if(SmartDashboard.getBoolean("Reset Arm Encoders", false)) {
            resetEncoders();
        }
        SmartDashboard.putBoolean("Reset Arm Encoders", false);

        SmartDashboard.putNumber("Arm Relative Pos", getAngleDegrees());
        SmartDashboard.putNumber("Arm Goal Degrees", Units.radiansToDegrees(this.getController().getGoal().position));
        SmartDashboard.putBoolean("Arm At Goal", getController().atGoal());
        
        if(SmartDashboard.getBoolean("Arm Brake Mode", true) != brakeModeEnabled) {
            this.brakeModeEnabled = SmartDashboard.getBoolean("Arm Brake Mode", true);
            leftMotor.setNeutralMode(brakeModeEnabled ? NeutralModeValue.Brake: NeutralModeValue.Coast);
            rightMotor.setNeutralMode(brakeModeEnabled ? NeutralModeValue.Brake: NeutralModeValue.Coast);
        }
        
        if(getLimitSwitch()) {
            setEncoderPosition(new Rotation2d());
        }

        if(SmartDashboard.getBoolean("Arm Enabled", true) && !isEnabled()) {
            this.enable();
        }
        else if(!SmartDashboard.getBoolean("Arm Enabled", true) && isEnabled()) {
            this.disable();
        }
    }
}
