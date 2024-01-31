package frc.robot.subsystems;

import org.frc5587.lib.subsystems.PivotingArmBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

public class Arm extends PivotingArmBase {
    public static CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    public static CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput frontLimitSwitch = new DigitalInput(ArmConstants.SWITCH_PORTS[0]);
    private final DigitalInput rearLimitSwitch = new DigitalInput(ArmConstants.SWITCH_PORTS[1]);
    private final DutyCycleEncoder throughBore = new DutyCycleEncoder(1);
    private State lastGoal = new State();
    private Pose3d armPose = new Pose3d();

    public static PivotingArmConstants constants = new PivotingArmConstants(
            ArmConstants.GEARING,
            1,
            0,
            ArmConstants.SOFT_LIMITS,
            (int) ArmConstants.ZERO_OFFSET,
            ArmConstants.ENCODER_CPR,
            ArmConstants.PID,
            ArmConstants.FF

    );

    public Arm(CANSparkMax leftMotor, CANSparkMax rightMotor) {
        super("arm", constants, leftMotor);
        enable();
        resetEncoders();
        setGoal(0);
        // throughBore.setDutyCycleRange(1./1024., 1023./1024.); placeholder values
        this.lastGoal = this.getController().getGoal();
    }

    public Arm() {
        this(leftMotor, rightMotor);
    }

    public DigitalInput getFrontLimitSwitch() {
        return frontLimitSwitch;
    }

    public DigitalInput getRearLimitSwitch() {
        return rearLimitSwitch;
    }

    @Override
    public double getEncoderPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        leftMotor.getEncoder().setPosition(position);
    }

    @Override
    public void configureMotors() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setInverted(ArmConstants.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(ArmConstants.RIGHT_MOTOR_INVERTED);

        leftMotor.setSmartCurrentLimit(ArmConstants.STALL_LIMIT, ArmConstants.FREE_LIMIT);
        rightMotor.setSmartCurrentLimit(ArmConstants.STALL_LIMIT, ArmConstants.FREE_LIMIT);

        rightMotor.follow(leftMotor);
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

    @Override
    public void simulationPeriodic() {
        armPose = new Pose3d(0, -0.625, 0.22, new Rotation3d(getController().getGoal().position, 0, 0));
        Logger.recordOutput("Arm Pose", armPose);
    }
}