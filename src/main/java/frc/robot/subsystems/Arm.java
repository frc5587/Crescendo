package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import org.frc5587.lib.subsystems.PivotingArmBase;

public class Arm extends PivotingArmBase {
 public static CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
 public static CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);  

 private final DigitalInput frontLimitSwitch = new DigitalInput(ArmConstants.SWITCH_PORTS[0]);
 private final DigitalInput rearLimitSwitch = new DigitalInput(ArmConstants.SWITCH_PORTS[1]);
 private final DutyCycleEncoder throughBore = new DutyCycleEncoder(1);


 public static PivotingArmConstants constants = new PivotingArmConstants (
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
    throughBore.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0); //placeholder values

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
        if(SmartDashboard.getBoolean("Reset Encoders", false)) {
            resetEncoders();
        }
        SmartDashboard.putBoolean("Reset Encoders", false);

        if(!throughBore.isConnected()) {
            this.disable();
         this.stop();
     }
    }


    public double setAbsolutePosition(){
        return throughBore.get();
    }
    public void zeroThroughBore(){
        double absolutePosition = setAbsolutePosition();
        throughBore.setPositionOffset(absolutePosition);
    }
}
