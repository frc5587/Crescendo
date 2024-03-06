package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmCharacterization extends CharacterizationBase {
    private final Arm arm;

    public ArmCharacterization(Arm arm) {
        super(MechanismType.Rotational, arm);
        this.arm = arm;
    }

    @Override
    public void setVoltage(double volts) {
        arm.setVoltage(volts);
    }

    @Override
    public double getMotorVoltage() {
        return arm.leftMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getMechanismPosition() {
        return arm.getPosition().getRotations();
    }

    @Override
    public double getMechanismVelocity() {
        return arm.getVelocityRotationsPerSecond();
    }

    @Override
    public double getMechanismAcceleration() {
        return arm.leftMotor.getAcceleration().getValueAsDouble() / ArmConstants.GEARING_MOTOR_TO_ARM;
    }

    @Override
    public Command preRoutine() {
        return new InstantCommand(arm::disable);
    }
    
    @Override
    public Command postRoutine() {
        return new InstantCommand(arm::enable);
    }
}
