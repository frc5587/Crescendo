package frc.robot.commands;

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
        return arm.getAngleDegrees() / 360;
    }

    @Override
    public double getMechanismVelocity() {
        return arm.getEncoderVelocity() / ArmConstants.GEARING_MOTOR_TO_ARM;
    }

    @Override
    public double getMechanismAcceleration() {
        return arm.leftMotor.getAcceleration().getValueAsDouble() / ArmConstants.GEARING_MOTOR_TO_ARM;
    }
}
