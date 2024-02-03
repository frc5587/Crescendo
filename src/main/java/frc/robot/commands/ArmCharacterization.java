package frc.robot.commands;

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
        return arm.getEncoderPosition();
    }

    @Override
    public double getMechanismVelocity() {
        return arm.getEncoderVelocity();
    }

    @Override
    public double getMechanismAcceleration() {
        return arm.leftMotor.getAcceleration().getValueAsDouble();
    }
}
