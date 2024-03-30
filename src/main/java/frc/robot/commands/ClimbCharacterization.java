package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;

public class ClimbCharacterization extends CharacterizationBase {
    private final Climb climb;

    public ClimbCharacterization(Climb climb) {
        super(MechanismType.Linear, climb);
        this.climb = climb;
    }

    @Override
    public void setVoltage(double volts) {
        climb.setVoltage(volts);
    }

    @Override
    public double getMotorVoltage() {
        return climb.getVoltage();
    }

    @Override
    public double getMechanismPosition() {
        return climb.getMeasurement();
    }

    @Override
    public double getMechanismVelocity() {
        return climb.getLinearVelocity();
    }

    @Override
    public double getMechanismAcceleration() {
        return climb.leftMotor.getEncoder().getVelocity() / ArmConstants.GEARING_MOTOR_TO_ARM;
    }

    @Override
    public Command preRoutine() {
        return new InstantCommand(climb::disable);
    }
    
    @Override
    public Command postRoutine() {
        // return new InstantCommand(climb::enable);
        return Commands.none();
    }
}
