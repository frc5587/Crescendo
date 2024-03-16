package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class RunIntakeWithArm extends Command {
    private final Intake intake;
    private final Arm arm;
    private final BooleanSupplier shooterSpunUpSupplier;

    public RunIntakeWithArm(Intake intake, Arm arm, BooleanSupplier shooterSpunUpSupplier) {
        this.intake = intake;
        this.arm = arm;
        this.shooterSpunUpSupplier = shooterSpunUpSupplier;
    }

    @Override
    public void initialize() {
        if(!shooterSpunUpSupplier.getAsBoolean() && arm.getController().getGoal().position != ArmConstants.AMP_SETPOINT) {
            arm.armRest();
        }
    }

    @Override
    public void execute() {
        if(shooterSpunUpSupplier.getAsBoolean() || arm.getController().getGoal().position == ArmConstants.AMP_SETPOINT || !intake.getLimitSwitch()) {
            intake.forward();
        }
        else {
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(arm.getController().getGoal().position != ArmConstants.AMP_SETPOINT) {            
            arm.armTravel();
        }
        intake.stop();
    }
}
