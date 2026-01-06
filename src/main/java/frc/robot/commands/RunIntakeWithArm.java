package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class RunIntakeWithArm extends Command {
    private final Intake intake;
    private final Arm arm;
    private final BooleanSupplier shooterSpunUpSupplier, spunUpOverrideSupplier;

    public RunIntakeWithArm(Intake intake, Arm arm, BooleanSupplier shooterSpunUpSupplier, BooleanSupplier spunUpOverrideSupplier) {
        this.intake = intake;
        this.arm = arm;
        this.shooterSpunUpSupplier = shooterSpunUpSupplier;
        this.spunUpOverrideSupplier = spunUpOverrideSupplier;
    }

    @Override
    public void initialize() {
        updateArmPosition();
    }

    @Override
    public void execute() {
        intake.forward();
        updateArmPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if(arm.getController().getGoal().position != ArmConstants.AMP_SETPOINT) {            
            arm.armTravel();
        }
        intake.stop();
    }

    private void updateArmPosition() {
        if (arm.getController().getGoal().position == ArmConstants.AMP_SETPOINT) {
            return;
        }

        boolean shooterRevving = shooterSpunUpSupplier.getAsBoolean() || spunUpOverrideSupplier.getAsBoolean();
        // Drive the arm down until we have a note, then return to travel when intaking.
        if (!intake.getLimitSwitch()) {
            arm.armBottom();
        } else if (!shooterRevving) {
            arm.armTravel();
        }
    }
}
