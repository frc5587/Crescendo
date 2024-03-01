package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
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
        if(!shooterSpunUpSupplier.getAsBoolean()) {
            arm.armRest();
        }
    }

    @Override
    public void execute() {
        if(intake.getLimitSwitch() && !shooterSpunUpSupplier.getAsBoolean()) {
            intake.stop();
        }
        else {
            intake.forward();
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.travelSetpoint();
        intake.stop();
    }
}
