package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmToIntake extends Command {
    private final Arm arm;
    private boolean initialRunAttempted, secondRunAttempted;

    public ArmToIntake(Arm arm) {
        addRequirements(arm);
        this.arm = arm;
    }

    @Override
    public void initialize() {
        if(!secondRunAttempted) {
            initialRunAttempted = true;
            arm.armRest();
        }
        else {
            initialRunAttempted = true;
            secondRunAttempted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return initialRunAttempted && secondRunAttempted;
    }
}
