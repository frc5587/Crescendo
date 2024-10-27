package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;

public class FullClimb extends Command {
    private final Climb climb;
    private final Arm arm;
    private final Shooter shooter;

    public FullClimb(Climb climb, Arm arm, Shooter shooter) {
        this.climb = climb;
        this.arm = arm;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.stop();
        climb.hookMiddle();
    }

    @Override
    public void execute() {
        if(climb.getController().atGoal() && climb.getController().getGoal().position == ClimbConstants.MIDDLE_POSITION) {
            arm.armStage();
            climb.hookBottom();
        }
    }
}
