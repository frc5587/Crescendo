package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class PositionArm extends Command {
    private final Arm arm;
    private final Supplier<Pose2d> robotPoseSupplier;
    
    public PositionArm(Arm arm, Supplier<Pose2d> robotPoseSupplier) {
        addRequirements(arm);
        this.arm = arm;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override public void execute() {
        arm.setGoal(Math.atan(3.267 / Math.sqrt(Math.pow(
            robotPoseSupplier.get().getX(), 2) + Math.pow(robotPoseSupplier.get().getY(), 2))));
    }
}
