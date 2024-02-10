package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmDistancePose extends Command {
    private final Arm arm;
    private final Supplier<Pose2d> poseSupplier;
    private boolean wasInterrupted;

    public ArmDistancePose(Arm arm, Supplier<Pose2d> poseSupplier) {
        addRequirements(arm);
        this.arm = arm;
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void execute() {
        arm.armDistanceSetpoint(poseSupplier.get());
    }
}
