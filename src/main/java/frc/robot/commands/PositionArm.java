package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Arm;

public class PositionArm extends Command {
    private final Arm arm;
    private final Supplier<Pose2d> robotPoseSupplier;

    public PositionArm(Arm arm, Supplier<Pose2d> robotPoseSupplier) {
        addRequirements(arm);
        this.arm = arm;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void execute() {
        arm.setGoal(-Math.atan(FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getZ() / Math.sqrt(
                        Math.pow(
                                robotPoseSupplier.get().getX() - (DriverStation.getAlliance().get().equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getX()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getX()), 2) +
                        Math.pow((robotPoseSupplier.get().getY()
                                - (DriverStation.getAlliance().get().equals(Alliance.Blue)
                                        ? FieldConstants.BLUE_SPEAKER_OPENING_TRANSLATION.getY()
                                        : FieldConstants.RED_SPEAKER_OPENING_TRANSLATION.getY())),
                                2))) + Math.toRadians(50));
    }
}
