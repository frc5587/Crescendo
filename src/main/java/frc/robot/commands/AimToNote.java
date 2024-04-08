package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteDetector;
import frc.robot.subsystems.Swerve;

public class AimToNote extends Command {
    private final NoteDetector noteDetector;
    private final Swerve swerve;

    public AimToNote(NoteDetector noteDetector, Swerve swerve) {
        this.noteDetector = noteDetector;
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        double omegaRPS = noteDetector.getRotationToNote().getRadians() * 3;
        swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, omegaRPS));
    }
}
