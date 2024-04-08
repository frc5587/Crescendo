package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteDetector;
import frc.robot.subsystems.Swerve;

public class AimToNote extends Command {
    private final NoteDetector noteDetector;
    private final Swerve swerve;
    private final BooleanSupplier hasNoteSupplier;
    private double endTime = 0.;

    public AimToNote(NoteDetector noteDetector, Swerve swerve, BooleanSupplier hasNoteSupplier) {
        this.noteDetector = noteDetector;
        this.swerve = swerve;
        this.hasNoteSupplier = hasNoteSupplier;
    }

    @Override
    public void initialize() {
        endTime = 0;
    }

    @Override
    public void execute() {
        if(noteDetector.hasTarget()) {
            double strafeMPS = noteDetector.getRotationToNote().getRadians() * 3.5;
            double fwdMPS = 15 / noteDetector.getDistanceToNoteMeters();
            swerve.drive(new Translation2d(fwdMPS, strafeMPS), 0, false, false);
        }
        else if(!noteDetector.hasTarget() && endTime == 0.) {
            endTime = Timer.getFPGATimestamp() + 0.5;
            swerve.drive(new Translation2d(-0.5, 0.), 0, false, false);
        }
        else if(!noteDetector.hasTarget() && Timer.getFPGATimestamp() > endTime) {
            swerve.drive(new Translation2d(-0.5, 0.), 0, false, false);
        }
        else {
            end(false);
        }
    }

    @Override
    public boolean isFinished() {
        return hasNoteSupplier.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
