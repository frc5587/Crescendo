package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteDetector;
import frc.robot.subsystems.Swerve;

public class AimToNote extends Command {
    private final NoteDetector noteDetector;
    private final Swerve swerve;
    private final BooleanSupplier hasNoteSupplier;
    private final DoubleSupplier armAngleSupplier;
    private double endTime = 0.;

    public AimToNote(NoteDetector noteDetector, Swerve swerve, BooleanSupplier hasNoteSupplier, DoubleSupplier armAngleSupplier) {
        this.noteDetector = noteDetector;
        this.swerve = swerve;
        this.hasNoteSupplier = hasNoteSupplier;
        this.armAngleSupplier = armAngleSupplier;
    }

    @Override
    public void initialize() {
        endTime = 0;
    }

    @Override
    public void execute() {
        if(noteDetector.hasTarget()) {
            double fwdMPS = 50 / noteDetector.getDistanceToNoteMeters(Rotation2d.fromRadians(armAngleSupplier.getAsDouble()));
            double strafeMPS = noteDetector.getRotationToNote(Rotation2d.fromRadians(armAngleSupplier.getAsDouble())).getRadians() * 1.5;
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
