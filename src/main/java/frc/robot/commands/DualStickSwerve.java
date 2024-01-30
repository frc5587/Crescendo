package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Swerve;

public class DualStickSwerve extends Command {    
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldRelativeSup;

    public DualStickSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier fieldRelativeSup) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldRelativeSup = fieldRelativeSup;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Translation2d translation = new Translation2d(translationSup.getAsDouble(), -strafeSup.getAsDouble()).times(DrivetrainConstants.MAX_SPEED);
        double rotation = rotationSup.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_VELOCITY;
        
        swerve.drive(translation, rotation,
                fieldRelativeSup.getAsBoolean(),
                true);
    }
}