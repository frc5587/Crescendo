// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DualStickSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    private final Swerve swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();

    private final CommandXboxController xbox = new DeadbandCommandXboxController(0);
    private final CommandXboxController xbox2 = new DeadbandCommandXboxController(1);

    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getLeftY, xbox::getLeftX,
            () -> -xbox.getRightX(), () -> xbox.rightBumper().negate().getAsBoolean());

    public RobotContainer() {
        swerve.setDefaultCommand(driveCommand);
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        xbox2.rightBumper().whileTrue(new InstantCommand(intake::forward)).onFalse(new InstantCommand(intake::stop));
        xbox2.rightTrigger().whileTrue(new InstantCommand(shooter::forward)).onFalse(new InstantCommand(shooter::stop));
        xbox2.leftTrigger().whileTrue(new InstantCommand(shooter::backward)).onFalse(new InstantCommand(shooter::stop));
        xbox2.x().onTrue(new InstantCommand(arm::armSpeaker));
        xbox2.a().onTrue(new InstantCommand(arm::armRest));
        xbox2.b().onTrue(new InstantCommand(() -> arm.armDistanceSetpoint(swerve.getPose())));
        xbox2.y().onTrue(new InstantCommand(() -> arm.setGoal(Units.degreesToRadians(10))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
