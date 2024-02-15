// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DualStickSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    private final Limelight limelight = new Limelight();
    private final Swerve swerve = new Swerve(limelight);
    private final Arm arm = new Arm(swerve::getPose);
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake(shooter::getMotorSpeeds);

    private final CommandXboxController xbox = new DeadbandCommandXboxController(0);
    private final CommandXboxController xbox2 = new DeadbandCommandXboxController(1);

    private final SendableChooser<Command> autoChooser;

    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getLeftY, xbox::getLeftX,
            () -> -xbox.getRightX(), () -> xbox.rightBumper().getAsBoolean());

    public RobotContainer() {
        swerve.setDefaultCommand(driveCommand);
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        PowerDistribution pd = new PowerDistribution();
        pd.clearStickyFaults();
        pd.close();
        // arm.setDefaultCommand(armDistancePose);
        // Initializing autoChooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        xbox2.leftBumper().whileTrue(new InstantCommand(intake::backward)).onFalse(new InstantCommand(intake::stop));
        xbox2.rightTrigger().whileTrue(new InstantCommand(shooter::forward)).onFalse(new InstantCommand(shooter::stop));
        xbox2.leftTrigger().whileTrue(new InstantCommand(shooter::backward)).onFalse(new InstantCommand(shooter::stop));
        xbox2.y().onTrue(new InstantCommand(() -> {arm.setManualMode(true); arm.armAmp();}));
        xbox2.a().onTrue(new InstantCommand(() -> {arm.setManualMode(true); arm.armRest();}));
        xbox2.b().whileTrue(new InstantCommand(() -> {arm.setManualMode(false);}));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
