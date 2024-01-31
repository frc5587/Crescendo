// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.PositionArm;
import frc.robot.commands.SimDualStickSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SimGyro;
import frc.robot.subsystems.SimSwerve;
import frc.robot.subsystems.SimSwerveModule;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    protected final Swerve swerve = new Swerve();
    protected final Arm arm = new Arm();
    protected final SimSwerve simSwerve = new SimSwerve(
            new SimGyro() {}, new SimSwerveModule() {}, new SimSwerveModule() {},
            new SimSwerveModule() {}, new SimSwerveModule() {});

    private final SendableChooser<Command> autoChooser;
    private final DeadbandCommandXboxController xbox = new DeadbandCommandXboxController(0, 0.2);

    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getLeftY, xbox::getLeftX,
            () -> {
                return -xbox.getRightX();
            }, () -> xbox.rightBumper().negate().getAsBoolean());
    private final SimDualStickSwerve simDriveCommand = new SimDualStickSwerve(simSwerve, () -> -xbox.getRawAxis(1),
            () -> xbox.getRawAxis(0),
            () -> -xbox.getRawAxis(2), () -> xbox.rightBumper().negate().getAsBoolean());

    private final PositionArm positionArm = new PositionArm(arm, simSwerve::getPose);

    /*
     * Constructor
     */
    public RobotContainer() {
        // Configure the trigger bindings
        swerve.setDefaultCommand(driveCommand);
        simSwerve.setDefaultCommand(simDriveCommand);
        arm.setDefaultCommand(positionArm);
        configureBindings();

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
        xbox.a().onTrue(new InstantCommand(arm::ArmSpeaker));
        xbox.y().onTrue(new InstantCommand(arm::ArmRest));
        xbox.rightBumper().whileTrue(new RunCommand(() -> SmartDashboard.putBoolean("Robot Oriented Drive", true))).onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Robot Oriented Drive", false)));
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
