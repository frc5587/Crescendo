// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimToNote;
import frc.robot.commands.AutoAmpWhenLinedUp;
import frc.robot.commands.AutoRotateToShoot;
import frc.robot.commands.AutoShootWhenLinedUp;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.FullClimb;
import frc.robot.commands.RunIntakeWithArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NoteDetector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    public final DeadbandCommandXboxController xbox = new DeadbandCommandXboxController(0, 0.2);
    public final DeadbandCommandXboxController xbox2 = new DeadbandCommandXboxController(1);

    private final Limelight limelight = new Limelight();
    private final NoteDetector noteDetector = new NoteDetector();
    private final Swerve swerve = new Swerve(limelight);
    private final Shooter shooter = new Shooter(swerve::getPose);
    private final Intake intake = new Intake(shooter::isSpunUp, xbox2.rightTrigger(), swerve::getLinearVelocity, (rumbleMagnitude) -> {
            xbox.getHID().setRumble(RumbleType.kBothRumble, rumbleMagnitude);
            xbox2.getHID().setRumble(RumbleType.kBothRumble, rumbleMagnitude);
    });
    public final Arm arm = new Arm(swerve::getPose);
    private final Climb climb = new Climb();
    private final SendableChooser<Command> autoChooser;

    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getLeftY, () -> -xbox.getLeftX(),
            () -> xbox.getRightX(), xbox.rightBumper().negate(), xbox.leftTrigger());
    private final AutoRotateToShoot autoRotateToShoot = new AutoRotateToShoot(swerve);
    private final AutoShootWhenLinedUp autoShootWhenLinedUp = new AutoShootWhenLinedUp(shooter, intake, xbox.leftBumper());
    private final AutoAmpWhenLinedUp autoAmpWhenLinedUp = new AutoAmpWhenLinedUp(shooter, intake, xbox.leftBumper());
    private final RunIntakeWithArm runIntakeWithArm = new RunIntakeWithArm(intake, arm, shooter::isSpunUp, xbox2.rightTrigger());
    private final FullClimb fullClimb = new FullClimb(climb, arm, shooter);
    private final Command aimToNote = new AimToNote(noteDetector, swerve, intake::getLimitSwitch, arm::getAngleRadians);

    public void teleopInitRoutine() {
        swerve.gyro.setYaw(swerve.getYaw()
                .plus(DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? new Rotation2d() : (swerve.getYaw().getDegrees() < 0 ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(-180))));
        intake.stop();
        shooter.idleSpeed();
    }

    public RobotContainer() {
        swerve.setDefaultCommand(driveCommand);
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        PowerDistribution pd = new PowerDistribution();
        pd.clearStickyFaults();
        pd.close();
        NamedCommands.registerCommand("intakeForward", new InstantCommand(intake::forward));
        NamedCommands.registerCommand("intakeStop", new InstantCommand(intake::stop));
        NamedCommands.registerCommand("shooterForward", new RunCommand(shooter::forward));
        NamedCommands.registerCommand("shooterIdle", new InstantCommand(shooter::idleSpeed));
        NamedCommands.registerCommand("shooterStop", new InstantCommand(shooter::stop));
        NamedCommands.registerCommand("armTravel", arm.armTravelCommand());
        NamedCommands.registerCommand("armRest", arm.armTravelCommand());
        NamedCommands.registerCommand("armAim", new InstantCommand(() -> {arm.setManualMode(false);}));
        NamedCommands.registerCommand("armAmp", arm.armAmpCommand());
        NamedCommands.registerCommand("armFerry", arm.armFerryCommand());
        NamedCommands.registerCommand("rotateToShoot", new AutoRotateToShoot(swerve));
        NamedCommands.registerCommand("noteAim", aimToNote);
        NamedCommands.registerCommand("confirmShot", new InstantCommand(intake::confirmShot));
        NamedCommands.registerCommand("denyShot", new InstantCommand(intake::denyShot));
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
        xbox2.leftBumper().whileTrue(new InstantCommand(intake::backward).alongWith(new InstantCommand(shooter::backward))).onFalse(new InstantCommand(intake::stop).alongWith(new InstantCommand(shooter::idleSpeed)));
        xbox2.rightBumper().whileTrue(runIntakeWithArm);
        xbox2.leftTrigger().whileTrue(new InstantCommand(shooter::backward)).onFalse(new InstantCommand(shooter::idleSpeed));
        xbox2.rightTrigger().whileTrue(new RunCommand(shooter::forward)).onFalse(new InstantCommand(shooter::idleSpeed));

        xbox2.a().onTrue(arm.armTravelCommand());
        xbox2.b().onTrue(arm.disableManualMode());
        xbox2.x().onTrue(arm.armTravelCommand());
        xbox2.y().onTrue(arm.armAmpCommand());
        xbox2.back().whileTrue(fullClimb);
        
        xbox2.povUp().onTrue(new InstantCommand(climb::hookTop));
        xbox2.povDown().onTrue(new InstantCommand(climb::hookBottom));
        xbox2.povRight().whileTrue(autoAmpWhenLinedUp);
        xbox2.povLeft().whileTrue(autoShootWhenLinedUp);
        
        xbox.povDown().whileTrue(autoRotateToShoot);
        xbox.povUp().whileTrue(swerve.subwooferLineUp());
        xbox.a().whileTrue(new InstantCommand(swerve::standYourGround, swerve));
        xbox.x().onTrue(arm.shuffleBoardArmCommand());
        xbox.y().whileTrue(aimToNote);
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
