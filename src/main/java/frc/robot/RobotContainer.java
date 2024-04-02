// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAmpWhenLinedUp;
import frc.robot.commands.AutoRotateToShoot;
import frc.robot.commands.AutoShootWhenLinedUp;
import frc.robot.commands.CharacterizationBase;
import frc.robot.commands.CharacterizationManager;
import frc.robot.commands.ClimbWithAxis;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.FullClimb;
import frc.robot.commands.LineUpToSpeaker;
import frc.robot.commands.RunIntakeWithArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DeadbandedCommandXboxController;

public class RobotContainer {
    public final DeadbandedCommandXboxController xbox = new DeadbandedCommandXboxController(0, 0.2);
    public final DeadbandedCommandXboxController xbox2 = new DeadbandedCommandXboxController(1);

    private final Limelight limelight = new Limelight();
    private final Swerve swerve = new Swerve(limelight);
    private final Shooter shooter = new Shooter(swerve::getPose);
    private final Intake intake = new Intake(shooter::isSpunUp, swerve::getLinearVelocity, (rumbleMagnitude) -> {
        xbox.getHID().setRumble(RumbleType.kBothRumble, rumbleMagnitude);
        xbox2.getHID().setRumble(RumbleType.kBothRumble, rumbleMagnitude);
    });
    public final Arm arm = new Arm(swerve::getPose);
    private final Climb climb = new Climb();
    private final SendableChooser<Command> autoChooser;
    private final CharacterizationManager charManager = new CharacterizationManager(arm, swerve, shooter, climb);

    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getLeftY, () -> -xbox.getLeftX(),
            () -> xbox.getRightX(), xbox.rightBumper().negate(), xbox.leftTrigger());
    private final AutoRotateToShoot autoRotateToShoot = new AutoRotateToShoot(swerve);
    private final LineUpToSpeaker lineUpToSpeaker = new LineUpToSpeaker(swerve);
    private final AutoShootWhenLinedUp autoShootWhenLinedUp = new AutoShootWhenLinedUp(shooter, intake, arm, xbox.leftBumper());
    private final AutoAmpWhenLinedUp autoAmpWhenLinedUp = new AutoAmpWhenLinedUp(shooter, intake, xbox.leftBumper());
    private final RunIntakeWithArm runIntakeWithArm = new RunIntakeWithArm(intake, arm, shooter::isSpunUp);
    private final SendableChooser<CharacterizationBase> charChooser = new SendableChooser<CharacterizationBase>();
    private final ClimbWithAxis climbWithAxis = new ClimbWithAxis(xbox2::getLeftTriggerAxis, arm, climb, false);
    private final ClimbWithAxis climbWithAxisReverse = new ClimbWithAxis(xbox2::getRightTriggerAxis, arm, climb, true);
    private final FullClimb fullClimb = new FullClimb(climb, arm, shooter);

    public void zeroYaw() {
        swerve.zeroGyro();
    }

    public RobotContainer() {
        swerve.setDefaultCommand(driveCommand);
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        PowerDistribution pd = new PowerDistribution();
        pd.clearStickyFaults();
        pd.close();
        // arm.setDefaultCommand(armDistancePose);
        NamedCommands.registerCommand("intakeForward", new InstantCommand(intake::forward));
        NamedCommands.registerCommand("intakeStop", new InstantCommand(intake::stop));
        NamedCommands.registerCommand("shooterForward", new RunCommand(shooter::forward));
        NamedCommands.registerCommand("shooterIdle", new InstantCommand(shooter::idleSpeed));
        NamedCommands.registerCommand("shooterStop", new InstantCommand(shooter::stop));
        NamedCommands.registerCommand("armTravel", arm.armTravelCommand());
        NamedCommands.registerCommand("armRest", arm.armRestCommand());
        NamedCommands.registerCommand("armAim", new InstantCommand(() -> {arm.setManualMode(false);}));
        NamedCommands.registerCommand("armAmp", arm.armAmpCommand());
        NamedCommands.registerCommand("armFerry", arm.armFerryCommand());
        NamedCommands.registerCommand("rotateToShoot", new AutoRotateToShoot(swerve));
        NamedCommands.registerCommand("confirmShot", new InstantCommand(intake::confirmShot));
        NamedCommands.registerCommand("denyShot", new InstantCommand(intake::denyShot));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        charChooser.setDefaultOption("Arm Char", charManager.getArmChar());
        charChooser.addOption("Climb Char", charManager.getClimbChar());
        charChooser.addOption("Swerve Char", charManager.getSwerveChar());
        charChooser.addOption("Shooter Char", charManager.getShooterChar());
        SmartDashboard.putData("Char", charChooser);
        
        CameraServer.startAutomaticCapture(0);
    }

    public void stopIntake() {
        intake.stop();
        shooter.idleSpeed();
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
        xbox2.y().whileTrue(charChooser.getSelected().dynamic(Direction.kForward));
        xbox2.a().whileTrue(charChooser.getSelected().dynamic(Direction.kReverse));
        xbox2.povUp().whileTrue(charChooser.getSelected().quasistatic(Direction.kForward));
        xbox2.povDown().whileTrue(charChooser.getSelected().quasistatic(Direction.kReverse));
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
