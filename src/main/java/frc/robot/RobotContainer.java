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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoRotateToShoot;
import frc.robot.commands.AutoShootWhenLinedUp;
import frc.robot.commands.CharacterizationManager;
import frc.robot.commands.ClimbWithAxis;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.LineUpToSpeaker;
import frc.robot.commands.RunIntakeWithArm;
import frc.robot.subsystems.Arm;
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
    private final Arm arm = new Arm(swerve::getPose);
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake(shooter::isSpunUp, swerve::getLinearVelocity, (rumbleMagnitude) -> {
        xbox.getHID().setRumble(RumbleType.kBothRumble, rumbleMagnitude);
        xbox2.getHID().setRumble(RumbleType.kBothRumble, rumbleMagnitude);
    });
    private final SendableChooser<Command> autoChooser;
    private final CharacterizationManager charManager = new CharacterizationManager(arm, swerve, shooter);

    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getLeftY, () -> -xbox.getLeftX(),
            () -> xbox.getRightX(), xbox.rightBumper().negate());
    private final AutoRotateToShoot autoRotateToShoot = new AutoRotateToShoot(swerve);
    private final LineUpToSpeaker lineUpToSpeaker = new LineUpToSpeaker(swerve);
    private final AutoShootWhenLinedUp autoShootWhenLinedUp = new AutoShootWhenLinedUp(shooter, intake, arm, xbox.leftBumper());
    private final RunIntakeWithArm runIntakeWithArm = new RunIntakeWithArm(intake, arm, shooter::isSpunUp);
    private final SendableChooser<Command> charChooser = new SendableChooser<Command>();
    private final ClimbWithAxis 
    climbWithAxis = new ClimbWithAxis(xbox2::getLeftTriggerAxis, arm);

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
        NamedCommands.registerCommand("shooterForward", new InstantCommand(shooter::forward));
        NamedCommands.registerCommand("shooterIdle", new InstantCommand(shooter::idleSpeed));
        NamedCommands.registerCommand("shooterStop", new InstantCommand(shooter::stop));
        NamedCommands.registerCommand("armRest", new InstantCommand(() -> {arm.setManualMode(true); arm.armRest();}));
        NamedCommands.registerCommand("armAim", new InstantCommand(() -> {arm.setManualMode(false);}));
        NamedCommands.registerCommand("armAmp", arm.armAmpCommand());
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        charChooser.setDefaultOption("Arm Q Fwd", charManager.getArmChar().quasistatic(SysIdRoutine.Direction.kForward));
        charChooser.addOption("Arm Q Bwd", charManager.getArmChar().quasistatic(SysIdRoutine.Direction.kReverse));
        charChooser.addOption("Arm D Fwd", charManager.getArmChar().dynamic(SysIdRoutine.Direction.kForward));
        charChooser.addOption("Arm D Bwd", charManager.getArmChar().dynamic(SysIdRoutine.Direction.kReverse));
        charChooser.addOption("Swerve Q Fwd", charManager.getSwerveChar().quasistatic(SysIdRoutine.Direction.kForward));
        charChooser.addOption("Swerve Q Bwd", charManager.getSwerveChar().quasistatic(SysIdRoutine.Direction.kReverse));
        charChooser.addOption("Swerve D Fwd", charManager.getSwerveChar().dynamic(SysIdRoutine.Direction.kForward));
        charChooser.addOption("Swerve D Bwd", charManager.getSwerveChar().dynamic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Char", charChooser);
        
        CameraServer.startAutomaticCapture(0);
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
        xbox2.leftBumper().whileTrue(new InstantCommand(intake::backward)).onFalse(new InstantCommand(intake::stop));
        xbox2.rightBumper().whileTrue(runIntakeWithArm);
        xbox2.rightTrigger().onTrue(new InstantCommand(shooter::forward)).onFalse(new InstantCommand(shooter::idleSpeed));
        // xbox2.leftTrigger().whileTrue(new InstantCommand(shooter::backward)).onFalse(new InstantCommand(shooter::idleSpeed));
        xbox2.leftTrigger(0.1).whileTrue(climbWithAxis);
        // xbox2.povLeft().whileTrue(autoShootWhenLinedUp);
        // xbox2.a().onTrue(arm.armTravelCommand());
        // xbox2.b().onTrue(arm.disableManualMode());
        // xbox2.x().onTrue(arm.armRestCommand());
        // xbox2.y().onTrue(arm.armAmpCommand());
        xbox2.a().whileTrue(charManager.getShooterChar().dynamic(Direction.kReverse));
        xbox2.b().whileTrue(charManager.getShooterChar().quasistatic(Direction.kReverse));
        xbox2.x().whileTrue(charManager.getShooterChar().quasistatic(Direction.kForward));
        xbox2.y().whileTrue(charManager.getShooterChar().dynamic(Direction.kForward));
        xbox2.povUp().whileTrue(charManager.getArmChar().dynamic(Direction.kForward));
        xbox2.povDown().whileTrue(charManager.getArmChar().dynamic(Direction.kReverse));
        xbox2.povLeft().whileTrue(charManager.getArmChar().quasistatic(Direction.kForward));
        xbox2.povRight().whileTrue(charManager.getArmChar().quasistatic(Direction.kReverse));
        xbox.povUp().whileTrue(charManager.getSwerveChar().dynamic(Direction.kForward));
        xbox.povDown().whileTrue(charManager.getSwerveChar().dynamic(Direction.kReverse));
        xbox.povLeft().whileTrue(charManager.getSwerveChar().quasistatic(Direction.kForward));
        xbox.povRight().whileTrue(charManager.getSwerveChar().quasistatic(Direction.kReverse));
        // xbox.povDown().whileTrue(autoRotateToShoot);
        // xbox.povUp().whileTrue(lineUpToSpeaker);
        // xbox.povLeft().whileTrue(swerve.subwooferLineUp());
        // xbox.povRight().whileTrue(swerve.ampLineUp());
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return charChooser.getSelected();
  }
}
