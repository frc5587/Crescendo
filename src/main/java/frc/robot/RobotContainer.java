// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DeadbandedCommandXboxController;

public class RobotContainer {
    public final DeadbandedCommandXboxController xbox = new DeadbandedCommandXboxController(0, 0.2);
    public final DeadbandedCommandXboxController xbox2 = new DeadbandedCommandXboxController(1);

    private final Limelight limelight = new Limelight();
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
    private final RunIntakeWithArm runIntakeWithArm = new RunIntakeWithArm(intake, arm, shooter::isSpunUp);
    private final FullClimb fullClimb = new FullClimb(climb, arm, shooter);

    public void teleopInitRoutine() {
        // TODO: VERIFY!!
        swerve.gyro.setYaw(swerve.getYaw()
                .times(DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? 1 : -1));
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
        xbox2.leftBumper().whileTrue(new InstantCommand(intake::backward).alongWith(new InstantCommand(shooter::backward))).onFalse(new InstantCommand(intake::stop).alongWith(new InstantCommand(shooter::idleSpeed)));
        xbox2.rightBumper().whileTrue(runIntakeWithArm);
        xbox2.leftTrigger().whileTrue(new InstantCommand(shooter::backward)).onFalse(new InstantCommand(shooter::idleSpeed));
        xbox2.rightTrigger().whileTrue(new InstantCommand(shooter::forward)).onFalse(new InstantCommand(shooter::idleSpeed));

        xbox2.a().onTrue(arm.armTravelCommand());
        xbox2.b().onTrue(arm.disableManualMode());
        xbox2.x().onTrue(arm.armZeroCommand());
        xbox2.y().onTrue(arm.armAmpCommand());
        // xbox2.start().onTrue(arm.armFerryCommand());
        xbox2.back().whileTrue(fullClimb);

        xbox2.povUp().onTrue(new InstantCommand(climb::up));
        xbox2.povDown().onTrue(new InstantCommand(climb::down));
        xbox2.povLeft().whileTrue(autoAmpWhenLinedUp);
        xbox2.povRight().whileTrue(autoShootWhenLinedUp);
        
        xbox.povDown().whileTrue(autoRotateToShoot);
        xbox.povUp().whileTrue(swerve.subwooferLineUp());
        xbox.a().whileTrue(new InstantCommand(swerve::standYourGround, swerve));
        
        /* Unused Binds */
        // xbox2.rightBumper().whileTrue(new InstantCommand(intake::forward)).onFalse(new InstantCommand(intake::stop));
        // xbox2.povRight().onTrue(new InstantCommand(shooter::stop));
        // xbox2.leftTrigger(0.1).whileTrue(climbWithAxis);
        // xbox2.povUp().whileTrue(new InstantCommand(shooter::spinUpToAmp)).onFalse(new InstantCommand(shooter::idleSpeed));
        // xbox2.povDown().onTrue(arm.chinUp());
        // intakeLimitSwitch.onTrue(arm.travelSetpointCommand());
        // rB.whileTrue(new RunCommand(() -> intake.setVelocity(((Math.sqrt(Math.pow(swerve.getChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(swerve.getChassisSpeeds().vyMetersPerSecond, 2))) * IntakeConstants.SWERVE_VELOCITY_OFFSET) + IntakeConstants.MINIMUM_VELOCITY)));
        // lB.whileTrue(new RunCommand(() -> intake.setVelocity(IntakeConstants.MINIMUM_VELOCITY)));/* .onFalse(new InstantCommand(intake::stop));*/
        // xbox.povLeft().whileTrue(swerve.subwooferLineUp());
        // xbox.povRight().whileTrue(swerve.ampLineUp());
        // xbox2.rightTrigger(0.1).whileTrue(climbWithAxisReverse);
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
