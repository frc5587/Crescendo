// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AutoRotateToShoot;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.LineUpToSpeaker;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DeadbandedCommandXboxController;

public class RobotContainer {
    private final Limelight limelight = new Limelight();
    private final Swerve swerve = new Swerve(limelight);
    private final Arm arm = new Arm(swerve::getPose);
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake(shooter::getMotorSpeeds);
    private final SendableChooser<Command> autoChooser;

    public final DeadbandedCommandXboxController xbox = new DeadbandedCommandXboxController(0, 0.2);
    public final DeadbandedCommandXboxController xbox2 = new DeadbandedCommandXboxController(1);

    private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getLeftY, () -> -xbox.getLeftX(),
            () -> xbox.getRightX(), () -> xbox.rightBumper().negate().getAsBoolean());
    private final AutoRotateToShoot autoRotateToShoot = new AutoRotateToShoot(swerve);
    private final LineUpToSpeaker lineUpToSpeaker = new LineUpToSpeaker(swerve);

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
        NamedCommands.registerCommand("shooterStop", new InstantCommand(shooter::stop));
        NamedCommands.registerCommand("armRest", new InstantCommand(() -> {arm.setManualMode(true); arm.armRest();}));
        NamedCommands.registerCommand("armAim", new InstantCommand(() -> {arm.setManualMode(false);}));
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
        Trigger rB = xbox2.rightBumper();
        Trigger lB = xbox2.leftBumper();
        Trigger rT = xbox2.rightTrigger();
        Trigger lT = xbox2.leftTrigger();
        Trigger y = xbox2.y();
        Trigger a = xbox2.a();
        Trigger b = xbox2.b();
        Trigger intakeLimitSwitch = new Trigger(intake::getLimitSwitch);
        // rB.whileTrue(new RunCommand(() -> intake.setVelocity(((Math.sqrt(Math.pow(swerve.getChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(swerve.getChassisSpeeds().vyMetersPerSecond, 2))) * IntakeConstants.SWERVE_VELOCITY_OFFSET) + IntakeConstants.MINIMUM_VELOCITY)));
        // lB.whileTrue(new RunCommand(() -> intake.setVelocity(IntakeConstants.MINIMUM_VELOCITY)));/* .onFalse(new InstantCommand(intake::stop));*/
        lB.whileTrue(new InstantCommand(intake::backward)).onFalse(new InstantCommand(intake::stop));
        rB.whileTrue(new InstantCommand(intake::forward)).onFalse(new InstantCommand(intake::stop));
        
        rT.whileTrue(new InstantCommand(shooter::forward)).onFalse(new InstantCommand(shooter::stop));
        lT.whileTrue(new InstantCommand(shooter::backward)).onFalse(new InstantCommand(shooter::stop));
        y.onTrue(arm.armAmpCommand());
        a.onTrue(arm.armRestCommand());
        b.onTrue(arm.disableManualMode());
        xbox2.x().onTrue(arm.enableManualMode().andThen(new InstantCommand(() -> arm.setGoal(Units.degreesToRadians(3)))));
        xbox.povDown().onTrue(swerve.ampLineUp());
        intakeLimitSwitch.onTrue(arm.disableManualMode());
        xbox.povDown().whileTrue(autoRotateToShoot);
        xbox.povUp().whileTrue(lineUpToSpeaker);
        xbox2.povLeft().onTrue(arm.armStageCommand());
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
