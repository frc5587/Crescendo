// package frc.robot.commands;

// import java.util.function.Supplier;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.subsystems.Swerve;

// public class DriveToPose extends Command {
//     private final Swerve swerve;
//     private final Supplier<Pose2d> poseSupplier;
//     public Field2d desiredPoseField = new Field2d();

//     private boolean running = false;

//     /** Drives to the specified pose under full software control. */
//     public DriveToPose(Swerve swerve, Pose2d pose) {
//         this(swerve, () -> pose);
//     }

//     /** Drives to the specified pose under full software control. */
//     public DriveToPose(Swerve swerve, Supplier<Pose2d> poseSupplier) {
//         this.swerve = swerve;
//         this.poseSupplier = poseSupplier;
//         addRequirements(swerve);
//         AutoConstants.BOT_ANGLE_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
//         SmartDashboard.putData("Desired Pose Field", desiredPoseField);
//     }

//     @Override
//     public void initialize() {
//         // Reset all controllers
//         Pose2d currentPose = swerve.getPose();
//         AutoConstants.BOT_DRIVE_CONTROLLER.reset(
//                 currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
//         AutoConstants.BOT_ANGLE_CONTROLLER.reset(currentPose.getRotation().getRadians());
//     }

//     @Override
//     public void execute() {
//         this.running = true;
//         desiredPoseField.setRobotPose(poseSupplier.get());
//         // Get current and target pose
//         Pose2d currentPose = swerve.getPose();
//         Pose2d targetPose = poseSupplier.get();
//         // Command speeds
//         double driveVelocityScalar = AutoConstants.BOT_DRIVE_CONTROLLER.calculate(
//                 currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()), 0.0);
//         double thetaVelocity = AutoConstants.BOT_ANGLE_CONTROLLER.calculate(
//                 currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
//         if (AutoConstants.BOT_DRIVE_CONTROLLER.atGoal()) {
//             driveVelocityScalar = 0.0;
//         }
//         if (AutoConstants.BOT_ANGLE_CONTROLLER.atGoal()) {
//             thetaVelocity = 0.0;
//         }
//         Translation2d driveVelocity = new Pose2d(
//                 new Translation2d(),
//                 currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
//                 .transformBy(new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
//                 .getTranslation();
//         swerve.setChassisSpeeds(
//                 new ChassisSpeeds(
//                         -driveVelocity.getX(), driveVelocity.getY(), thetaVelocity));
//     }

//     @Override
//     public void end(boolean interrupted) {
//         this.running = false;
//         swerve.stop();
//     }

//     public boolean atGoal() {
//         return running && AutoConstants.BOT_DRIVE_CONTROLLER.atGoal() && AutoConstants.BOT_ANGLE_CONTROLLER.atGoal();
//     }
// }