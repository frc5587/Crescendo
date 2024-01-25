package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;

public class AutoCommands {
    // Auto Paths
    private PathPlannerTrajectory amp_dep_preloaded = PathPlannerPath.fromPathFile("amp_dep_preloaded").getTrajectory();
}
