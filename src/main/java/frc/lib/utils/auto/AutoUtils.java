package frc.lib.utils.auto;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutoUtils {
    public static List<PathPlannerTrajectory> loadTrajectoriesWithConstraints(String name) {
        return PathPlanner.loadPathGroup(name, PathPlanner.getConstraintsFromPath(name));
    }
}
