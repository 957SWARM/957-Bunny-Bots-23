package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

public class PathPlannerUtil {
    // this is to get around a stupid limitation with pathplanner 2023
    // source code (mostly) copied from pathplannerlib internals, and subject to its original
    // license
    public static PathPlannerState sampleFullState(double timeSeconds, PathPlannerTrajectory traj) {
        if (timeSeconds <= traj.getInitialState().timeSeconds) return traj.getInitialState();
        if (timeSeconds >= traj.getTotalTimeSeconds()) return traj.getEndState();

        int low = 1;
        int high = traj.getStates().size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (traj.getState(mid).timeSeconds < timeSeconds) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }

        // should really be lerping this, but it's *good enough* (in theory)
        // unfortunately the lerp method is not visible
        return traj.getState(low);
    }
}
