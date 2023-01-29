package frc.sorutil.path;

import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class AsyncTrajectory {
    private static class TrajectoryGenerationTask implements Callable<Trajectory> {
        private final Pose2d start, end;
        private final List<Translation2d> waypoints;
        private final TrajectoryConfig config;

        public TrajectoryGenerationTask(Pose2d start, Pose2d end, List<Translation2d> waypoints,
                TrajectoryConfig config) {
            this.start = start;
            this.end = end;
            this.waypoints = waypoints;
            this.config = config;
        }

        public Trajectory call() {
            return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
        }
    }

    private static final ExecutorService executor = Executors.newSingleThreadExecutor();

    /**
     * Wraps the standard WPILib {@link TrajectoryGenerator}'s generateTrajectory
     * method. Instead of directly computing a trajectory and returning it
     * immediately, this method will run the trajectory computation on another
     * thread, allowing robot code execution to continue.
     * 
     * <p>
     * To use this method, call it and store the result, on subsequent program
     * loops, call isDone() on the returned Future. If it is done, call get() and
     * discard the Future.
     * </p>
     */
    public static Future<Trajectory> generateTrajectory(Pose2d start, Pose2d end, List<Translation2d> waypoints,
            TrajectoryConfig config) {
        var callable = new TrajectoryGenerationTask(start, end, waypoints, config);
        return executor.submit(callable);
    }
}
