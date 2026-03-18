package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.PoseUtils;

public class AutoConstants {
    public static final double DEFAULT_WAYPOINT_TOLERANCE = 0.1;

    public enum Position {
        STARTING_LINE_RIGHT,
        NEUTRAL_RIGHT_1,
        NEUTRAL_RIGHT_2,
        NEUTRAL_RIGHT_3,
        ALLIANCE_RIGHT_1,
        STARTING_LINE_LEFT,
        NEUTRAL_LEFT_1,
        NEUTRAL_LEFT_2,
        NEUTRAL_LEFT_3,
        ALLIANCE_LEFT_1,
        LEFT_DEPOT_BACKUP,
        LEFT_DEPOT_ALIGN,
        LEFT_DEPOT_INTAKE,
        LEFT_DEPOT_SHOOT
    }

    // Create map position enum to Pose2D
    public static Map<Position, Pose2d> positionToPose = Map.ofEntries(
            
            Map.entry(Position.STARTING_LINE_RIGHT, new Pose2d(4.0218614, 0.632, new Rotation2d(0))),
            Map.entry(Position.NEUTRAL_RIGHT_1, new Pose2d(8.02169132232666, 0.632, new Rotation2d(-1.3203533077139966))),
            Map.entry(Position.NEUTRAL_RIGHT_2,
            new Pose2d(7.77288293838501, 3.4187989234924316, new Rotation2d(-1.3203533077139966))),
            Map.entry(Position.NEUTRAL_RIGHT_3, new Pose2d(5.869497776031494, 0.632, new Rotation2d(Degrees.of(90)))),
            Map.entry(Position.ALLIANCE_RIGHT_1, new Pose2d(4.015, 0.632, new Rotation2d(Degrees.of(80.25)))),
            Map.entry(Position.STARTING_LINE_LEFT, PoseUtils.flipY(new Pose2d(4.0218614, 0.632, new Rotation2d(0)))),
            Map.entry(Position.NEUTRAL_LEFT_1,
            PoseUtils.flipY(new Pose2d(8.02169132232666, 0.632, new Rotation2d(-1.3203533077139966)))),
            Map.entry(Position.NEUTRAL_LEFT_2,
            PoseUtils.flipY(new Pose2d(7.77288293838501, 3.4187989234924316, new Rotation2d(-1.3203533077139966)))),
            Map.entry(Position.NEUTRAL_LEFT_3,
            PoseUtils.flipY(new Pose2d(5.869497776031494, 0.632, new Rotation2d(Degrees.of(90))))),
            Map.entry(Position.ALLIANCE_LEFT_1, PoseUtils.flipY(new Pose2d(4.015, 0.632, new Rotation2d(Degrees.of(80.25))))),
            Map.entry(Position.LEFT_DEPOT_BACKUP, new Pose2d(3.1397669315338135, 7.401589870452881, new Rotation2d(Degrees.of(0)))),
            Map.entry(Position.LEFT_DEPOT_ALIGN, new Pose2d(2.0726799964904785, 5.954537868499756, new Rotation2d(Degrees.of(0)))),
            Map.entry(Position.LEFT_DEPOT_INTAKE, new Pose2d(0.6686581373214722, 5.954537868499756, new Rotation2d(Degrees.of(0)))),
            Map.entry(Position.LEFT_DEPOT_SHOOT, new Pose2d(2.3726799964904785, 5.954537868499756, new Rotation2d(Degrees.of(0))))
            );
}
