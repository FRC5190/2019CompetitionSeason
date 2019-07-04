package org.ghrobotics.frc2019.auto.paths

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*

object TrajectoryWaypoints {

    /** Measured Field Coordinates **/

    // Habitat Zone
    val kHabitatL2RX = 48.00.inch
    val kHabitatL2BY = 97.00.inch
    val kHabitatL1RX = 95.28.feet
    val kHabitatL1Platform = Rectangle2d(Translation2d(4.feet, 7.feet), Translation2d(8.feet, 20.feet))
    val kRampHypotenuse = .4.inch


    // Cargo Ship
    val kCargoShipFL = Pose2d(220.25.inch, 172.88.inch, 0.degree)
    val kCargoShipFR = Pose2d(220.25.inch, 151.12.inch, 0.degree)
    val kCargoShipS1 = Pose2d(260.75.inch, 133.13.inch, 90.degree)
    val kCargoShipS2 = Pose2d(282.55.inch, 133.13.inch, 90.degree)
    val kCargoShipS3 = Pose2d(304.30.inch, 133.13.inch, 90.degree)

    // Rocket
    val kRocketN = Pose2d(214.57.inch, 19.57.inch, (-028.75).degree)
    val kRocketF = Pose2d(244.00.inch, 19.57.inch, (-151.25).degree)
    val kRocketBay = Pose2d(229.28.inch, 27.50.inch, (-90).degree)

    // Loading Station
    val kLoadingStation = Pose2d(0.inch, 25.72.inch, 0.degree)

    // Depot
    val kDepotBRCorner = Pose2d(47.inch, 78.396.inch, (-25).degree)


    /** Robot Starting Locations **/

    // Determine the starting X value for the robot.
    private val kStartX = kHabitatL2RX + Constants.kBumperThickness + Constants.kRobotLength / 2.0 - kRampHypotenuse

    // Starting on Level 1 HAB on the right side.
    val kSideStart = Pose2d(
        kHabitatL2RX + Constants.kBumperThickness + Constants.kRobotLength / 2.0 - kRampHypotenuse,
        kHabitatL2BY + Constants.kBumperThickness + Constants.kRobotWidth / 2.0,
        0.degree
    )

    val kSideStartReversed = Pose2d(kSideStart.translation, 180.degree)

    // Starting on Level 1 HAB in the center.
    val kCenterStart = Pose2d(kStartX, 13.5.feet)

    data class Waypoint(
        val trueLocation: Pose2d,
        val transform: Pose2d = Pose2d(),
        val translationalOffset: Translation2d = Translation2d(),
        val rotationalOffset: Rotation2d = 0.radian
    ) {

        private val trueAndTransform = trueLocation + transform

        val position = Pose2d(
            trueAndTransform.translation + translationalOffset,
            trueAndTransform.rotation + rotationalOffset
        )
    }
}