package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.utils.withEquals

class TestTrajectoriesRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.sideStartToRocketN
    private val path2 = TrajectoryFactory.rocketNToLoadingStation
    private val path3 = TrajectoryFactory.loadingStationToRocketF
    private val path4 = TrajectoryFactory.rocketFToDepot

    override val duration: Time
        get() = path1.duration + path2.duration + path3.duration + path4.duration

    override val routine
        get() = sequential {
            val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

            +DriveSubsystem.followTrajectory(path1, pathMirrored)
            +DriveSubsystem.followTrajectory(path2, pathMirrored)
            +DriveSubsystem.followTrajectory(path3, pathMirrored)
//
//
//            +DriveSubsystem.followVisionAssistedTrajectory(path1, pathMirrored, 3.feet, 2.feet)
//            +DriveSubsystem.followTrajectory(path2, pathMirrored)
//            +DriveSubsystem.followVisionAssistedTrajectory(path3, pathMirrored, 4.feet, 3.feet)
        }
}
