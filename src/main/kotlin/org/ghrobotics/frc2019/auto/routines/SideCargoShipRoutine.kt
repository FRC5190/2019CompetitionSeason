package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.auto.paths.TrajectoryWaypoints
import org.ghrobotics.frc2019.subsystems.elevator.ClosedLoopElevatorCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.utils.withEquals

class SideCargoShipRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.sideStartToCargoShipS1
    private val path2 = TrajectoryFactory.cargoShipS1ToLoadingStation
    private val path3 = TrajectoryFactory.loadingStationToCargoShipS2

    private val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

    override val duration = path1.duration + path2.duration + path3.duration

    override val routine: FalconCommand
        get() = sequential {
            +ClosedLoopElevatorCommand(3.inch)
            +followVisionAssistedTrajectory(path1, pathMirrored, 4.feet, true)
            +relocalize(TrajectoryWaypoints.kCargoShipS1, forward = true, pathMirrored = pathMirrored)

            +followVisionAssistedTrajectory(path2, pathMirrored, 6.feet, false)
            +relocalize(TrajectoryWaypoints.kLoadingStation, forward = false, pathMirrored = pathMirrored)

            +followVisionAssistedTrajectory(path3, pathMirrored, 4.feet, true)
        }
}