package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.auto.paths.TrajectoryWaypoints
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

class BottomRocketRoutine : AutoRoutine() {

    // First path goes to the far side of the rocket
    private val path1 = TrajectoryFactory.sideStartToRocketF

    // Second path goes to the loading station to pick up a hatch panel
    private val path2 = TrajectoryFactory.rocketFToLoadingStation

    // Third path goes to the near side of the rocket
    private val path3 = TrajectoryFactory.loadingStationToRocketN


    // Calculates the duration of the path
    override val duration = path1.duration + path2.duration + path3.duration


    // Auto routine
    override val routine
        get() = sequential {

            // Part 1: Go to the far side of the rocket and get ready to place a hatch on the lowest level.
            +parallel {
                // Make sure the intake is holding the hatch panel.
                +IntakeHatchCommand(releasing = false)
                // Follow the trajectory with vision correction to the far side of the rocket.
                +super.followVisionAssistedTrajectory(
                    path1,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    4.feet, true
                )
                // Take the superstructure to scoring height once out of the platform.
                +sequential {
                    +DriveSubsystem.notWithinRegion(TrajectoryWaypoints.kHabitatL1Platform)
                    +Superstructure.kFrontHatchFromLoadingStation
                }.withTimeout(4.second)
            }

            // Reorient position on field based on Vision alignment.
            +relocalize(
                TrajectoryWaypoints.kRocketF,
                true,
                Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)
            )

            // Part 2: Place hatch and go to loading station.
            +parallel {
                // Place hatch panel.
                +IntakeHatchCommand(true)
                // Follow the trajectory with vision correction to the loading station.
                +super.followVisionAssistedTrajectory(
                    path2,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    4.feet, false
                )
                // Take the superstructure to scoring height and arm hatch intake 3 seconds before arrival.
                +sequential {
                    +DelayCommand(path2.duration - 3.second)
                    +IntakeHatchCommand(false)
                    +Superstructure.kBackHatchFromLoadingStation
                }.withTimeout(3.second)
            }

            // Reorient position on field based on Vision alignment.
            +relocalize(
                TrajectoryWaypoints.kLoadingStation,
                false,
                Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)
            )

            // Part 3: Pickup hatch and go to the near side of the rocket.
            +parallel {
                // Make sure the intake is holding the hatch panel.
                +IntakeHatchCommand(false)
                // Follow the trajectory with vision correction to the near side of the rocket.
                +super.followVisionAssistedTrajectory(
                    path3,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    6.feet, true
                )
                // Take the superstructure to scoring height.
                +Superstructure.kFrontHatchFromLoadingStation.withTimeout(2.second)
            }

            // Part 4: Score the hatch and go to the loading station for the end of the sandstorm period.
            +parallel {
                // Score hatch.
                +IntakeHatchCommand(releasing = true)
                // Follow the trajectory to the loading station.
                +DriveSubsystem.followTrajectory(
                    TrajectoryFactory.rocketNToLoadingStation,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)
                )
            }
        }
}