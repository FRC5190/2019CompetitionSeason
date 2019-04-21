package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.auto.paths.TrajectoryWaypoints
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeCloseCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.millisecond
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

            val path2 = super.followVisionAssistedTrajectory(
                path2,
                Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                4.feet, false
            )

            // Part 2: Place hatch and go to loading station.
            +parallel {
                // Follow the trajectory with vision correction to the loading station.
                +path2
                // Take the superstructure to pickup position and arm hatch intake 3 seconds before arrival.
                +sequential {
                    // Place hatch panel.
                    +IntakeHatchCommand(true).withTimeout(0.5.second)
                    +IntakeCloseCommand()
                    +Superstructure.kBackHatchFromLoadingStation.withTimeout(3.second)
                    +IntakeHatchCommand(false).withExit { path2.wrappedValue.isCompleted }
                }
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
                +IntakeHatchCommand(false).withTimeout(0.5.second)
                // Follow the trajectory with vision correction to the near side of the rocket.
                +super.followVisionAssistedTrajectory(
                    path3,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    6.feet, true
                )
                // Take the superstructure to scoring height.
                +Superstructure.kFrontHatchFromLoadingStation.withTimeout(4.second)
            }

            // Part 4: Score the hatch and go to the loading station for the end of the sandstorm period.
            +parallel {
                // Score hatch.
                // Follow the trajectory to the loading station.
                +DriveSubsystem.followTrajectory(
                    TrajectoryFactory.rocketNToLoadingStation,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)
                )
                // Take the superstructure to a position to pick up the next hatch.
                +sequential {
                    +IntakeHatchCommand(releasing = true).withTimeout(0.5.second)
                    +IntakeCloseCommand()
                    +Superstructure.kBackHatchFromLoadingStation
                }
            }
        }
}