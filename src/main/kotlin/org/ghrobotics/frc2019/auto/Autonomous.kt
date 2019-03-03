package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.auto.paths.TrajectoryWaypoints
import org.ghrobotics.frc2019.auto.routines.BaselineRoutine
import org.ghrobotics.frc2019.auto.routines.CargoShipRoutine
import org.ghrobotics.frc2019.auto.routines.RocketRoutine
import org.ghrobotics.frc2019.auto.routines.TestTrajectoriesRoutine
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.InstantRunnableCommand
import org.ghrobotics.lib.commands.S3ND
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.commands.stateCommandGroup
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.and
import org.ghrobotics.lib.utils.monitor
import org.ghrobotics.lib.utils.onChangeToTrue
import org.ghrobotics.lib.wrappers.FalconRobot

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous {

    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    // Starting position of the robot
    val startingPosition = { Network.startingPositionChooser.selected }

//    val cargoShipGamePiece1 = { Network.cargoShip1Chooser.selected }
//    val cargoShipGamePiece2 = { Network.cargoShip2Chooser.selected }
//    val cargoShipGamePiece3 = { Network.cargoShip3Chooser.selected }

    // Stores whether the current config is valid.
    private var configValid = Source(true)

    // Stores if we are ready to send it.
    private val isReady =
        { Robot.lastRobotMode == FalconRobot.Mode.AUTONOMOUS && Robot.lastEnabledState } and configValid

    /**
     * Routine that gets executed if an invalid starting position + routine is selected.
     */
    private val invalidOptionRoutine
        get() = sequential {
            +InstantRunnableCommand {
                println("[Autonomous] Invalid Option for this Starting Configuration. Running Baseline.")
            }
            +BaselineRoutine()()
        }

    fun update() {
        // Update localization.
        startingPositionMonitor.onChange { if (!Robot.lastEnabledState) DriveSubsystem.localization.reset(it.pose) }

        // Just send it when everything is ready.
        isReadyMonitor.onChangeToTrue {
            JUST S3ND IT
        }

        // Stop the auto routine when we are not in autonomous anymore.
        modeMonitor.onChange { newValue ->
            if (newValue != FalconRobot.Mode.AUTONOMOUS) JUST.stop()
        }
    }

    // Autonomous Master Group
    private val JUST = stateCommandGroup(startingPosition) {
        state(StartingPositions.LEFT, StartingPositions.RIGHT) {
            stateCommandGroup(autoMode) {

                state(
                    Mode.ROCKET,
                    RocketRoutine()
                )
                state(
                    Mode.BASELINE,
                    BaselineRoutine()
                )
//                state(Mode.SIDE_CARGO_SHIP, SideCargoShipRoutine())

                state(
                    Mode.TEST_TRAJECTORIES,
                    TestTrajectoriesRoutine()
                )

                state(Mode.CARGO_SHIP, invalidOptionRoutine)
            }
        }
        state(StartingPositions.CENTER) {
            stateCommandGroup(autoMode) {
                state(
                    Mode.CARGO_SHIP,
                    CargoShipRoutine()()
                )
                state(
                    Mode.BASELINE,
                    BaselineRoutine()
                )

                state(
                    Mode.TEST_TRAJECTORIES,
                    TestTrajectoriesRoutine()
                )

                state(Mode.ROCKET, invalidOptionRoutine)
            }
        }
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = startingPosition.monitor
    private val isReadyMonitor = isReady.monitor
    private val modeMonitor = { Robot.lastRobotMode }.monitor


    enum class StartingPositions(val pose: Pose2d) {
        LEFT(TrajectoryWaypoints.kSideStart.mirror),
        CENTER(TrajectoryWaypoints.kCenterStart),
        RIGHT(TrajectoryWaypoints.kSideStart)
    }

    enum class GamePiece { HATCH, CARGO }

    enum class Mode { TEST_TRAJECTORIES, ROCKET, CARGO_SHIP, BASELINE }
}
