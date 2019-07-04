package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.auto.paths.TrajectoryWaypoints
import org.ghrobotics.frc2019.auto.routines.*
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
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

    // Stores whether the current config is valid.
    private var configValid = Source(true)

    // Stores if we are ready to send it.
    private val isReady =
        { Robot.lastRobotMode == FalconRobot.Mode.AUTONOMOUS && Robot.lastEnabledState } and configValid

    // Update the autonomous listener.
    fun update() {
        // Update localization.
        startingPositionMonitor.onChange { if (!Robot.lastEnabledState) DriveSubsystem.localization.reset(it.pose) }

        modeMonitor.onChange { newValue ->
            if (newValue != FalconRobot.Mode.AUTONOMOUS) JUST.stop()
        }

        isReadyMonitor.onChangeToTrue {
            JUST S3ND IT
        }
    }

    // Autonomous Master Group
    private val JUST = stateCommandGroup(startingPosition) {
        state(
            StartingPositions.LEFT,
            StartingPositions.RIGHT,
            StartingPositions.LEFT_REVERSED,
            StartingPositions.RIGHT_REVERSED
        ) {
            stateCommandGroup(autoMode) {
                state(Mode.TEST_TRAJECTORIES, TestTrajectoriesRoutine())
                state(Mode.FORWARD_CARGO_SHIP, sequential {})
                state(Mode.DO_NOTHING, sequential {})
                state(Mode.BOTTOM_ROCKET, BottomRocketRoutine()())
                state(Mode.BOTTOM_ROCKET_2, BottomRocketRoutine2()())
                state(Mode.SIDE_CARGO_SHIP, CargoShipRoutine(CargoShipRoutine.Mode.SIDE)())
                state(Mode.HYBRID_LEFT, sequential {})
                state(Mode.HYBRID_RIGHT, sequential {})
            }
        }
        state(StartingPositions.CENTER) {
            stateCommandGroup(autoMode) {
                state(Mode.FORWARD_CARGO_SHIP, CargoShipRoutine(CargoShipRoutine.Mode.FRONT)())
                state(Mode.TEST_TRAJECTORIES, TestTrajectoriesRoutine())
                state(Mode.BOTTOM_ROCKET, sequential {})
                state(Mode.BOTTOM_ROCKET_2, sequential {})
                state(Mode.SIDE_CARGO_SHIP, sequential {})
                state(Mode.HYBRID_LEFT, HybridRoutine(HybridRoutine.Mode.LEFT))
                state(Mode.HYBRID_RIGHT, HybridRoutine(HybridRoutine.Mode.RIGHT))
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
        RIGHT(TrajectoryWaypoints.kSideStart),
        LEFT_REVERSED(TrajectoryWaypoints.kSideStartReversed.mirror),
        RIGHT_REVERSED(TrajectoryWaypoints.kSideStartReversed)
    }

    enum class Mode { TEST_TRAJECTORIES, BOTTOM_ROCKET, BOTTOM_ROCKET_2, FORWARD_CARGO_SHIP, SIDE_CARGO_SHIP, HYBRID_LEFT, HYBRID_RIGHT, DO_NOTHING }
}