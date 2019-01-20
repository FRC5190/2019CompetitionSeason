package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.auto.routines.*
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
import org.ghrobotics.lib.wrappers.FalconRobotBase

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
    private val isReady = { Robot.isAutonomous && Robot.isEnabled } and configValid

    /**
     * Routine that gets executed if an invalid starting position + routine is selected.
     */
    private val invalidOptionRoutine
        get() = sequential {
            +InstantRunnableCommand {
                println("[Autonomous] Invalid Option for this Starting Configuration. Running Baseline.")
            }
            +baselineRoutine()
        }

    // Updates the monitors and starts the autonomous routine if everything is ready.
    fun update() {
        // Just send it when everything is ready.
        isReadyMonitor.onChangeToTrue {
            JUST S3ND IT
        }
        // Update localization.
        startingPositionMonitor.onChange { DriveSubsystem.localization.reset(it.pose) }
        // Stop the auto routine when we are not in autonomous anymore.
        modeMonitor.onChange { newValue ->
            if (newValue != FalconRobotBase.Mode.AUTONOMOUS) JUST.stop()
        }
    }

    // Autonomous Master Group
    private val JUST = stateCommandGroup(startingPosition) {
        state(StartingPositions.LEFT, StartingPositions.RIGHT) {
            stateCommandGroup(autoMode) {
                state(AutoMode.HIGH_HATCHES_ROCKET, highHatchesRocketRoutine())
                state(AutoMode.HATCH_AND_CARGO_ROCKET, hatchAndCargoRocketRoutine())
                state(AutoMode.BASELINE, baselineRoutine())
                state(AutoMode.CHARACTERIZE, characterizationRoutine())

                state(AutoMode.FORWARD_CARGO_SHIP, invalidOptionRoutine)
            }
        }
        state(StartingPositions.CENTER) {
            stateCommandGroup(autoMode) {
                state(AutoMode.FORWARD_CARGO_SHIP, forwardCargoShipRoutine())
                state(AutoMode.BASELINE, baselineRoutine())
                state(AutoMode.CHARACTERIZE, characterizationRoutine())

                state(AutoMode.HIGH_HATCHES_ROCKET, invalidOptionRoutine)
                state(AutoMode.HATCH_AND_CARGO_ROCKET, invalidOptionRoutine)
            }
        }
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = startingPosition.monitor
    private val isReadyMonitor = isReady.monitor
    private val modeMonitor = { Robot.currentMode }.monitor
}

enum class StartingPositions(val pose: Pose2d) {
    LEFT(Trajectories.kSideStart.mirror),
    CENTER(Trajectories.kCenterStart),
    RIGHT(Trajectories.kSideStart)
}

enum class AutoMode { CHARACTERIZE, HIGH_HATCHES_ROCKET, HATCH_AND_CARGO_ROCKET, FORWARD_CARGO_SHIP, BASELINE }
