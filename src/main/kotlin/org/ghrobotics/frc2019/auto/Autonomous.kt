package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.auto.routines.baselineRoutine
import org.ghrobotics.frc2019.auto.routines.characterizationRoutine
import org.ghrobotics.frc2019.auto.routines.doubleHatchRocketRoutine
import org.ghrobotics.frc2019.auto.routines.forwardCargoShipRoutine
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

object Autonomous {

    private val autoMode = { Network.autoModeChooser.selected }
    val startingPosition = { Network.startingPositionChooser.selected }

    private var configValid = Source(true)
    private val isReady = { Robot.isAutonomous && Robot.isEnabled } and configValid


    private val invalidOptionRoutine
        get() = sequential {
            +InstantRunnableCommand {
                println("[Autonomous] Invalid Option for this Starting Configuration. Running Baseline.")
            }
            +baselineRoutine()
        }

    // Autonomous Master Group
    private val JUST = stateCommandGroup(startingPosition) {
        state(StartingPositions.LEFT, StartingPositions.RIGHT) {
            stateCommandGroup(autoMode) {
                state(AutoMode.DOUBLE_HATCH_ROCKET, doubleHatchRocketRoutine())
                state(AutoMode.BASELINE, baselineRoutine())
                state(AutoMode.CHARACTERIZE, characterizationRoutine())

                state(
                    AutoMode.FORWARD_CARGO_SHIP,
                    invalidOptionRoutine
                )
            }
        }
        state(StartingPositions.CENTER) {
            stateCommandGroup(autoMode) {
                state(AutoMode.FORWARD_CARGO_SHIP, forwardCargoShipRoutine())
                state(AutoMode.BASELINE, baselineRoutine())
                state(AutoMode.CHARACTERIZE, characterizationRoutine())

                state(
                    AutoMode.DOUBLE_HATCH_ROCKET,
                    invalidOptionRoutine
                )
            }
        }
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = startingPosition.monitor
    private val isReadyMonitor = isReady.monitor
    private val modeMonitor = { Robot.currentMode }.monitor

    fun update() {
        startingPositionMonitor.onChange { DriveSubsystem.localization.reset(it.pose) }
        isReadyMonitor.onChangeToTrue { JUST S3ND IT }
        modeMonitor.onChange { newValue ->
            if (newValue != FalconRobotBase.Mode.AUTONOMOUS) JUST.stop()
        }
    }
}

enum class StartingPositions(val pose: Pose2d) {
    LEFT(Trajectories.kSideStart.mirror),
    CENTER(Trajectories.kCenterStart),
    RIGHT(Trajectories.kSideStart)
}

enum class AutoMode { CHARACTERIZE, DOUBLE_HATCH_ROCKET, FORWARD_CARGO_SHIP, BASELINE }
