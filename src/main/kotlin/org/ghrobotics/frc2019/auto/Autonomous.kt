package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.robot.auto.routines.baselineRoutine
import org.ghrobotics.frc2019.robot.auto.routines.characterizationRoutine
import org.ghrobotics.frc2019.robot.auto.routines.doubleHatchRocketRoutine
import org.ghrobotics.frc2019.robot.auto.routines.forwardCargoShipRoutine
import org.ghrobotics.frc2019.robot.subsystems.drive.DriveSubsystem
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

    private val autoMode = { org.ghrobotics.frc2019.Network.autoModeChooser.selected }
    val startingPosition = { org.ghrobotics.frc2019.Network.startingPositionChooser.selected }

    private var configValid = Source(true)
    private val isReady = { org.ghrobotics.frc2019.Robot.isAutonomous && org.ghrobotics.frc2019.Robot.isEnabled } and org.ghrobotics.frc2019.auto.Autonomous.configValid


    private val invalidOptionRoutine
        get() = sequential {
            +InstantRunnableCommand {
                println("[Autonomous] Invalid Option for this Starting Configuration. Running Baseline.")
            }
            +baselineRoutine()
        }

    // Autonomous Master Group
    private val JUST = stateCommandGroup(org.ghrobotics.frc2019.auto.Autonomous.startingPosition) {
        state(org.ghrobotics.frc2019.auto.StartingPositions.LEFT, org.ghrobotics.frc2019.auto.StartingPositions.RIGHT) {
            stateCommandGroup(org.ghrobotics.frc2019.auto.Autonomous.autoMode) {
                state(org.ghrobotics.frc2019.auto.AutoMode.DOUBLE_HATCH_ROCKET, doubleHatchRocketRoutine())
                state(org.ghrobotics.frc2019.auto.AutoMode.BASELINE, baselineRoutine())
                state(org.ghrobotics.frc2019.auto.AutoMode.CHARACTERIZE, characterizationRoutine())

                state(
                    org.ghrobotics.frc2019.auto.AutoMode.FORWARD_CARGO_SHIP,
                    org.ghrobotics.frc2019.auto.Autonomous.invalidOptionRoutine
                )
            }
        }
        state(org.ghrobotics.frc2019.auto.StartingPositions.CENTER) {
            stateCommandGroup(org.ghrobotics.frc2019.auto.Autonomous.autoMode) {
                state(org.ghrobotics.frc2019.auto.AutoMode.FORWARD_CARGO_SHIP, forwardCargoShipRoutine())
                state(org.ghrobotics.frc2019.auto.AutoMode.BASELINE, baselineRoutine())
                state(org.ghrobotics.frc2019.auto.AutoMode.CHARACTERIZE, characterizationRoutine())

                state(
                    org.ghrobotics.frc2019.auto.AutoMode.DOUBLE_HATCH_ROCKET,
                    org.ghrobotics.frc2019.auto.Autonomous.invalidOptionRoutine
                )
            }
        }
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = org.ghrobotics.frc2019.auto.Autonomous.startingPosition.monitor
    private val isReadyMonitor = org.ghrobotics.frc2019.auto.Autonomous.isReady.monitor
    private val modeMonitor = { org.ghrobotics.frc2019.Robot.currentMode }.monitor

    fun update() {
        org.ghrobotics.frc2019.auto.Autonomous.startingPositionMonitor.onChange { DriveSubsystem.localization.reset(it.pose) }
        org.ghrobotics.frc2019.auto.Autonomous.isReadyMonitor.onChangeToTrue { org.ghrobotics.frc2019.auto.Autonomous.JUST S3ND org.ghrobotics.frc2019.auto.Autonomous.IT }
        org.ghrobotics.frc2019.auto.Autonomous.modeMonitor.onChange { newValue ->
            if (newValue != FalconRobotBase.Mode.AUTONOMOUS) org.ghrobotics.frc2019.auto.Autonomous.JUST.stop()
        }
    }
}

enum class StartingPositions(val pose: Pose2d) {
    LEFT(org.ghrobotics.frc2019.auto.Trajectories.kSideStart.mirror),
    CENTER(org.ghrobotics.frc2019.auto.Trajectories.kCenterStart),
    RIGHT(org.ghrobotics.frc2019.auto.Trajectories.kSideStart)
}

enum class AutoMode { CHARACTERIZE, DOUBLE_HATCH_ROCKET, FORWARD_CARGO_SHIP, BASELINE }
