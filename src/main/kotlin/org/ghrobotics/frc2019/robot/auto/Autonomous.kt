package org.ghrobotics.frc2019.robot.auto

import kotlinx.coroutines.GlobalScope
import org.ghrobotics.lib.commands.S3ND
import org.ghrobotics.lib.commands.stateCommandGroup
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.*
import org.ghrobotics.lib.wrappers.FalconRobotBase
import org.ghrobotics.frc2019.robot.Network
import org.ghrobotics.frc2019.robot.Robot
import org.ghrobotics.frc2019.robot.auto.routines.routineCharacterization
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem

object Autonomous {

    val autoMode = { Network.autoModeChooser.selected }
    val startingPosition = { Network.startingPositionChooser.selected }

    private var configValid = Source(true)
    private val isReady = { Robot.isAutonomous && Robot.isEnabled } and configValid

    // Autonomous Master Group
    private val JUST = stateCommandGroup(autoMode) {
        state(AutoMode.REAL) {
            stateCommandGroup(startingPosition) {

            }
        }
        state(AutoMode.CHARACTERIZE, routineCharacterization())
    }

    init {
        @Suppress("LocalVariableName")
        val IT = ""

        val startingPositionMonitor = startingPosition.monitor
        val isReadyMonitor = isReady.monitor
        val modeMonitor = { Robot.currentMode }.monitor

        GlobalScope.launchFrequency {
            startingPositionMonitor.onChange { DriveSubsystem.localization.reset(it.pose) }
            isReadyMonitor.onChangeToTrue { JUST S3ND IT }
            modeMonitor.onChange { newValue ->
                if (newValue != FalconRobotBase.Mode.AUTONOMOUS) JUST.stop()
            }
        }
    }
}

/* TODO Find Actual Value */
enum class StartingPositions(val pose: Pose2d) {
    LEFT(Pose2d()),
    CENTER(Pose2d()),
    RIGHT(Pose2d())
}

enum class AutoMode { CHARACTERIZE, REAL }
