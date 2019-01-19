package org.ghrobotics.frc2019.subsystems.led

import edu.wpi.first.wpilibj.SerialPort
import kotlinx.coroutines.GlobalScope
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.utils.launchFrequency
import org.ghrobotics.lib.utils.monitor

object LEDSubsystem : FalconSubsystem(), EmergencyHandleable {

    private val arduino = SerialPort(9600, SerialPort.Port.kMXP)

    private val fault = LEDCommand(Mode.EMERGENCY)
    private val climb = LEDCommand(Mode.CLIMB)
    private val vision = LEDCommand(Mode.VISION)
    private val disabled = LEDCommand(Mode.DISABLED)

    private val climbingMonitor = { Controls.isClimbing && !Robot.emergencyActive }.monitor
    private val visionMonitor = { VisionDriveCommand.isActive && !Robot.emergencyActive }.monitor
    private val disabledMonitor = { Robot.isDisabled && !Robot.emergencyActive }.monitor

    init {
        defaultCommand = LEDCommand(Mode.NONE)
    }

    override fun lateInit() {
        GlobalScope.launchFrequency {
            climbingMonitor.onChange {
                if (it) {
                    climb.start()
                } else climb.stop()
            }
            visionMonitor.onChange {
                if (it) {
                    vision.start()
                } else vision.stop()
            }
            disabledMonitor.onChange {
                if (it) {
                    disabled.start()
                } else disabled.stop()
            }
        }
        disabled.start()
    }

    fun set(mode: Mode) {
        arduino.writeString(mode.value.toString())
    }

    override fun activateEmergency() = fault.start()
    override fun recoverFromEmergency() = fault.stop()

    enum class Mode(val value: Int) {
        CLIMB(4), VISION(3), DISABLED(1), EMERGENCY(2), NONE(0)
    }
}