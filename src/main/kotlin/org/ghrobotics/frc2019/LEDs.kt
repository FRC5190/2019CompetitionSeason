package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.SerialPort
import org.ghrobotics.frc2019.subsystems.drive.VisionDriveCommand
import kotlin.concurrent.thread

object LEDs {

    private var wantedLEDMode = Mode.NONE

    init {
        thread {
            while (true) {
                try {
                    val port = SerialPort(9600, SerialPort.Port.kMXP)
                    port.setTimeout(0.5190)
                    var currentLEDMode: Mode? = null
                    @Suppress("ConvertTryFinallyToUseCall")
                    try {
                        while (true) {
                            val newLEDMode = wantedLEDMode
                            if (currentLEDMode != newLEDMode) {
                                port.writeString("${newLEDMode.value}\n")
                                currentLEDMode = newLEDMode
                            }
                            Thread.sleep(1000 / 50)
                        }
                    } finally {
                        port.close()
                    }
                } catch (e: Throwable) {
                    e.printStackTrace()
                    Thread.sleep(5000)
                }
            }
        }
    }

    fun update() {
        wantedLEDMode = when {
            Robot.emergencyActive -> Mode.EMERGENCY
            Controls.isClimbing -> Mode.CLIMB
            Robot.isDisabled -> Mode.DISABLED
            VisionDriveCommand.isActive -> Mode.VISION
            else -> Mode.NONE
        }
    }

    enum class Mode(val value: Int) {
        CLIMB(4),
        VISION(3),
        DISABLED(1),
        EMERGENCY(2),
        NONE(0)
    }
}