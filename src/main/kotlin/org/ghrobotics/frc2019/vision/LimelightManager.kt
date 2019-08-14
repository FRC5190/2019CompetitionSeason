package org.ghrobotics.frc2019.vision

import edu.wpi.first.wpilibj.Notifier
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.InstantRunnableCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.second

object LimelightManager {
    private val limelight_ = Limelight(44.3.inch, (-25).degree, 29.inch)
    private val notifier_ = Notifier(::update)

    var isAlive: Boolean = false
        private set

    init {
        notifier_.startPeriodic(0.02)
    }

    fun turnOnLED() = limelight_.turnOnLED()
    fun turnOffLED() = limelight_.turnOffLED()
    fun blinkLEDs() = limelight_.blinkLEDs()

    private fun update() {
        isAlive = limelight_.isAlive
        limelight_.updateTargetTracker()
    }

    val blinkCommand
        get() = sequential {
            +InstantRunnableCommand { blinkLEDs() }
            +DelayCommand(1.5.second)
            +InstantRunnableCommand { turnOffLED() }
        }
}