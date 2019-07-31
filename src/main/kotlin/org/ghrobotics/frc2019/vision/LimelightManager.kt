package org.ghrobotics.frc2019.vision

import edu.wpi.first.wpilibj.Notifier
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch

object LimelightManager {
    private val limelight_ = Limelight(46.inch, (-30).degree, 29.inch)
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
}