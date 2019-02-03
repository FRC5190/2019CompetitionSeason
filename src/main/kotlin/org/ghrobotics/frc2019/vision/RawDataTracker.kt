package org.ghrobotics.frc2019.vision

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second

object RawDataTracker {

    var bestTargetRawData: RawDataTarget? = null

    fun addSamples(time: Time, targets: List<Pose2d>) {
        val current = Timer.getFPGATimestamp().second
        if (time >= current) return

        if (targets.isNotEmpty()) {
            bestTargetRawData = RawDataTarget(time, targets.minBy { it.translation.norm }!!)
        }
    }
}

data class RawDataTarget(val timestamp: Time, val pose: Pose2d)