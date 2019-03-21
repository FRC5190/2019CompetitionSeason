package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.meter
import org.junit.Test

class FusedLocalizationTest {
    @Test
    fun testFusedLocalization() {
        val static = Pose2d(0.meter, 4.meter, 0.degree)
        val ttarget = Pose2d(0.meter, 3.meter, 45.degree)
        val historicalPose = Pose2d(5.meter, 5.meter, 17.degree)

        val toTarget = Pose2d(
            Translation2d(
                ttarget.translation.x - historicalPose.translation.x,
                ttarget.translation.y - historicalPose.translation.y
            )
        )

        val visionHistoricalPose = Pose2d(
            Translation2d(
                static.translation.x - toTarget.translation.x,
                static.translation.y - toTarget.translation.y
            )
        )
        println("$visionHistoricalPose" + "${visionHistoricalPose.rotation.degree}")
    }
}