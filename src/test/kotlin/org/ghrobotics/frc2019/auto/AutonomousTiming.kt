package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.auto.routines.BottomRocketRoutine
import org.junit.Test

class AutonomousTiming {

    @Test
    fun bottomRocketTimer() {
        val time = BottomRocketRoutine().duration.second
        println("Bottom Rocket Execution Time: $time")
    }
}
