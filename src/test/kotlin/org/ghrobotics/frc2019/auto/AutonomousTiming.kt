package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.auto.routines.BottomRocketRoutine
import org.ghrobotics.frc2019.auto.routines.BottomRocketRoutine2
import org.junit.Test

class AutonomousTiming {

    @Test
    fun bottomRocketTimer() {
        val time = BottomRocketRoutine().duration.second
        println("Bottom Rocket Execution Time: $time")
    }


    @Test
    fun bottomRocket2Timer() {
        val time = BottomRocketRoutine2().duration.second
        println("Bottom Rocket 2 Execution Time: $time")
    }
}
