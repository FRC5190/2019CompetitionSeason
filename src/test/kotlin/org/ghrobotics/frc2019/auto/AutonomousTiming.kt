package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.auto.routines.BottomRocketRoutine
import org.ghrobotics.frc2019.auto.routines.NearRocketRoutine
import org.junit.Test

class AutonomousTiming {

    @Test
    fun hatchAndCargoRocketTimer() {
        val time = NearRocketRoutine().duration.second
        println("Hatch and Cargo Rocket Execution Time: $time")
    }

    @Test
    fun bottomRocketTimer() {
        val time = BottomRocketRoutine().duration.second
        println("Bottom Rocket Execution Time: $time")
    }
}
