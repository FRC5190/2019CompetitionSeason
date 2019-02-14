package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.auto.routines.ForwardCargoShipRoutine
import org.ghrobotics.frc2019.auto.routines.HatchAndCargoRocketRoutine
import org.ghrobotics.frc2019.auto.routines.HighHatchesRocketRoutine
import org.junit.Test

class AutonomousTiming {
    @Test
    fun highHatchesRocketTimer() {
        val time = HighHatchesRocketRoutine().duration.second
        println("High Hatches Rocket Execution Time: $time")
    }

    @Test
    fun forwardCargoShipTimer() {
        val time = ForwardCargoShipRoutine().duration.second
        println("Forward Cargo Ship Execution Time: $time")
    }

    @Test
    fun hatchAndCargoRocketTimer() {
        val time = HatchAndCargoRocketRoutine().duration.second
        println("Hatch and Cargo Rocket Execution Time: $time")
    }
}
