package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.auto.routines.CargoShipRoutine
import org.ghrobotics.frc2019.auto.routines.RocketRoutine
import org.junit.Test

class AutonomousTiming {
    @Test
    fun forwardCargoShipTimer() {
        val time = CargoShipRoutine().duration.second
        println("Forward Cargo Ship Execution Time: $time")
    }

    @Test
    fun hatchAndCargoRocketTimer() {
        val time = RocketRoutine().duration.second
        println("Hatch and Cargo Rocket Execution Time: $time")
    }
}
