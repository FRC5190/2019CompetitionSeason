package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.auto.routines.BottomRocketRoutine
import org.ghrobotics.frc2019.auto.routines.ForwardCargoShipRoutine
import org.ghrobotics.frc2019.auto.routines.NearRocketRoutine
import org.ghrobotics.frc2019.auto.routines.SideCargoShipRoutine
import org.junit.Test

class AutonomousTiming {
    @Test
    fun forwardCargoShipTimer() {
        val time = ForwardCargoShipRoutine().duration.second
        println("Forward Cargo Ship Execution Time: $time")
    }

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
