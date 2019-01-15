package org.ghrobotics.frc2019.auto

import org.junit.Test

class AutonomousTiming {
    @Test
    fun doubleHatchRocketAutoTimer() {
        val time = Trajectories.sideStartToNearRocketHatch.lastState.t.second +
            Trajectories.nearRocketHatchToLoadingStation.lastState.t.second +
            Trajectories.loadingStationToFarRocketHatch.lastState.t.second +
            Trajectories.farRocketHatchToCargoBall1.lastState.t.second
        println("Double Hatch Rocket Execution Time: $time")
    }

    @Test
    fun forwardCargoShipAutoTimer() {
        val time = Trajectories.centerStartToLeftForwardCargoShip.lastState.t.second +
            Trajectories.leftForwardCargoShipToLoadingStation.lastState.t.second +
            Trajectories.loadingStationToRightForwardCargoShip.lastState.t.second
        println("Forward Cargo Ship Execution Time: $time")
    }
}