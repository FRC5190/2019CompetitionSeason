 package org.ghrobotics.frc2019.auto

import org.junit.Test

class AutonomousTiming {
    @Test
    fun doubleHatchRocketAutoTimer() {
        val time = Trajectories.sideStartToNearRocketHatch.lastState.t.second +
            Trajectories.nearRocketHatchToLoadingStation.lastState.t.second +
            Trajectories.loadingStationToFarRocketHatch.lastState.t.second
        println("High Hatches Rocket Execution Time: $time")
    }

    @Test
    fun forwardCargoShipAutoTimer() {
        val time = Trajectories.centerStartToLeftForwardCargoShip.lastState.t.second +
            Trajectories.leftForwardCargoShipToLoadingStation.lastState.t.second +
            Trajectories.loadingStationToRightForwardCargoShip.lastState.t.second
        println("Forward Cargo Ship Execution Time: $time")
    }

    @Test
    fun hatchAndCargoRocketAutoTimer() {
        val time = Trajectories.sideStartToNearRocketHatch.lastState.t.second +
            Trajectories.nearRocketHatchToLoadingStation.lastState.t.second +
            Trajectories.loadingStationToNearRocketHatch.lastState.t.second +
            Trajectories.nearRocketHatchToCargoBall.lastState.t.second +
            Trajectories.cargoBallToForcedNearSideRocketBay.lastState.t.second

        println("Hatch and Cargo Rocket Execution Time: $time")
    }
}