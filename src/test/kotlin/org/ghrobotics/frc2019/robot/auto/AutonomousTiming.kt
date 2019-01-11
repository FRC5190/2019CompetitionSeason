package org.ghrobotics.frc2019.robot.auto

import org.junit.Test

class AutonomousTiming {
    @Test
    fun doubleHatchRocketAutoTimer() {
        val time = Trajectories.sideStartToFarRocket.lastState.t.second +
            Trajectories.farRocketToLoadingStation.lastState.t.second +
            Trajectories.loadingStationToNearRocket.lastState.t.second +
            Trajectories.nearRocketToCargoBall1.lastState.t.second +
            Trajectories.cargoBall1ToRocketBay.lastState.t.second
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