package org.ghrobotics.frc2019.auto

import org.junit.Test

class AutonomousTiming {
    @Test
    fun doubleHatchRocketAutoTimer() {
        val time = org.ghrobotics.frc2019.auto.Trajectories.sideStartToNearRocketHatch.lastState.t.second +
            org.ghrobotics.frc2019.auto.Trajectories.nearRocketHatchToLoadingStation.lastState.t.second +
            org.ghrobotics.frc2019.auto.Trajectories.loadingStationToFarRocketHatch.lastState.t.second +
            org.ghrobotics.frc2019.auto.Trajectories.farRocketHatchToCargoBall1.lastState.t.second
        println("Double Hatch Rocket Execution Time: $time")
    }

    @Test
    fun forwardCargoShipAutoTimer() {
        val time = org.ghrobotics.frc2019.auto.Trajectories.centerStartToLeftForwardCargoShip.lastState.t.second +
            org.ghrobotics.frc2019.auto.Trajectories.leftForwardCargoShipToLoadingStation.lastState.t.second +
            org.ghrobotics.frc2019.auto.Trajectories.loadingStationToRightForwardCargoShip.lastState.t.second
        println("Forward Cargo Ship Execution Time: $time")
    }
}