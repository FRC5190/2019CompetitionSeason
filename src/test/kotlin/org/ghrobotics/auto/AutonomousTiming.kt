package org.ghrobotics.auto

import org.ghrobotics.frc2019.robot.auto.Trajectories
import org.junit.Test

class AutonomousTiming {
    @Test
    fun doubleHatchRocketAutoTimer() {
        val trajectories = Trajectories.sideStartToFarRocket.lastState.t.second +
            Trajectories.farRocketToLoadingStation.lastState.t.second +
            Trajectories.loadingStationToNearRocket.lastState.t.second
        println(trajectories)
    }

    @Test
    fun forwardCargoShipAutoTimer() {
        val trajectories = Trajectories.centerStartToLeftForwardCargoShip.lastState.t.second +
            Trajectories.leftForwardCargoShipToLoadingStation.lastState.t.second +
            Trajectories.loadingStationToRightForwardCargoShip.lastState.t.second
        println(trajectories)
    }
}