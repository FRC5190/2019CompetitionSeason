package org.ghrobotics.auto

import org.ghrobotics.frc2019.robot.auto.Trajectories
import org.junit.Test

class AutonomousTiming {
    @Test
    fun rocketAutoTimingTest() {
        val trajectories = Trajectories.sideStartToFarRocket.lastState.t.second +
            Trajectories.farRocketToLoadingStation.lastState.t.second +
            Trajectories.loadingStationToNearRocket.lastState.t.second
        println(trajectories)
    }
}