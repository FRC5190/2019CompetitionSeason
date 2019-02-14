package org.ghrobotics.frc2019.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.lib.mathematics.units.derivedunits.feetPerSecond
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.second
import org.junit.Test
import org.knowm.xchart.QuickChart
import kotlin.math.pow

class DriveModelTest {
    @Test
    fun testAccelerationCurve() {
        val voltage = DifferentialDrive.WheelState(12.0, 12.0)
        var velocity = DifferentialDrive.WheelState(0.0, 0.0)
        val dt = 0.01

        var t = 0.0

        val time = ArrayList<Double>()
        val speed = ArrayList<Double>()

        for (i in 0..500) {
            speed.add(velocity.left * Constants.kDriveWheelRadius.value)
            time.add(t)

            val predictedAcceleration =
                Constants.kDriveModel.solveForwardDynamics(velocity, voltage).wheelAcceleration
            velocity = DifferentialDrive.WheelState(
                velocity.left + predictedAcceleration.left * dt,
                velocity.right + predictedAcceleration.right * dt
            )

            t += dt
        }

        println("Max Speed: ${speed.max()}")

//        SwingWrapper(
//            QuickChart.getChart(
//                "Speed with Constant Voltage",
//                "T",
//                "V",
//                "Plot",
//                time.toDoubleArray(),
//                speed.toDoubleArray()
//            )
//        ).displayChart()
//        Thread.sleep(100000)
    }

    @Test
    fun testVoltage() {
        val voltage = Constants.kDriveModel.solveInverseDynamics(
            DifferentialDrive.ChassisState(12.feet.meter, 0.0),
            DifferentialDrive.ChassisState(10.feet.meter, 0.0)
        ).voltage

        println(voltage)
    }

    @Test
    fun getMaxAccelerationAt12Fps() {
        val acceleration = Constants.kDriveModel.getMinMaxAcceleration(
            DifferentialDrive.ChassisState(10.feet.meter, 0.0),
            0.0,
            12.0
        )
        println(acceleration.max.meter.feet)
    }

    @Test
    fun visualizeTrajectory() {
        val iterator = TrajectoryFactory.loadingStationToRocketF.iterator()

        val t = ArrayList<Double>()
        val v = ArrayList<Double>()

        while (!iterator.isDone) {
            val state = iterator.currentState
            t.add(state.state.t.second)
            v.add(state.state.velocity.absoluteValue.feetPerSecond)
            iterator.advance(0.02.second)
        }

        val chart = QuickChart.getChart(
            "Velocity", "t", "v", "Velocity", t.toDoubleArray(), v.toDoubleArray()
        )
//
//        SwingWrapper(chart).displayChart()
//        Thread.sleep(1000000)
    }

    @Test
    fun testTorque() {
        val torquePerVolt =
            Constants.kDriveWheelRadius.value.pow(2) * Constants.kRobotMass.value / (2.0 * Constants.kDriveRightKa)
        println(torquePerVolt * 12.0)
    }
}