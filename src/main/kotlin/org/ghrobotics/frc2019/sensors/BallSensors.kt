package org.ghrobotics.frc2019.sensors

import edu.wpi.first.wpilibj.AnalogInput
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.utils.and
import org.ghrobotics.lib.utils.greaterThan

object BallSensors {

    private val leftBallSensor = AnalogInput(Constants.kLeftBallSensorId).let { { it.averageVoltage } }
    private val rightBallSensor = AnalogInput(Constants.kRightBallSensorId).let { { it.averageVoltage } }

    private const val kVoltThreshold = 0.9

    val ballIn = leftBallSensor.greaterThan(kVoltThreshold) and rightBallSensor.greaterThan(kVoltThreshold)
}