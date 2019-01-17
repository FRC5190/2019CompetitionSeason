package org.ghrobotics.frc2019.subsystems.drive

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatusFrame
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.wrappers.ctre.FalconSRX

class DriveGearbox(
    masterId: Int,
    slaveOneId: Int,
    inverted: Boolean
) {
    val master = FalconSRX(masterId, Constants.kDriveNativeUnitModel)
    val slaveOne = FalconSRX(slaveOneId, Constants.kDriveNativeUnitModel)

    val allMotors = listOf(master, slaveOne)

    init {
        slaveOne.follow(master)
        // Configure Inversion
        master.inverted = inverted
        slaveOne.inverted = inverted

        // Configure Encoder
        master.feedbackSensor = FeedbackDevice.QuadEncoder
        master.encoderPhase = inverted
        master.sensorPosition = 0.meter

        allMotors.forEach { motor ->
            motor.peakForwardOutput = 1.0
            motor.peakReverseOutput = -1.0

            motor.nominalForwardOutput = 0.0
            motor.nominalReverseOutput = 0.0

            motor.brakeMode = NeutralMode.Brake

            motor.voltageCompensationSaturation = 12.volt
            motor.voltageCompensationEnabled = true

            motor.peakCurrentLimit = 0.amp
            motor.peakCurrentLimitDuration = 0.millisecond
            motor.continuousCurrentLimit = Constants.kDriveCurrentLimit
            motor.currentLimitingEnabled = true

            motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10)
        }

        setClosedLoopGains()
    }

    fun setClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.kP = Constants.kDriveKp
            motor.kD = Constants.kDriveKd
        }
    }

    fun zeroClosedLoopGains() {
        allMotors.forEach { motor ->
            motor.kP = 0.0
            motor.kD = 0.0
        }
    }
}