package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.wrappers.ctre.AbstractFalconSRX
import org.ghrobotics.lib.wrappers.ctre.FalconSRX
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

object ClimbSubsystem : FalconSubsystem(), EmergencyHandleable {

    private val frontWinchMaster =
        FalconSRX(Constants.kClimbFrontWinchMasterId, Constants.kClimbWinchNativeUnitModel)

    private val backWinchMaster =
        FalconSRX(Constants.kClimbBackWinchMasterId, Constants.kClimbWinchNativeUnitModel)

    private val allMotors: List<AbstractFalconSRX<*>>
    private val allMasters = listOf(frontWinchMaster, backWinchMaster)

    var frontPercentOutput: Double
        get() = frontWinchMaster.percentOutput
        set(value) {
            frontWinchMaster.percentOutput = value
        }

    var backPercentOutput: Double
        get() = backWinchMaster.percentOutput
        set(value) {
            backWinchMaster.percentOutput = value
        }

    var frontPosition
        get() = frontWinchMaster.sensorPosition
        set(value) {
            frontWinchMaster.set(ControlMode.MotionMagic, value)
        }

    var backPosition
        get() = backWinchMaster.sensorPosition
        set(value) {
            backWinchMaster.set(ControlMode.MotionMagic, value)
        }

    val frontCurrent get() = frontWinchMaster.outputCurrent
    val backCurrent get() = backWinchMaster.outputCurrent

    val frontVelocity get() = frontWinchMaster.sensorVelocity
    val backVelocity get() = backWinchMaster.sensorVelocity

    init {
        val frontWinchSlave = NativeFalconSRX(Constants.kClimbFrontWinchSlaveId)
        val backWinchSlave = NativeFalconSRX(Constants.kClimbBackWinchSlaveId)

        allMotors = listOf(frontWinchMaster, frontWinchSlave, backWinchMaster, backWinchSlave)

        frontWinchSlave.follow(frontWinchMaster)
        backWinchSlave.follow(backWinchMaster)

        allMotors.forEach { motor ->
            motor.brakeMode = NeutralMode.Brake

            motor.voltageCompensationSaturation = 12.volt
            motor.voltageCompensationEnabled = true

            motor.peakCurrentLimit = 0.amp
            motor.peakCurrentLimitDuration = 0.millisecond
            motor.continuousCurrentLimit = Constants.kClimbWinchCurrentLimit
            motor.currentLimitingEnabled = true
        }

        allMasters.forEach { master ->
            // Soft Limits

            master.motionCruiseVelocity = Constants.kClimbWinchCruiseVelocity
            master.motionAcceleration = Constants.kClimbWinchAcceleration
        }

        defaultCommand = OpenLoopClimbCommand(0.0, 0.0)

        setClosedLoopGains()
    }

    private fun setClosedLoopGains() {
        /*
        allMasters.forEach {
            it.kP = Constants.kClimbWinchKp
        }
        */
    }

    private fun zeroClosedLoopGains() {
        allMasters.forEach { it.kP = 0.0 }
    }


    override fun activateEmergency() = zeroClosedLoopGains()
    override fun recoverFromEmergency() = setClosedLoopGains()
}