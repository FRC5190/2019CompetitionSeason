package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.motorcontrol.*
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.drive.ManualDriveCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.wrappers.ctre.AbstractFalconSRX
import org.ghrobotics.lib.wrappers.ctre.FalconSRX
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX
import kotlin.properties.Delegates

object ClimbSubsystem : FalconSubsystem(), EmergencyHandleable {

    private val frontWinchMaster = FalconSRX(Constants.kClimbFrontWinchMasterId, Constants.kClimbWinchNativeUnitModel)
    private val backWinchMaster = FalconSRX(Constants.kClimbBackWinchMasterId, Constants.kClimbWinchNativeUnitModel)

    private val wheelMaster = NativeFalconSRX(Constants.kClimbWheelId)

    private val rampsSolenoid = Solenoid(Constants.kPCMId, Constants.kRampsSolenoidId)
    private val wheelSolenoid = Solenoid(Constants.kPCMId, Constants.kClimberWheelSolenoidId)

    private val allMotors: List<AbstractFalconSRX<*>>
    private val allMasters = listOf(frontWinchMaster, backWinchMaster)

    var wheelPercentOutput
        get() = wheelMaster.percentOutput
        set(value) {
            wheelMaster.percentOutput = value
        }

    var frontWinchPercentOutput: Double
        get() = frontWinchMaster.percentOutput
        set(value) {
            frontWinchMaster.percentOutput = value
        }

    var backWinchPercentOutput: Double
        get() = backWinchMaster.percentOutput
        set(value) {
            backWinchMaster.percentOutput = value
        }

    var frontWinchPosition
        get() = frontWinchMaster.sensorPosition
        set(value) {
            frontWinchMaster.set(ControlMode.MotionMagic, value)
        }

    var backWinchPosition
        get() = backWinchMaster.sensorPosition
        set(value) {
            backWinchMaster.set(ControlMode.MotionMagic, value)
        }

    var ramps by Delegates.observable(false) { _, _, newValue -> rampsSolenoid.set(newValue) }
    var wheel by Delegates.observable(false) { _, _, newValue -> wheelSolenoid.set(newValue) }

    val frontWinchCurrent get() = frontWinchMaster.outputCurrent
    val backWinchCurrent get() = backWinchMaster.outputCurrent

    val frontWinchVoltage get() = frontWinchMaster.motorOutputPercent * 12.0
    val backWinchVoltage get() = backWinchMaster.motorOutputPercent * 12.0

    val frontWinchVelocity get() = frontWinchMaster.sensorVelocity
    val backWinchVelocity get() = backWinchMaster.sensorVelocity

    init {
        val backWinchSlave = NativeFalconSRX(Constants.kClimbBackWinchSlaveId)
        val frontWinchSlave = NativeFalconSRX(Constants.kClimbFrontWinchSlaveId)

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
            // TODO configure soft limits
            /*master.softLimitForward = 0.nativeUnits
            master.softLimitReverse = 0.nativeUnits
            master.softLimitForwardEnabled = true
            master.softLimitReverseEnabled = true*/

            // Configure Pigeon as a remote sensor 0
            master.configRemoteFeedbackFilter(Constants.kPigeonIMUId, RemoteSensorSource.Pigeon_Pitch, 0, 10)
            master.configRemoteFeedbackFilter(Constants.kPigeonIMUId, RemoteSensorSource.CANifier_PWMInput0, 1, 10)

            // Configure Mag Encoder as main Feedback (Main PID)
            master.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 0, 10)
            master.configSelectedFeedbackCoefficient(1.0, 0, 10)

            // Configure Pigeon (Remote Feedback) as leveling Feedback (Aux PID)
            master.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, 10)
            master.configSelectedFeedbackCoefficient(1.0, 1, 10)

            // Determine which pid slot affects which pid loop
            master.selectProfileSlot(Constants.kClimbEncoderPIDSlot, 0)
            master.selectProfileSlot(Constants.kClimbLevelingPIDSlot, 1)

            master.inverted = false
            master.encoderPhase = false

            master.motionCruiseVelocity = Constants.kClimbWinchCruiseVelocity
            master.motionAcceleration = Constants.kClimbWinchAcceleration

            master.setSelectedSensorPosition(0)
        }

        frontWinchMaster.configAuxPIDPolarity(false)
        backWinchMaster.configAuxPIDPolarity(true)

        defaultCommand = ManualDriveCommand()

        setClosedLoopGains()
    }

    fun setPercentOutput(newOutput: Double, level: Boolean = false) {
        if (level) {
            frontWinchMaster.set(ControlMode.PercentOutput, newOutput, DemandType.AuxPID, 0.0)
            backWinchMaster.set(ControlMode.PercentOutput, newOutput, DemandType.AuxPID, 0.0)
        } else {
            frontWinchMaster.set(ControlMode.PercentOutput, newOutput)
            backWinchMaster.set(ControlMode.PercentOutput, newOutput)
        }
    }

    fun climbToHeight(height: Double) {
        frontWinchMaster.set(ControlMode.MotionMagic, height, DemandType.AuxPID, 0.0)
        backWinchMaster.set(ControlMode.MotionMagic, height, DemandType.AuxPID, 0.0)
//        if (height < Constants.kClimbLegHeightOffset) {
//            frontWinchMaster.set(ControlMode.MotionMagic, height + Constants.kClimbLegHeightOffset)
//            backWinchMaster.set(ControlMode.MotionMagic, height - Constants.kClimbLegHeightOffset)
//        } else {
//            frontWinchMaster.set(
//                ControlMode.MotionMagic,
//                height + Constants.kClimbLegHeightOffset, // Offset to angle the robot to wanted angle
//                DemandType.AuxPID,
//                Constants.kClimbAngle.degree // Angle wanted when climbing
//            )
//            backWinchMaster.set(
//                ControlMode.MotionMagic,
//                height - Constants.kClimbLegHeightOffset,// Offset to angle the robot to wanted angle
//                DemandType.AuxPID,
//                Constants.kClimbAngle.degree // Angle wanted when climbing
//            )
//        }
    }

    private fun setClosedLoopGains() {
        allMasters.forEach {
            it.config_kP(Constants.kClimbEncoderPIDSlot, Constants.kClimbWinchLidarKp, 10)

            it.config_kP(Constants.kClimbLevelingPIDSlot, Constants.kClimbWinchPitchKp, 10)
        }
    }

    private fun zeroClosedLoopGains() {
        allMasters.forEach {
            it.config_kP(Constants.kClimbEncoderPIDSlot, 0.0, 10)

            it.config_kP(Constants.kClimbLevelingPIDSlot, 0.0, 10)
        }
    }


    override fun activateEmergency() {
        zeroOutputs()

        zeroClosedLoopGains()
    }

    override fun recoverFromEmergency() = setClosedLoopGains()
}
