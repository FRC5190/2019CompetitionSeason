package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.CANifier
import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.drive.ManualDriveCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.wrappers.ctre.AbstractFalconSRX
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX
import kotlin.properties.Delegates

object ClimbSubsystem : FalconSubsystem(), EmergencyHandleable {

    private val frontWinchMaster = NativeFalconSRX(Constants.kClimbFrontWinchMasterId)
    private val backWinchMaster = NativeFalconSRX(Constants.kClimbBackWinchMasterId)

    private val wheelMaster = NativeFalconSRX(Constants.kClimbWheelId)

    private val rampsSolenoid = Solenoid(Constants.kPCMId, Constants.kRampsSolenoidId)

    private val canifier = CANifier(Constants.kCanifier)

    private val allMotors: List<AbstractFalconSRX<*>>
    private val allMasters = listOf(frontWinchMaster, backWinchMaster)

    var lidarRaw = 0.0
        private set

    var lidarRawHeight = 0.0
        private set

    var lidarHeight = 0.meter
        private set

    var frontWinchHeight = 0.meter
        private set

    var backWinchHeight = 0.meter
        private set

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

    val frontWinchCurrent get() = frontWinchMaster.outputCurrent
    val backWinchCurrent get() = backWinchMaster.outputCurrent

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

            // Configure Mag Encoder as main Feedback (Main PID)
            master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)
            master.configSelectedFeedbackCoefficient(1.0, 0, 10)

            // Determine which pid slot affects which pid loop
            master.selectProfileSlot(0, 0)

            master.inverted = false
            master.encoderPhase = false

            master.selectedSensorPosition = 0
        }

        frontWinchMaster.configAuxPIDPolarity(false)
        backWinchMaster.configAuxPIDPolarity(true)

        defaultCommand = ManualDriveCommand()

        setClosedLoopGains()
    }

    fun setPercentOutput(newOutput: Double) {
        frontWinchMaster.set(ControlMode.PercentOutput, newOutput)
        backWinchMaster.set(ControlMode.PercentOutput, newOutput)
    }

    fun climbToHeight(height: Double) {
        frontWinchMaster.set(ControlMode.MotionMagic, height, DemandType.AuxPID, 0.0)
        backWinchMaster.set(ControlMode.MotionMagic, height, DemandType.AuxPID, 0.0)
    }

    private val tempPWMData = DoubleArray(2)

    override fun periodic() {
        canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, tempPWMData)
        lidarRaw = tempPWMData[0]

        lidarRawHeight = lidarRaw * DriveSubsystem.pitch.cos

        lidarHeight = Constants.kClimbLidarScale * lidarRawHeight
        frontWinchHeight =
            Length(lidarHeight.value + Constants.kClimbLidarDistanceFromFront.value * DriveSubsystem.pitch.sin)
        backWinchHeight =
            Length(lidarHeight.value - Constants.kClimbLidarDistanceFromBack.value * DriveSubsystem.pitch.sin)
    }

    private fun setClosedLoopGains() {
        allMasters.forEach {
            it.config_kP(0, Constants.kClimbWinchPositionKp, 10)
        }
    }

    private fun zeroClosedLoopGains() {
        allMasters.forEach {
            it.config_kP(0, 0.0, 10)
        }
    }


    override fun activateEmergency() {
        zeroOutputs()

        zeroClosedLoopGains()
    }

    override fun recoverFromEmergency() = setClosedLoopGains()
}
