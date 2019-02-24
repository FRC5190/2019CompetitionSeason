package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.CANifier
import com.ctre.phoenix.motorcontrol.*
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.toNativeUnitPosition
import org.ghrobotics.lib.util.CircularBuffer
import org.ghrobotics.lib.wrappers.FalconMotor
import org.ghrobotics.lib.wrappers.ctre.AbstractFalconSRX
import org.ghrobotics.lib.wrappers.ctre.FalconSRX
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX
import kotlin.properties.Delegates

object ClimbSubsystem : FalconSubsystem(), EmergencyHandleable {

    val frontWinchMaster = FalconSRX(Constants.kClimbFrontWinchMasterId, Constants.kClimbWinchNativeUnitModel)
    val backWinchMaster = FalconSRX(Constants.kClimbBackWinchMasterId, Constants.kClimbWinchNativeUnitModel)

    private val wheelMaster = NativeFalconSRX(Constants.kClimbWheelId)

    private val rampsSolenoid = Solenoid(Constants.kPCMId, Constants.kRampsSolenoidId)

    val rollingAverage = CircularBuffer(20)

    private val canifier = CANifier(Constants.kCanifier)

    private val allMotors: List<AbstractFalconSRX<*>>
    private val allMasters = listOf<FalconSRX<Length>>(frontWinchMaster, backWinchMaster)

    val rawFront get() = frontWinchMaster.selectedSensorPosition
    val rawBack get() = backWinchMaster.selectedSensorPosition

    var lidarRaw = 0.0
        private set

    var lidarRawHeight = 0.0
        private set

    var lidarHeight = 0.meter
        private set

    var frontWinchHeightFromLidar = 0.meter
        private set

    var backWinchHeightFromLidar = 0.meter
        private set

    var robotHeight = 0.meter
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
            master.softLimitForward = 24.inch.toNativeUnitPosition(Constants.kClimbWinchNativeUnitModel)
            master.softLimitForwardEnabled = false

            master.configRemoteFeedbackFilter(Constants.kPigeonIMUId, RemoteSensorSource.GadgeteerPigeon_Roll, 0)

            master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)
            master.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, 10)

            // Determine which pid slot affects which pid loop
            master.selectProfileSlot(0, 0)
            master.selectProfileSlot(1, 1)

            master.motionAcceleration = 1.5.feet.acceleration
            master.motionCruiseVelocity = 1.0.feet.velocity

            master.inverted = false
            master.encoderPhase = false

            master.selectedSensorPosition = 0
        }

        frontWinchMaster.configAuxPIDPolarity(false)
        backWinchMaster.configAuxPIDPolarity(true)

        defaultCommand = ManualClimbCommand()

        setClosedLoopGains()
    }

    fun climbToLevel3() {
        frontWinchMaster.set(ControlMode.MotionMagic, Constants.kFrontEncoderPositionL3, DemandType.AuxPID, 0.0)
        backWinchMaster.set(ControlMode.MotionMagic, Constants.kBackEncoderPositionL3, DemandType.AuxPID, 0.0)
    }

    private val tempPWMData = DoubleArray(2)

    var resetFront = false
    var resetBack = false

    override fun periodic() {

        if (frontWinchMaster.sensorCollection.isRevLimitSwitchClosed) frontWinchMaster.selectedSensorPosition = -1254
        if (backWinchMaster.sensorCollection.isRevLimitSwitchClosed) backWinchMaster.selectedSensorPosition = -2220

        canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, tempPWMData)
        rollingAverage.add(tempPWMData[0])
        lidarRaw = rollingAverage.average


        lidarRawHeight = (lidarRaw - Constants.kClimbLidarZero) * DriveSubsystem.pitch.cos

        lidarHeight = Constants.kClimbLidarScale * lidarRawHeight
        frontWinchHeightFromLidar =
            (lidarHeight.value + Constants.kClimbDistanceBetweenLegs.value * DriveSubsystem.pitch.sin).meter
        backWinchHeightFromLidar = lidarHeight

        robotHeight = Length((frontWinchHeightFromLidar.value + backWinchHeightFromLidar.value) / 2.0)

//        if (!resetFront && frontWinchHeightFromLidar > Constants.kEncoderLidarResetTolerance) {
//            frontWinchMaster.sensorPosition = 0.inch
//            println("Reset Front")
//            resetFront = true
//        }
//        if (frontWinchMaster.sensorCollection.isRevLimitSwitchClosed) {
//            resetFront = false
//        }
//
//        if (!resetBack && backWinchHeightFromLidar > Constants.kEncoderLidarResetTolerance) {
//            backWinchMaster.sensorPosition = 0.inch
//            println("Reset Back")
//            resetBack = true
//        }
//        if (backWinchMaster.sensorCollection.isRevLimitSwitchClosed) {
//            resetBack = false
//        }

        println("LE: ${frontWinchMaster.getSelectedSensorPosition(0)}, RE: ${backWinchMaster.getSelectedSensorPosition(0)}")

    }

    private fun setClosedLoopGains() {
        allMasters.forEach {
            it.config_kP(0, Constants.kClimbWinchPositionKp, 10)
            it.config_kP(1, Constants.kClimbWinchLevelingKp, 10)
        }
    }

    private fun zeroClosedLoopGains() {
        allMasters.forEach {
            it.config_kP(0, 0.0, 10)
            it.config_kP(1, 0.0, 10)
        }
    }

    override fun activateEmergency() {
        zeroOutputs()
        zeroClosedLoopGains()
    }

    override fun recoverFromEmergency() = setClosedLoopGains()

    enum class Winch(val motor: FalconSRX<Length>) { FRONT(frontWinchMaster), BACK(backWinchMaster) }
}
