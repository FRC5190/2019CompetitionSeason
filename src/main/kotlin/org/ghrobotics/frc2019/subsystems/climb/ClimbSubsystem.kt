package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.CANifier
import com.ctre.phoenix.motorcontrol.*
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.toNativeUnitAcceleration
import org.ghrobotics.lib.mathematics.units.nativeunits.toNativeUnitPosition
import org.ghrobotics.lib.mathematics.units.nativeunits.toNativeUnitVelocity
import org.ghrobotics.lib.util.CircularBuffer
import org.ghrobotics.lib.wrappers.ctre.AbstractFalconSRX
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

object ClimbSubsystem : FalconSubsystem(), EmergencyHandleable {

    private val frontWinchMaster = NativeFalconSRX(Constants.kClimbFrontWinchMasterId)
    private val backWinchMaster = NativeFalconSRX(Constants.kClimbBackWinchMasterId)

    private val wheelMaster = NativeFalconSRX(Constants.kClimbWheelId)

    private val rampsSolenoid = Solenoid(Constants.kPCMId, Constants.kRampsSolenoidId)

    private val rollingAverage = CircularBuffer(20)

    private val canifier = CANifier(Constants.kCanifier)

    private val allMotors: List<AbstractFalconSRX<*>>
    private val allMasters = listOf(frontWinchMaster, backWinchMaster)

    val rawFront get() = frontWinchMaster.selectedSensorPosition
    val rawBack get() = backWinchMaster.selectedSensorPosition

    var lidarRaw = 0.0
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

//    var ramps by Delegates.observable(false) { _, _, newValue -> rampsSolenoid.set(newValue) }

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
            master.softLimitForward = 24.inch.toNativeUnitPosition(Constants.kClimbWinchNativeUnitModel)
            master.softLimitForwardEnabled = false

            master.configRemoteFeedbackFilter(Constants.kPigeonIMUId, RemoteSensorSource.GadgeteerPigeon_Roll, 0)

            master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)
            master.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, 10)

            // Determine which pid slot affects which pid loop
            master.selectProfileSlot(0, 0)
            master.selectProfileSlot(1, 1)

            master.motionAcceleration =
                1.5.feet.acceleration.toNativeUnitAcceleration(Constants.kClimbWinchNativeUnitModel)
            master.motionCruiseVelocity = 1.0.feet.velocity.toNativeUnitVelocity(Constants.kClimbWinchNativeUnitModel)

            master.inverted = false
            master.encoderPhase = false

            master.selectedSensorPosition = 0
        }

        frontWinchMaster.configAuxPIDPolarity(false)
        backWinchMaster.configAuxPIDPolarity(true)

        defaultCommand = ManualClimbCommand()

        setClosedLoopGains()
    }

    private val tempPWMData = DoubleArray(2)

    override fun periodic() {

        if (frontWinchMaster.sensorCollection.isRevLimitSwitchClosed) frontWinchMaster.selectedSensorPosition =
            Constants.kClimbFrontWinchLimitSwitchTicks

        if (backWinchMaster.sensorCollection.isRevLimitSwitchClosed) backWinchMaster.selectedSensorPosition =
            Constants.kClimbBackWinchLimitSwitchTicks

        canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, tempPWMData)
        rollingAverage.add(tempPWMData[0])
        lidarRaw = rollingAverage.average
    }

    fun climbToHeight(frontPosition: Double, backPosition: Double) {
        frontWinchMaster.set(ControlMode.MotionMagic, frontPosition, DemandType.AuxPID, 0.0)
        backWinchMaster.set(ControlMode.MotionMagic, backPosition, DemandType.AuxPID, 0.0)
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

    enum class Winch(val motor: NativeFalconSRX) { FRONT(frontWinchMaster), BACK(backWinchMaster) }
}
