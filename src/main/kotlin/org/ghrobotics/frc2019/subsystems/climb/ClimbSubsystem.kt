package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.CANifier
import com.ctre.phoenix.motorcontrol.*
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DigitalInput
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunits.toNativeUnitAcceleration
import org.ghrobotics.lib.mathematics.units.nativeunits.toNativeUnitPosition
import org.ghrobotics.lib.mathematics.units.nativeunits.toNativeUnitVelocity
import org.ghrobotics.lib.util.CircularBuffer
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.wrappers.ctre.AbstractFalconSRX
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

object ClimbSubsystem : FalconSubsystem(), EmergencyHandleable {

    private val frontWinchMaster = NativeFalconSRX(Constants.kClimbFrontWinchMasterId)
    private val backWinchMaster = NativeFalconSRX(Constants.kClimbBackWinchMasterId)

    private val wheelMaster = NativeFalconSRX(Constants.kClimbWheelId)

    private val canifier = CANifier(Constants.kCanifierId)
    private val frontOnPlatformSensor = AnalogInput(Constants.kClimberSensorId)

    private val rollingLidarAverage = CircularBuffer(20)

    private val allMotors: List<AbstractFalconSRX<*>>
    private val allMasters = listOf(frontWinchMaster, backWinchMaster)

    var wantedWheelPercentOutput = 0.0
    var wantedFrontWinchState: ClimbLegState = ClimbLegState.Nothing
    var wantedBackWinchState: ClimbLegState = ClimbLegState.Nothing

    var currentFrontWinchState: ClimbLegState = ClimbLegState.Nothing
        private set
    var currentBackWinchState: ClimbLegState = ClimbLegState.Nothing
        private set

    // PERIODIC
    var rawFrontWinchPosition: Int = 0
        private set
    var rawBackWinchPosition: Int = 0
        private set
    var frontWinchPosition = 0.0
        private set
    var backWinchPosition = 0.0
        private set
    var lidarRawAveraged: Double = 0.0
        private set
    var frontOnPlatform: Boolean = false
        private set
    var isFrontReverseLimitSwitchClosed = false
        private set
    var isBackReverseLimitSwitchClosed = false
        private set

    // DEBUG PERIODIC
    var frontWinchCurrent = 0.0
        private set
    var backWinchCurrent = 0.0
        private set

    private var setLimits = false

    private val backHallEffectSensor = DigitalInput(Constants.kClimberHallEffectSensor)

    init {
        val backWinchSlave = NativeFalconSRX(Constants.kClimbBackWinchSlaveId)
        val frontWinchSlave = NativeFalconSRX(Constants.kClimbFrontWinchSlaveId)

        allMotors = listOf(frontWinchMaster, frontWinchSlave, backWinchMaster, backWinchSlave)

        frontWinchSlave.follow(frontWinchMaster)
        backWinchSlave.follow(backWinchMaster)

        frontWinchMaster.inverted = false
        frontWinchSlave.inverted = false

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
            master.softLimitForward = 24.inch.toNativeUnitPosition(Constants.kClimbBackWinchNativeUnitModel)
            master.softLimitForwardEnabled = false

            master.configRemoteFeedbackFilter(Constants.kPigeonIMUId, RemoteSensorSource.GadgeteerPigeon_Roll, 0)

            master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)
            master.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, 10)

            // Determine which pid slot affects which pid loop
            master.selectProfileSlot(0, 0)
            master.selectProfileSlot(2, 1)

            master.encoderPhase = false

            master.selectedSensorPosition = 0
        }

        frontWinchMaster.motionAcceleration =
            0.8.feet.acceleration.toNativeUnitAcceleration(Constants.kClimbFrontWinchNativeUnitModel)
        frontWinchMaster.motionCruiseVelocity =
            1.65.feet.velocity.toNativeUnitVelocity(Constants.kClimbFrontWinchNativeUnitModel)

        backWinchMaster.motionAcceleration =
            0.65.feet.acceleration.toNativeUnitAcceleration(Constants.kClimbBackWinchNativeUnitModel)
        backWinchMaster.motionCruiseVelocity =
            1.2.feet.velocity.toNativeUnitVelocity(Constants.kClimbBackWinchNativeUnitModel)

        frontWinchMaster.configAuxPIDPolarity(false)
        backWinchMaster.configAuxPIDPolarity(true)

        defaultCommand = ManualClimbCommand()

        setClosedLoopGains()
    }

    private val tempPWMData = DoubleArray(2)

    override fun periodic() {
        // PERIODIC
        rawFrontWinchPosition = frontWinchMaster.selectedSensorPosition
        rawBackWinchPosition = backWinchMaster.selectedSensorPosition

        frontWinchPosition = Constants.kClimbFrontWinchNativeUnitModel
            .fromNativeUnitPosition(rawFrontWinchPosition.toDouble())
        backWinchPosition = Constants.kClimbBackWinchNativeUnitModel
            .fromNativeUnitPosition(rawBackWinchPosition.toDouble())

//        println(frontOnPlatformSensor.averageVoltage)

        canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0, tempPWMData)
        rollingLidarAverage.add(tempPWMData[0])
        lidarRawAveraged = rollingLidarAverage.average

        isFrontReverseLimitSwitchClosed = frontWinchMaster.sensorCollection.isRevLimitSwitchClosed
        isBackReverseLimitSwitchClosed = !backHallEffectSensor.get()

        frontOnPlatform = frontOnPlatformSensor.averageVoltage > 3.4

        // DEBUG PERIODIC
        if (Robot.shouldDebug) {
            frontWinchCurrent = frontWinchMaster.outputCurrent
            backWinchCurrent = backWinchMaster.outputCurrent
        }

        val wantedFrontWinchState = this.wantedFrontWinchState
        val currentFrontWinchState = this.currentFrontWinchState
        this.currentFrontWinchState = wantedFrontWinchState
        updateLeg(
            frontWinchMaster,
            Constants.kClimbFrontWinchNativeUnitModel,
            wantedFrontWinchState,
            currentFrontWinchState
        )

        val wantedBackWinchState = this.wantedBackWinchState
        val currentBackWinchState = this.currentBackWinchState
        this.currentBackWinchState = wantedBackWinchState

        if (isBackReverseLimitSwitchClosed && !setLimits) {
            backWinchMaster.nominalReverseOutput = 0.0
            backWinchMaster.peakReverseOutput = 0.0
            setLimits = true
        }
        if (!isBackReverseLimitSwitchClosed && setLimits) {
            backWinchMaster.nominalReverseOutput = 0.0
            backWinchMaster.peakReverseOutput = -1.0
            setLimits = false
        }

        updateLeg(
            backWinchMaster,
            Constants.kClimbBackWinchNativeUnitModel,
            wantedBackWinchState,
            currentBackWinchState
        )

        if (isFrontReverseLimitSwitchClosed) frontWinchMaster.selectedSensorPosition = 0
        if (isBackReverseLimitSwitchClosed) backWinchMaster.selectedSensorPosition = 0

        wheelMaster.set(ControlMode.PercentOutput, wantedWheelPercentOutput)
    }

    private fun updateLeg(
        winchMaster: NativeFalconSRX,
        model: NativeUnitModel<Length>,
        wantedState: ClimbLegState,
        currentState: ClimbLegState
    ) {
        when (wantedState) {
            is ClimbLegState.Nothing -> {
                winchMaster.set(ControlMode.Disabled, 0.0)
            }
            is ClimbLegState.MotionMagic -> {
                if (currentState !is ClimbLegState.MotionMagic) {
                    winchMaster.selectProfileSlot(0, 0)
                } else if (currentState.position == wantedState.position) return

                winchMaster.set(
                    ControlMode.MotionMagic,
                    model.toNativeUnitPosition(wantedState.position)
                )
            }
            is ClimbLegState.Position -> {
                if (currentState !is ClimbLegState.Position) {
                    winchMaster.selectProfileSlot(1, 0)
                } else if (currentState.position == wantedState.position) return

                winchMaster.set(
                    ControlMode.Position,
                    model.toNativeUnitPosition(wantedState.position)
                )
            }
            is ClimbLegState.Climb -> {
                if (currentState !is ClimbLegState.Climb) {
                    winchMaster.selectProfileSlot(0, 0)
                } else if (currentState.position == wantedState.position) return

                winchMaster.set(
                    ControlMode.MotionMagic,
                    model.toNativeUnitPosition(wantedState.position),
                    DemandType.AuxPID,
                    0.0
                )
            }
            is ClimbLegState.OpenLoop -> {
                winchMaster.set(
                    ControlMode.PercentOutput,
                    wantedState.output()
                )
            }
        }
    }

    private fun setClosedLoopGains() {
        allMasters.forEach {
            // MotionMagic PID
            it.config_kP(0, Constants.kClimbWinchPositionKp, 10)

            // Position PID
            it.config_kP(1, Constants.kClimbWinchPositionKp, 10)

            // Leveling Aux
            it.config_kP(2, Constants.kClimbWinchLevelingKp, 10)
            it.config_kD(2, Constants.kClimbWinchLevelingKd, 10)
        }
    }

    private fun zeroClosedLoopGains() {
        allMasters.forEach {
            // Motion Magic PID
            it.config_kP(0, 0.0, 10)

            // Position PID
            it.config_kP(1, 0.0, 10)

            // Leveling Aux
            it.config_kP(2, 0.0, 10)
            it.config_kD(2, 0.0, 10)
        }
    }

    override fun zeroOutputs() {
        wantedFrontWinchState = ClimbLegState.Nothing
        wantedBackWinchState = ClimbLegState.Nothing
        wantedWheelPercentOutput = 0.0
    }

    override fun activateEmergency() {
        zeroOutputs()
        zeroClosedLoopGains()
    }

    override fun recoverFromEmergency() = setClosedLoopGains()

    sealed class ClimbLegState {
        object Nothing : ClimbLegState()
        class OpenLoop(val output: Source<Double>) : ClimbLegState()
        abstract class SetPointState(val position: Double) : ClimbLegState()
        class MotionMagic(position: Double) : SetPointState(position)
        class Position(position: Double) : SetPointState(position)
        class Climb(position: Double) : SetPointState(position)
    }
}
