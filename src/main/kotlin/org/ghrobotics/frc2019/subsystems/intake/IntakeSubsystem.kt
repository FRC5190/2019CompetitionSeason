package org.ghrobotics.frc2019.subsystems.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.sensors.asSource
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

//object IntakeSubsystem : FalconSubsystem() {
//    private val intakeCargoMaster = NativeFalconSRX(Constants.kIntakeCargoId)
//    private val pigeon = PigeonIMU(intakeCargoMaster)
//    val pigeonSource = pigeon.asSource()
//
//    private val pushHatchSolenoid = DoubleSolenoid(
//        Constants.kPCMId,
//        Constants.kIntakePushHatchSolenoidForwardId,
//        Constants.kIntakePushHatchSolenoidReverseId
//    )
//    private val holdHatchSolenoid = Solenoid(Constants.kPCMId, Constants.kIntakeHoldHatchSolenoidId)
//
//    var wantedPercentOutput = 0.0
//    var wantedHoldHatchSolenoidState = HoldHatchSolenoidState.HOLD
//    var wantedPushHatchSolenoidState = PushHatchSolenoidState.EXIST
//
//    // PERIODIC
//    var isSeeingCargo: Boolean = false
//        private set
//    var isHoldingCargo: Boolean = false
//        private set
//    var isHoldingHatch: Boolean = false
//        private set
//    var holdHatchSolenoidState = HoldHatchSolenoidState.HOLD
//        private set
//    var pushHatchSolenoidState = PushHatchSolenoidState.USEFUL
//        private set
//
//    // DEBUG PERIODIC
//    var robotPitch = 0.degree
//        private set
//
//    private val tempYPR = DoubleArray(3)
//
//    init {
//        pigeon.setTemperatureCompensationDisable(true)
//
//        with(intakeCargoMaster) {
//            voltageCompensationSaturation = 12.volt
//            voltageCompensationEnabled = true
//
//            continuousCurrentLimit = 18.amp
//            currentLimitingEnabled = true
//
//            brakeMode = NeutralMode.Coast
//        }
//    }
//
//    override fun periodic() {
//        // PERIODIC
//        isSeeingCargo = intakeCargoMaster.outputCurrent > 7.0
//        if (isSeeingCargo) {
//            isHoldingCargo = true
//        } else if (wantedPercentOutput < 0) {
//            isHoldingCargo = false
//        }
//        isHoldingHatch = false
//
//        // DEBUG PERIODIC
//        if (Robot.shouldDebug) {
//            pigeon.getYawPitchRoll(tempYPR)
//            robotPitch = tempYPR[2].degree
//        }
//
//        intakeCargoMaster.set(ControlMode.PercentOutput, wantedPercentOutput)
//        if (wantedHoldHatchSolenoidState != holdHatchSolenoidState) {
//            holdHatchSolenoid.set(wantedHoldHatchSolenoidState == IntakeSubsystem.HoldHatchSolenoidState.HOLD)
//            holdHatchSolenoidState = wantedHoldHatchSolenoidState
//        }
//        if (wantedPushHatchSolenoidState != pushHatchSolenoidState) {
//            pushHatchSolenoid.set(
//                if (wantedPushHatchSolenoidState == IntakeSubsystem.PushHatchSolenoidState.EXIST) {
//                    DoubleSolenoid.Value.kForward
//                } else {
//                    DoubleSolenoid.Value.kReverse
//                }
//            )
//            pushHatchSolenoidState = wantedPushHatchSolenoidState
//        }
//    }
//
//    override fun zeroOutputs() {
//        wantedPercentOutput = 0.0
//    }
//
//    enum class HoldHatchSolenoidState { HOLD, PLACE }
//    enum class PushHatchSolenoidState { USEFUL, EXIST }
//}


object IntakeSubsystem : FalconSubsystem() {
    private val intakeMaster = NativeFalconSRX(Constants.kIntakeLeftId)
    private val pigeon = PigeonIMU(intakeMaster)
    val pigeonSource = pigeon.asSource()

    private val extensionSolenoid = DoubleSolenoid(
        Constants.kPCMId,
        Constants.kIntakeExtensionSolenoidForwardId,
        Constants.kIntakeExtensionSolenoidReverseId
    )
    private val launcherSolenoid = Solenoid(Constants.kPCMId, Constants.kIntakeLauncherSolenoidId)

    private val leftBallSensor = AnalogInput(Constants.kLeftBallSensorId)
    private val rightBallSensor = AnalogInput(Constants.kRightBallSensorId)

    private val extensionLimitSwitch = DigitalOutput(Constants.kIntakeExtensionLimitSwitch)

    var wantedPercentOutput = 0.0
    var wantedExtensionSolenoidState = ExtensionSolenoidState.EXTENDED
    var wantedLauncherSolenoidState = false

    // PERIODIC
    var isSeeingCargo: Boolean = false
        private set
    var isFullyExtended: Boolean = false
        private set
    var isHoldingCargo: Boolean = false
        private set
    var isHoldingHatch: Boolean = false
        private set
    var extensionSolenoidState = ExtensionSolenoidState.RETRACTED
        private set
    var launcherSolenoidState: Boolean = true
        private set

    // DEBUG PERIODIC
    var robotPitch = 0.degree
        private set

    private val tempYPR = DoubleArray(3)

    init {
        pigeon.setTemperatureCompensationDisable(true)

        val intakeSlave = NativeFalconSRX(Constants.kIntakeRightId)

        intakeSlave.follow(intakeMaster)

        intakeMaster.inverted = true
        intakeSlave.inverted = false

        listOf(intakeMaster, intakeSlave).forEach { motor ->
            motor.voltageCompensationSaturation = 12.volt
            motor.voltageCompensationEnabled = true

            motor.continuousCurrentLimit = 18.amp
            motor.currentLimitingEnabled = true

            motor.brakeMode = NeutralMode.Coast
        }
    }

    override fun periodic() {
        // PERIODIC
        isSeeingCargo = leftBallSensor.averageVoltage > 1.7 || rightBallSensor.averageVoltage > 1.2
        isFullyExtended = extensionLimitSwitch.get()
        isHoldingCargo = extensionSolenoidState == IntakeSubsystem.ExtensionSolenoidState.RETRACTED && isSeeingCargo
        isHoldingHatch = extensionSolenoidState == IntakeSubsystem.ExtensionSolenoidState.EXTENDED && !isFullyExtended

        // DEBUG PERIODIC
        if (Robot.shouldDebug) {
            pigeon.getYawPitchRoll(tempYPR)
            robotPitch = tempYPR[2].degree
        }

        intakeMaster.set(ControlMode.PercentOutput, wantedPercentOutput)
        if (wantedExtensionSolenoidState != extensionSolenoidState) {
            extensionSolenoid.set(
                if (wantedExtensionSolenoidState == IntakeSubsystem.ExtensionSolenoidState.EXTENDED) {
                    DoubleSolenoid.Value.kForward
                } else {
                    DoubleSolenoid.Value.kReverse
                }
            )
            extensionSolenoidState = wantedExtensionSolenoidState
        }
        if (wantedLauncherSolenoidState != launcherSolenoidState) {
            launcherSolenoid.set(wantedLauncherSolenoidState)
            launcherSolenoidState = wantedLauncherSolenoidState
        }
    }

    override fun zeroOutputs() {
        wantedPercentOutput = 0.0
    }

    enum class ExtensionSolenoidState { EXTENDED, RETRACTED }
}
