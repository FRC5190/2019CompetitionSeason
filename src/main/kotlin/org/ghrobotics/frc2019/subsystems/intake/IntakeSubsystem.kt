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
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.sensors.asSource
import org.ghrobotics.lib.wrappers.ctre.NativeFalconSRX

//object IntakeSubsystem : FalconSubsystem() {
//    private val intakeCargoMaster = NativeFalconSRX(Constants.kIntakeCargoId)
//    private val pigeon = PigeonIMU(intakeCargoMaster)
//    val pigeonSource = pigeon.asSource()
//
//    private val holdHatchSolenoid = DoubleSolenoid(
//        Constants.kPCMId,
//        Constants.kIntakePushHatchSolenoidForwardId,
//        Constants.kIntakePushHatchSolenoidReverseId
//    )
//
//    private val rollingAverageCargoCurrent = CircularBuffer(30)
//
//    var wantedPercentOutput = 0.0
//    var wantedHoldHatchSolenoidState = HoldHatchSolenoidState.HOLD
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
//
//    var robotPositionWithIntakeOffset = Pose2d()
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
//            inverted = false
//            voltageCompensationSaturation = 12.volt
//            voltageCompensationEnabled = true
//
//            continuousCurrentLimit = 18.amp
//            currentLimitingEnabled = true
//
//            brakeMode = NeutralMode.Brake
//        }
//    }
//
//    override fun periodic() {
//        // PERIODIC
//        rollingAverageCargoCurrent.add(intakeCargoMaster.outputCurrent)
//        println("Average Current: ${rollingAverageCargoCurrent.average}")
//        isSeeingCargo = false // rollingAverageCargoCurrent.average > 11.0
//        if (isSeeingCargo) {
//            isHoldingCargo = true
//        } else if (wantedPercentOutput < 0) {
//            isHoldingCargo = false
//        }
//        isHoldingHatch = false
//
//        robotPositionWithIntakeOffset = DriveSubsystem.robotPosition + Pose2d(Length.kZero, -Constants.kBadIntakeOffset)
//
//        // DEBUG PERIODIC
//        if (Robot.shouldDebug) {
//            pigeon.getYawPitchRoll(tempYPR)
//            robotPitch = tempYPR[2].degree
//        }
//
//        intakeCargoMaster.set(ControlMode.PercentOutput, wantedPercentOutput)
//        if (wantedHoldHatchSolenoidState != holdHatchSolenoidState) {
//            if (wantedHoldHatchSolenoidState == IntakeSubsystem.HoldHatchSolenoidState.HOLD) {
//                holdHatchSolenoid.set(DoubleSolenoid.Value.kForward)
//            } else {
//                holdHatchSolenoid.set(DoubleSolenoid.Value.kReverse)
//            }
//
//            holdHatchSolenoidState = wantedHoldHatchSolenoidState
//        }
//    }
//
//    override fun zeroOutputs() {
//        wantedPercentOutput = 0.0
//    }
//
//    enum class HoldHatchSolenoidState { HOLD, PLACE }
//}


object IntakeSubsystem : FalconSubsystem() {

    var badIntakeOffset = 0.65.inch

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

    var wantedPercentOutput = 0.0
    var wantedExtensionSolenoidState = ExtensionSolenoidState.RETRACTED
    var wantedLauncherSolenoidState = false
    var wantedFlippingState = false

    // PERIODIC
    var isSeeingCargo: Boolean = false
        private set
    var isHoldingCargo: Boolean = false
        private set
    var isHoldingHatch: Boolean = false
        private set
    var extensionSolenoidState = ExtensionSolenoidState.RETRACTED
        private set
    var launcherSolenoidState: Boolean = true
        private set

    var robotPositionWithIntakeOffset = Pose2d()
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

            motor.continuousCurrentLimit = 25.amp
            motor.currentLimitingEnabled = true

            motor.brakeMode = NeutralMode.Brake
        }
    }

    override fun periodic() {
        // PERIODIC
        isSeeingCargo = leftBallSensor.averageVoltage > 1.7 || rightBallSensor.averageVoltage > 1.2
        isHoldingCargo = extensionSolenoidState == IntakeSubsystem.ExtensionSolenoidState.RETRACTED && isSeeingCargo
        isHoldingHatch = false // extensionSolenoidState == IntakeSubsystem.ExtensionSolenoidState.RETRACTED && !isFullyExtended

        robotPositionWithIntakeOffset = DriveSubsystem.robotPosition + Pose2d(Length.kZero, -badIntakeOffset)

        // DEBUG PERIODIC
        if (Robot.shouldDebug) {
            pigeon.getYawPitchRoll(tempYPR)
            robotPitch = tempYPR[2].degree
        }

        intakeMaster.set(ControlMode.PercentOutput, wantedPercentOutput)

        var wantedExtensionSolenoidState = this.wantedExtensionSolenoidState
        var wantedLauncherSolenoidState = this.wantedLauncherSolenoidState

        if(wantedFlippingState) {
            wantedExtensionSolenoidState = IntakeSubsystem.ExtensionSolenoidState.RETRACTED
            wantedLauncherSolenoidState = false
        }

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
