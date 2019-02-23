/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.subsystems.arm.ArmNativeUnitModel
import org.ghrobotics.frc2019.subsystems.elevator.SpringCascadeNativeUnitModel
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.nativeunits.*
import kotlin.math.pow


// Crossfire
//@Suppress("MemberVisibilityCanBePrivate")
//object Constants {
//
//    // PHYSICS
//    const val kAccelerationDueToGravity = 9.80665
//
//    // GLOBAL CTRE TIMEOUT
//    const val kCTRETimeout = 10
//
//    // MOTOR IDS
//    const val kDriveLeftMasterId = 1
//    const val kDriveLeftSlaveId = 2
//    const val kDriveRightMasterId = 3
//    const val kDriveRightSlaveId = 4
//    const val kElevatorMasterId = 5
//    const val kElevatorSlave1Id = 6
//    const val kElevatorSlave2Id = 7
//    const val kElevatorSlave3Id = 8
//    const val kArmId = 9
//    const val kIntakeLeftId = 10
//    const val kIntakeRightId = 11
//    const val kClimbFrontWinchMasterId = 12
//    const val kClimbFrontWinchSlaveId = 13
//    const val kClimbBackWinchMasterId = 14
//    const val kClimbBackWinchSlaveId = 15
//    const val kClimbWheelId = 16
//
//
//    // ANALOG INPUT
//    const val kLeftBallSensorId = 0
//    const val kRightBallSensorId = 1
//
//    // DIGITAL INPUT
//    const val kIntakeExtensionLimitSwitch = 0
//
//
//    // GYROS
//    const val kPigeonIMUId = kIntakeLeftId
//
//
//    // PNEUMATICS
//    const val kPCMId = 41
//    const val kIntakeExtensionSolenoidForwardId = 4
//    const val kIntakeExtensionSolenoidReverseId = 5
//    const val kIntakeLauncherSolenoidId = 3
//    const val kDriveSolenoidId = 0
//    const val kRampsSolenoidId = 1
//    const val kClimberWheelSolenoidId = 2
//
//
//    // ROBOT AND MECHANISM DIMENSIONS
//    val kRobotMass = 140.lb
//    const val kRobotMomentOfInertia = 10.0 // kg m^2
//    const val kRobotAngularDrag = 12.0 // Nm per rad/s
//
//    val kRobotWidth = 29.inch
//    val kRobotLength = 30.inch
//
//    val kBumperThickness = 4.5.inch
//    val kIntakeProtrusion = 5.inch       // Out of frame protrusion.
//    val kElevatorCrossbarHeightFromGround = 46.inch
//    val kIntakeCradleHeight = 6.inch
//    val kArmLength = 24.5.inch
//    val kBadIntakeOffset = 0.inch
//
//
//    // TRANSFORMATIONS
//    val kFrontBumperToCenter = Pose2d(-(kRobotLength / 2.0) - kBumperThickness, 0.meter, 0.degree)
//    val kBackBumperToCenter = Pose2d((kRobotLength / 2.0) + kBumperThickness, 0.meter, 0.degree)
//    val kForwardIntakeToCenter = Pose2d(-(kRobotLength / 2.0) - kIntakeProtrusion, kBadIntakeOffset, 0.degree)
//    val kBackwardIntakeToCenter = Pose2d(kRobotLength / 2.0 + kIntakeProtrusion, -kBadIntakeOffset, 0.degree)
//
//    val kCenterToFrontCamera = Pose2d(13.5.inch, (-11).inch, 0.degree)
//    val kCenterToBackCamera = Pose2d((-10.5).inch, (-9.5).inch, 180.degree)
//
//
//    // VISION
//    val kMaxTargetTrackingDistance = 16.inch
//    val kMaxTargetTrackingLifetime = 1.second
//
//
//    // DRIVE
//    val kDriveNativeUnitModel = SlopeNativeUnitModel(160.inch, 12094.nativeUnits)
//    val kDriveSensorUnitsPerRotation = 1440.nativeUnits
//    val kDriveWheelRadius = kDriveNativeUnitModel.wheelRadius(kDriveSensorUnitsPerRotation)
//    val kDriveTrackWidth = 27.75.inch
//
//    val kDriveSensorPhase = true
//
//    val kDriveCurrentLimit = 38.amp
//
//    const val kDriveKp = 1.5 // Talon SRX Units
//    const val kDriveKd = 5.0
//
//    const val kDriveLeftKv = 0.1489
//    const val kDriveLeftKa = 0.0716 // 0.0816
//    const val kDriveLeftKs = 1.2423
//
//    const val kDriveRightKv = 0.1475
//    const val kDriveRightKa = 0.1003 //
//    const val kDriveRightKs = 1.2468
//
//    const val kDriveBeta = 2.0 // Inverse meters squared
//    const val kDriveZeta = 0.7 // Unitless dampening co-efficient
//
//    private val kDriveLeftDCTransmission = DCMotorTransmission(
//        1 / kDriveLeftKv,
//        kDriveWheelRadius.value.pow(2) * kRobotMass.value / (2.0 * kDriveLeftKa),
//        kDriveLeftKs
//    )
//
//    private val kDriveRightDCTransmission = DCMotorTransmission(
//        1 / kDriveRightKv,
//        kDriveWheelRadius.value.pow(2) * kRobotMass.value / (2.0 * kDriveRightKa),
//        kDriveRightKs
//    )
//
//    val kDriveModel = DifferentialDrive(
//        kRobotMass.value,
//        kRobotMomentOfInertia,
//        kRobotAngularDrag,
//        kDriveWheelRadius.value,
//        kDriveTrackWidth.value / 2.0,
//        kDriveLeftDCTransmission,
//        kDriveRightDCTransmission
//    )
//
//
//    // ELEVATOR
//    // from elevator zero (measured from bottom of second stage)
//    val kElevatorSwitchHeight = 9.inch
//    val kElevatorSwitchNativeUnit = 2490.nativeUnits
//    val kElevatorAfterSwitchHeightSample = 54.5.inch
//    val kElevatorAfterSwitchNativeUnitSample = 9606.nativeUnits
//
//    val kElevatorNativeUnitModel = SpringCascadeNativeUnitModel(
//        switchHeight = kElevatorSwitchHeight,
//        switchNativeUnit = kElevatorSwitchNativeUnit,
//        afterSwitchHeightSample = kElevatorAfterSwitchHeightSample,
//        afterSwitchNativeUnitSample = kElevatorAfterSwitchNativeUnitSample
//    )
//
//    val kMaxElevatorHeightFromZero = 67.inch
//    val kElevatorSecondStageToArmShaft = 8.inch
//    val kElevatorHeightFromGround = 6.inch
//
//    val kElevatorSafeFlipHeight = 3.inch
//
//    val kElevatorCurrentLimit = 15.amp
//
//    val kElevatorClosedLoopVelocityTolerance = 1.inch.velocity
//    val kElevatorClosedLoopTolerance = 1.inch
//    val kElevatorAcceleration = 122.5.inch.acceleration
//    val kElevatorCruiseVelocity = 70.inch.velocity
//
//    const val kElevatorHoldVoltage = 1.07
//
//    const val kElevatorKp = 1.0
//    const val kElevatorKd = 0.0
//    val kElevatorKf = kElevatorNativeUnitModel.calculatekF(8.66 - kElevatorHoldVoltage, 64.inch.velocity.value)
//    const val kElevatorBelowSwitchKg = 0.0 / 12
//    const val kElevatorAfterSwitchKg = kElevatorHoldVoltage / 12.0
//
//
//    // ARM
//    val kArmSensorUnitsPerRotation = 1024.nativeUnits
//    val kArmUpTicks = (-514).nativeUnits
//
//    val kArmNativeUnitModel = ArmNativeUnitModel(
//        kArmUpTicks,
//        90.degree,
//        kArmSensorUnitsPerRotation,
//        true
//    )
//
//    val kArmFlipTolerance = 40.degree
//
//    val kArmCurrentLimit = 15.amp
//
//    val kArmClosedLoopVelocityTolerance = 2.degree.velocity
//    val kArmClosedLoopTolerance = 5.degree
//    val kArmCruiseVelocity = 260.156 * 1.0.degree.velocity
//    val kArmAcceleration = 218.0 * 1.0.degree.acceleration
//
//    const val kArmEmptyHoldVoltage = 1.0
//
//    val kArmEmptyKg = kArmEmptyHoldVoltage / kAccelerationDueToGravity / 12.0
//    const val kArmHatchKg = 1.5 / kAccelerationDueToGravity / 12.0
//
//    const val kArmKp = 3.5
//    const val kArmKd = 140.0
//    val kArmKf = kArmNativeUnitModel.calculatekF(11.718 - kArmEmptyHoldVoltage, Math.toRadians(260.156))
//
//    // CLIMB
//    val kClimbWinchRadius = 1.25.inch / 2.0
//    val kClimbWinchNativeUnitsPerRotation = 4096.nativeUnits
//
//    val kClimbWinchNativeUnitModel = NativeUnitLengthModel(
//        kClimbWinchNativeUnitsPerRotation,
//        kClimbWinchRadius
//    )
//
//    val kClimbWinchCurrentLimit = 20.amp
//
//    val kClimbWinchClosedLoopTolerance = 2.inch
//    val kClimbWinchClosedLoopVelocityTolerance = 1.inch.velocity
//    val kClimbWinchCruiseVelocity = 1.5.feet.velocity
//    val kClimbWinchAcceleration = 1.5.feet.acceleration
//
//    const val kClimbEncoderPIDSlot = 0
//    const val kClimbLevelingPIDSlot = 1
//
//    val kClimbDistanceBetweenLegs = 21.inch
//    val kClimbAngle = 5.degree
//
//    val kClimbLegHeightOffset = kClimbDistanceBetweenLegs / 2.0 * Math.tan(kClimbAngle.radian)
//    const val kClimbWinchKp = 0.3
//    const val kClimbWinchLevelingKp = 0.0
//
//    private fun NativeUnitModel<*>.calculatekF(voltage: Double, velocity: Double) =
//        (voltage / 12.0 * 1023.0) / (toNativeUnitVelocity(velocity) / 10.0)
//
//    @JvmStatic
//    fun main(args: Array<String>) {
//        println(kDriveWheelRadius.inch)
//    }
//}

// Curiosity

@Suppress("MemberVisibilityCanBePrivate")
object Constants {

    // PHYSICS
    const val kAccelerationDueToGravity = 9.80665

    // GLOBAL CTRE TIMEOUT
    const val kCTRETimeout = 10

    // MOTOR IDS
    const val kDriveLeftMasterId = 1
    const val kDriveLeftSlaveId = 2
    const val kDriveRightMasterId = 4
    const val kDriveRightSlaveId = 3
    const val kElevatorMasterId = 5
    const val kElevatorSlave1Id = 6
    const val kElevatorSlave2Id = 7
    const val kElevatorSlave3Id = 8
    const val kArmId = 9
    const val kIntakeLeftId = 10
    const val kIntakeRightId = 11
    const val kClimbFrontWinchMasterId = 12
    const val kClimbFrontWinchSlaveId = 13
    const val kClimbBackWinchMasterId = 14
    const val kClimbBackWinchSlaveId = 15
    const val kClimbWheelId = 16


    // ANALOG INPUT
    const val kLeftBallSensorId = 0
    const val kRightBallSensorId = 1


    // DIGITAL INPUT
    const val kIntakeExtensionLimitSwitch = 0


    // GYROS
    const val kPigeonIMUId = kIntakeLeftId


    // PNEUMATICS
    const val kPCMId = 41
    const val kIntakeExtensionSolenoidForwardId = 4
    const val kIntakeExtensionSolenoidReverseId = 5
    const val kIntakeLauncherSolenoidId = 3
    const val kDriveSolenoidId = 0
    const val kRampsSolenoidId = 1
    const val kClimberWheelSolenoidId = 2


    // ROBOT AND MECHANISM DIMENSIONS
    val kRobotMass = 140.lb
    const val kRobotMomentOfInertia = 10.0 // kg m^2
    const val kRobotAngularDrag = 12.0 // Nm per rad/s

    val kRobotWidth = 29.inch
    val kRobotLength = 30.inch

    val kBumperThickness = 4.5.inch
    val kIntakeProtrusion = 5.inch       // Out of frame protrusion.
    val kElevatorCrossbarHeightFromGround = 46.inch
    val kIntakeCradleHeight = 6.inch
    val kArmLength = 24.5.inch
    val kBadIntakeOffset = 0.inch


    // TRANSFORMATIONS
    val kFrontBumperToCenter = Pose2d(-(kRobotLength / 2.0) - kBumperThickness, 0.meter, 0.degree)
    val kBackBumperToCenter = Pose2d((kRobotLength / 2.0) + kBumperThickness, 0.meter, 0.degree)
    val kForwardIntakeToCenter = Pose2d(-(kRobotLength / 2.0) - kIntakeProtrusion, kBadIntakeOffset, 0.degree)
    val kBackwardIntakeToCenter = Pose2d(kRobotLength / 2.0 + kIntakeProtrusion, -kBadIntakeOffset, 0.degree)

    val kCenterToFrontCamera = Pose2d(13.5.inch, (-11).inch, 0.degree)
    val kCenterToBackCamera = Pose2d((-10.5).inch, (-9.5).inch, 180.degree)


    // VISION
    val kMaxTargetTrackingDistance = 16.inch
    val kMaxTargetTrackingLifetime = 1.second


    // DRIVE
    val kDriveNativeUnitModel = SlopeNativeUnitModel(
        170.5.inch,
        12918.nativeUnits
    )

    val kDriveSensorUnitsPerRotation = 1440.nativeUnits
    val kDriveWheelRadius = kDriveNativeUnitModel.wheelRadius(kDriveSensorUnitsPerRotation)
    val kDriveTrackWidth = 27.75.inch

    val kDriveSensorPhase = false

    val kDriveCurrentLimit = 38.amp

    val kDriveKp = 1.5 // Talon SRX Units
    val kDriveKd = 5.0

    const val kDriveLeftKv = 0.1489
    const val kDriveLeftKa = 0.0716 // 0.0816
    const val kDriveLeftKs = 1.2423

    const val kDriveRightKv = 0.1475
    const val kDriveRightKa = 0.1003 //
    const val kDriveRightKs = 1.2468

    const val kDriveBeta = 2.0 // Inverse meters squared
    const val kDriveZeta = 0.7 // Unitless dampening co-efficient

    private val kDriveLeftDCTransmission = DCMotorTransmission(
        1 / kDriveLeftKv,
        kDriveWheelRadius.value.pow(2) * kRobotMass.value / (2.0 * kDriveLeftKa),
        kDriveLeftKs
    )

    private val kDriveRightDCTransmission = DCMotorTransmission(
        1 / kDriveRightKv,
        kDriveWheelRadius.value.pow(2) * kRobotMass.value / (2.0 * kDriveRightKa),
        kDriveRightKs
    )

    val kDriveModel = DifferentialDrive(
        kRobotMass.value,
        kRobotMomentOfInertia,
        kRobotAngularDrag,
        kDriveWheelRadius.value,
        kDriveTrackWidth.value / 2.0,
        kDriveLeftDCTransmission,
        kDriveRightDCTransmission
    )


    // ELEVATOR
    // from elevator zero (measured from bottom of second stage)
    val kElevatorSwitchHeight = 8.inch
    val kElevatorSwitchNativeUnit = 2310.nativeUnits
    val kElevatorAfterSwitchHeightSample = 58.inch
    val kElevatorAfterSwitchNativeUnitSample = 10845.nativeUnits

    val kElevatorNativeUnitModel = SpringCascadeNativeUnitModel(
        switchHeight = kElevatorSwitchHeight,
        switchNativeUnit = kElevatorSwitchNativeUnit,
        afterSwitchHeightSample = kElevatorAfterSwitchHeightSample,
        afterSwitchNativeUnitSample = kElevatorAfterSwitchNativeUnitSample
    )

    val kMaxElevatorHeightFromZero = 67.inch
    val kElevatorSecondStageToArmShaft = 8.inch
    val kElevatorHeightFromGround = 6.inch

    val kElevatorSafeFlipHeight = 3.inch

    val kElevatorCurrentLimit = 15.amp

    val kElevatorClosedLoopVelocityTolerance = 1.inch.velocity
    val kElevatorClosedLoopTolerance = 1.inch
    val kElevatorAcceleration = 122.5.inch.acceleration
    val kElevatorCruiseVelocity = 70.inch.velocity

    const val kElevatorHoldVoltage = 1.07

    const val kElevatorKp = 1.0
    const val kElevatorKd = 0.0
    val kElevatorKf = kElevatorNativeUnitModel.calculatekF(11.1 - kElevatorHoldVoltage, 65.inch.velocity.value)
    const val kElevatorBelowSwitchKg = 0.0 / 12
    const val kElevatorAfterSwitchKg = kElevatorHoldVoltage / 12.0


    // ARM
    val kArmSensorUnitsPerRotation = 1024.nativeUnits
    val kArmUpTicks = (-562).nativeUnits

    val kArmNativeUnitModel = ArmNativeUnitModel(
        kArmUpTicks,
        90.degree,
        kArmSensorUnitsPerRotation,
        true
    )

    val kArmFlipTolerance = 40.degree

    val kArmCurrentLimit = 15.amp

    val kArmClosedLoopVelocityTolerance = 2.degree.velocity
    val kArmClosedLoopTolerance = 5.degree
    val kArmCruiseVelocity = 380.0 * 1.0.degree.velocity
    val kArmAcceleration = 372.0 * 1.0.degree.acceleration

    const val kArmEmptyHoldVoltage = 1.9

    val kArmEmptyKg = kArmEmptyHoldVoltage / kAccelerationDueToGravity / 12.0
    const val kArmHatchKg = 3.0 / kAccelerationDueToGravity / 12.0

    const val kArmKp = 7.0
    const val kArmKd = 140.0
    const val kArmKv = 0.0 / 12.0
    val kArmKf = kArmNativeUnitModel.calculatekF(11.366 - kArmEmptyHoldVoltage, Math.toRadians(260.1562))

    // CLIMB
    val kClimbWinchNativeUnitModel = SlopeNativeUnitModel(
        17.inch,
        15958.nativeUnits
    )

    val kClimbWinchCurrentLimit = 20.amp

    val kClimbWinchClosedLoopTolerance = 2.inch
    val kClimbWinchClosedLoopVelocityTolerance = 1.inch.velocity
    val kClimbWinchCruiseVelocity = 1.5.feet.velocity
    val kClimbWinchAcceleration = 1.5.feet.acceleration

    const val kClimbEncoderPIDSlot = 0
    const val kClimbLevelingPIDSlot = 1

    const val kClimbWinchLidarKp = 0.0
    const val kClimbWinchPitchKp = 0.3

    private fun NativeUnitModel<*>.calculatekF(voltage: Double, velocity: Double) =
        (voltage / 12.0 * 1023.0) / (toNativeUnitVelocity(velocity) / 10.0)

    @JvmStatic
    fun main(args: Array<String>) {
        println(kDriveWheelRadius.inch)
    }
}
