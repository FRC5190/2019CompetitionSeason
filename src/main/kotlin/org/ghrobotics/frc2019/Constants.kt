/*
 * FRC Team 5190
 * Green Hope Falcons
 */

@file:Suppress("unused")

package org.ghrobotics.frc2019

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.subsystems.arm.ArmNativeUnitModel
import org.ghrobotics.frc2019.subsystems.elevator.SpringCascadeNativeUnitModel
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunits.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunits.wheelRadius
import kotlin.math.pow


// Crossfire
@Suppress("MemberVisibilityCanBePrivate")
object Constants {

    // PHYSICS
    const val kAccelerationDueToGravity = 9.80665

    // GLOBAL CTRE TIMEOUT
    const val kCTRETimeout = 10

    // MOTOR IDS
    const val kDriveLeftMasterId = 1
    const val kDriveLeftSlaveId = 2
    const val kDriveRightMasterId = 3
    const val kDriveRightSlaveId = 4
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

    const val kCanifierId = 16


    // ANALOG INPUT
    const val kLeftBallSensorId = 0
    const val kRightBallSensorId = 1
    const val kClimberSensorId = 2

    // DIGITAL INPUT
    const val kIntakeExtensionLimitSwitch = 0
    const val kClimberHallEffectSensor = 1

    // GYROS
    const val kPigeonIMUId = kIntakeLeftId

    const val kUseMXPForLEDs = true


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

    val kRobotWidth = 30.inch
    val kRobotLength = 30.inch

    val kBumperThickness = 4.5.inch
    val kIntakeProtrusion = 6.inch       // Out of frame protrusion.
    val kElevatorCrossbarHeightFromGround = 46.inch
    val kIntakeCradleHeight = 6.inch
    val kArmLength = 24.5.inch
    val kBadIntakeOffset = 0.inch


    // TRANSFORMATIONS
    val kFrontBumperToCenter = Pose2d(-(kRobotLength / 2.0) - kBumperThickness, 0.meter, 0.degree)
    val kBackBumperToCenter = Pose2d((kRobotLength / 2.0) + kBumperThickness, 0.meter, 0.degree)
    val kForwardIntakeToCenter = Pose2d(-(kRobotLength / 2.0) - kIntakeProtrusion, kBadIntakeOffset, 0.degree)
    val kCenterToForwardIntake = Pose2d((kRobotLength / 2.0) + kIntakeProtrusion, -kBadIntakeOffset, 0.degree)
    val kBackwardIntakeToCenter = Pose2d(kRobotLength / 2.0 + kIntakeProtrusion, -kBadIntakeOffset, 0.degree)

    val kCenterToFrontCamera = Pose2d((-1.75).inch, 0.0.inch, 0.degree)
    val kCenterToBackCamera = Pose2d((-6.25).inch, 0.0.inch, 180.degree)


    // VISION
    const val kVisionCameraFPS = 30.0
    val kVisionCameraPing = 0.75.second
    val kVisionCameraTimeout = 2.second
    val kTargetTrackingDistanceErrorTolerance = 16.inch
    val kTargetTrackingMinLifetime = 0.1.second
    val kTargetTrackingMaxLifetime = 0.5.second


    // DRIVE
    val kDriveNativeUnitModel = SlopeNativeUnitModel(138.inch, 10000.nativeUnits)
    val kDriveSensorUnitsPerRotation = 1440.nativeUnits
    val kDriveWheelRadius = kDriveNativeUnitModel.wheelRadius(kDriveSensorUnitsPerRotation)
    val kDriveTrackWidth = 27.75.inch

    val kDriveSensorPhase = true

    val kDriveCurrentLimit = 38.amp

    const val kDriveKp = 2.35 // Talon SRX Units
    const val kDriveKd = 2.0

    const val kDriveLeftKv = 0.1489
    const val kDriveLeftKa = 0.05
    const val kDriveLeftKs = 1.2423

    const val kDriveRightKv = 0.1475
    const val kDriveRightKa = 0.05
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
    val kElevatorSwitchHeight = 9.inch
    val kElevatorSwitchNativeUnit = 2490.nativeUnits
    val kElevatorAfterSwitchHeightSample = 54.5.inch
    val kElevatorAfterSwitchNativeUnitSample = 9606.nativeUnits

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

    val kElevatorCurrentLimit = 20.amp

    val kElevatorClosedLoopVelocityTolerance = 1.inch.velocity
    val kElevatorClosedLoopTolerance = 1.inch
    val kElevatorAcceleration = 122.5.inch.acceleration
    val kElevatorCruiseVelocity = 70.inch.velocity

    const val kElevatorHoldVoltage = 1.07

    val kElevatorBlockingCameraRange = (29.inch.value)..(41.inch.value)
    val kElevatorVisionPosition = 45.inch

    const val kElevatorKp = 1.0
    const val kElevatorKd = 0.0
    const val kElevatorBelowSwitchKg = 0.7889999999999985 / 12.0
    const val kElevatorAfterSwitchKg = 1.5089999999999901 / 12.0
    const val kElevatorBelowSwitchKs = 1.3169999999999993 / 12.0
    const val kElevatorAfterSwitchKs = 1.0709999999999904 / 12.0
    val kElevatorKf = kElevatorNativeUnitModel.calculatekF(8.66 - kElevatorHoldVoltage, 64.inch.velocity.value)

    // ARM
    val kArmSensorUnitsPerRotation = 1024.nativeUnits
    val kArmUpTicks = (-495).nativeUnits

    val kArmNativeUnitModel = ArmNativeUnitModel(
        kArmUpTicks,
        90.degree,
        kArmSensorUnitsPerRotation,
        true
    )

    val kArmFlipTolerance = 40.degree
    val kArmSafeFlipAngle = 30.degree

    val kArmCurrentLimit = 15.amp

    val kArmClosedLoopVelocityTolerance = 2.degree.velocity
    val kArmClosedLoopTolerance = 5.degree
    val kArmCruiseVelocity = 260.156 * 1.0.degree.velocity
    val kArmAcceleration = 300.0 * 1.0.degree.acceleration

    const val kArmEmptyHoldVoltage = 1.5

    val kArmEmptyKg = kArmEmptyHoldVoltage / kAccelerationDueToGravity / 12.0
    const val kArmHatchKg = 1.5 / kAccelerationDueToGravity / 12.0

    const val kArmKp = 3.5
    const val kArmKd = 700.0
    val kArmKf = kArmNativeUnitModel.calculatekF(11.718 - kArmEmptyHoldVoltage, Math.toRadians(260.156))

    // CLIMB
    val kClimbBackWinchNativeUnitModel = SlopeNativeUnitModel(
        14.inch,
        14782.nativeUnits
    )

    val kClimbFrontWinchNativeUnitModel = SlopeNativeUnitModel(
        16.inch,
        12677.nativeUnits
    )

    val kClimbWinchCurrentLimit = 40.amp

    val kClimbWinchClosedLoopTolerance = 1.inch

    const val kClimbFrontWinchLimitSwitchTicks = -1254
    const val kClimbBackWinchLimitSwitchTicks = -2220


    val kClimbLidarScale = 22.inch / 516
    const val kClimbLidarZero = 150

    const val kClimbFrontL3Ticks = 22489.0
    const val kClimbBackL3Ticks = 18892.0

    const val kClimbWinchPositionKp = 2.0
    const val kClimbWinchLevelingKp = 9.0
    const val kClimbWinchLevelingKd = 25.0

    private fun NativeUnitModel<*>.calculatekF(voltage: Double, velocity: Double) =
        (voltage / 12.0 * 1023.0) / (toNativeUnitVelocity(velocity) / 10.0)
}

// Curiosity
//
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
//    const val kDriveRightMasterId = 4
//    const val kDriveRightSlaveId = 3
//    const val kElevatorMasterId = 5
//    const val kElevatorSlave1Id = 6
//    const val kElevatorSlave2Id = 7
//    const val kElevatorSlave3Id = 8
//    const val kArmId = 9
//    //    const val kIntakeCargoId = 10
//    const val kIntakeLeftId = 10
//    const val kIntakeRightId = 11
//    const val kClimbFrontWinchMasterId = 12
//    const val kClimbFrontWinchSlaveId = 13
//    const val kClimbBackWinchMasterId = 14
//    const val kClimbBackWinchSlaveId = 15
//    const val kClimbWheelId = 16
//
//    const val kCanifierId = 16
//
//    // ANALOG INPUT
//    const val kLeftBallSensorId = 0
//    const val kRightBallSensorId = 1
//    const val kClimberSensorId = 2
//
//
//    // DIGITAL INPUT
//    const val kIntakeExtensionLimitSwitch = 0
//    const val kClimberHallEffectSensor = 1
//
//
//    // GYROS
//    const val kPigeonIMUId = kIntakeLeftId
////    const val kPigeonIMUId = kIntakeCargoId
//
//    const val kUseMXPForLEDs = false
//
//
//    // PNEUMATICS
//    const val kPCMId = 41
//    const val kIntakeExtensionSolenoidForwardId = 0
//    const val kIntakeExtensionSolenoidReverseId = 1
//    //    const val kIntakePushHatchSolenoidForwardId = 0
////    const val kIntakePushHatchSolenoidReverseId = 1
//    const val kIntakeLauncherSolenoidId = 5
//    const val kIntakeHoldHatchSolenoidId = 5
//    const val kDriveSolenoidId = 4
//
//
//    // ROBOT AND MECHANISM DIMENSIONS
//    val kRobotMass = 140.lb
//    const val kRobotMomentOfInertia = 10.0 // kg m^2
//    const val kRobotAngularDrag = 12.0 // Nm per rad/s
//
//    val kRobotWidth = 30.inch
//    val kRobotLength = 30.inch
//
//    val kBumperThickness = 4.5.inch
//    val kIntakeProtrusion = 12.inch       // Out of frame protrusion.
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
//    val kCenterToForwardIntake = Pose2d((kRobotLength / 2.0) + kIntakeProtrusion, -kBadIntakeOffset, 0.degree)
//    val kBackwardIntakeToCenter = Pose2d(kRobotLength / 2.0 + kIntakeProtrusion, -kBadIntakeOffset, 0.degree)
//
//    val kCenterToFrontCamera = Pose2d((-1.75).inch, 0.inch, 0.degree)
//    val kCenterToBackCamera = Pose2d((-6.25).inch, 0.inch, 180.degree)
//
//
//    // VISION
//    const val kVisionCameraFPS = 30.0
//    val kVisionCameraTimeout = 2.second
//    val kTargetTrackingDistanceErrorTolerance = 16.inch
//    val kTargetTrackingMaxLifetime = 0.5.second
//
//
//    // DRIVE
//    val kDriveNativeUnitModel = SlopeNativeUnitModel(
//        170.5.inch,
//        12918.nativeUnits
//    )
//
////    val kDriveNativeUnitModel = SlopeNativeUnitModel(
////        112.inch,
////        45.25.nativeUnits
////    )
//
//    val kDriveSensorUnitsPerRotation = 1440.nativeUnits
//    val kDriveWheelRadius = kDriveNativeUnitModel.wheelRadius(kDriveSensorUnitsPerRotation)
//    val kDriveTrackWidth = 27.75.inch
//
//    const val kDriveSensorPhase = false
//
//    val kDriveCurrentLimit = 38.amp
//
//    const val kDriveKp = 3.0 // Talon SRX Units
//    const val kDriveKd = 2.0
//
////    const val kDriveKp = 0.001 // Talon SRX Units
////    const val kDriveKd = 0.0
//
//    const val kDriveLeftKv = 0.1489
//    const val kDriveLeftKa = 0.05 // 0.0816
//    const val kDriveLeftKs = 1.2423
//
//    const val kDriveRightKv = 0.1475
//    const val kDriveRightKa = 0.05 //
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
//    val kElevatorSwitchHeight = 7.inch
//    val kElevatorSwitchNativeUnit = 2027.nativeUnits
//    val kElevatorAfterSwitchHeightSample = 45.inch
//    val kElevatorAfterSwitchNativeUnitSample = 7834.nativeUnits
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
//    val kElevatorCurrentLimit = 25.amp
//
//    val kElevatorClosedLoopVelocityTolerance = 1.inch.velocity
//    val kElevatorClosedLoopTolerance = 1.inch
//    val kElevatorAcceleration = 122.5.inch.acceleration
//    val kElevatorCruiseVelocity = 70.inch.velocity
//
//    val kElevatorBlockingCameraRange = (29.inch.value)..(41.inch.value)
//    val kElevatorVisionPosition = 45.inch
//
//    const val kElevatorKp = 1.0
//    const val kElevatorKd = 0.0
//    const val kElevatorBelowSwitchKg = 0.7889999999999985 / 12.0
//    const val kElevatorAfterSwitchKg = 1.5089999999999901 / 12.0
//    const val kElevatorBelowSwitchKs = 1.3169999999999993 / 12.0
//    const val kElevatorAfterSwitchKs = 1.0709999999999904 / 12.0
//    val kElevatorKf =
//        kElevatorNativeUnitModel.calculatekF(11.1 - (kElevatorAfterSwitchKg * 12.0), 65.inch.velocity.value)
//
//
//    // ARM
//    val kArmSensorUnitsPerRotation = 1024.nativeUnits
//    val kArmUpTicks = (-550).nativeUnits
//
//    val kArmNativeUnitModel = ArmNativeUnitModel(
//        kArmUpTicks,
//        90.degree,
//        kArmSensorUnitsPerRotation,
//        true
//    )
//
//    val kArmFlipTolerance = 30.degree
//    val kArmSafeFlipAngle = 30.degree
//
//    val kArmCurrentLimit = 20.amp
//
//    val kArmClosedLoopVelocityTolerance = 2.degree.velocity
//    val kArmClosedLoopTolerance = 2.5.degree
//    val kArmCruiseVelocity = 254 * 1.0.degree.velocity
//    val kArmAcceleration = 6.28 * 1.0.radian.acceleration
//
//    const val kArmEmptyHoldVoltage = 1.75
//
//    val kArmEmptyKg = kArmEmptyHoldVoltage / kAccelerationDueToGravity / 12.0
//    const val kArmHatchKg = 4.0 / kAccelerationDueToGravity / 12.0
//
//    const val kArmKp = 6.5
//    const val kArmKd = 650.0
//    const val kArmKv = 0.0 / 12.0
//    val kArmKf = kArmNativeUnitModel.calculatekF(12 - kArmEmptyHoldVoltage, Math.toRadians(254.0))
//
//    // CLIMB
//    val kClimbBackWinchNativeUnitModel = SlopeNativeUnitModel(
//        12.5.inch,
//        8900.nativeUnits
//    )
//
//    val kClimbFrontWinchNativeUnitModel = kClimbBackWinchNativeUnitModel
//
//    val kClimbWinchCurrentLimit = 20.amp
//
//    val kClimbWinchClosedLoopTolerance = 1.inch
//
//    const val kClimbFrontWinchLimitSwitchTicks = -1254
//    const val kClimbBackWinchLimitSwitchTicks = -2220
//
//
//    val kClimbLidarScale = 22.inch / 516
//    const val kClimbLidarZero = 150
//
//    const val kClimbFrontL3Ticks = 22489.0
//    const val kClimbBackL3Ticks = 18892.0
//
//    val kEncoderErrorTolerance = 2.inch
//    val kEncoderLidarResetTolerance = 2.5.inch
//
//    val kClimbDistanceBetweenLegs = 21.5.inch
//    val kClimbMaxHeight = 30.inch
//    val kClimbDisableLevelingHeight = 3.inch
//
//    const val kClimbWinchPositionKp = 0.85
//    const val kClimbWinchLevelingKp = 4.0
//    const val kClimbWinchLevelingKd = 150.0
//
//    private fun NativeUnitModel<*>.calculatekF(voltage: Double, velocity: Double) =
//        (voltage / 12.0 * 1023.0) / (toNativeUnitVelocity(velocity) / 10.0)
//}
