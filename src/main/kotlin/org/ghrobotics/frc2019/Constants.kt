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
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunits.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunits.wheelRadius
import kotlin.math.pow

@Suppress("MemberVisibilityCanBePrivate")
object Constants {

    const val isRaceRobot = false

    // PHYSICS
    const val kAccelerationDueToGravity = 9.80665

    // GLOBAL CTRE TIMEOUT
    const val kCTRETimeout = 10

    // MOTOR IDS
    const val kDriveLeftMasterId = 1
    const val kDriveLeftSlaveId = 2
    val kDriveRightMasterId = if (isRaceRobot) 3 else 4
    val kDriveRightSlaveId = if (isRaceRobot) 4 else 3
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
    const val kPigeonIMUId = 17


    // PNEUMATICS
    const val kPCMId = 41
    const val kIntakeExtensionSolenoidId = 1
    const val kIntakeLauncherSolenoidId = 5
    const val kDriveSolenoidId = 3
    const val kRampsSolenoidId = 0
    const val kClimberWheelSolenoidId = 2


    // FIELD
    val kHypotenuseDifferenceForRamp = .4.inch


    // ROBOT AND MECHANISM DIMENSIONS
    val kRobotMass = 140.lb
    const val kRobotMomentOfInertia = 10.0 // kg m^2
    const val kRobotAngularDrag = 12.0 // Nm per rad/s

    val kRobotWidth = 29.inch
    val kRobotLength = 30.inch

    val kBumperThickness = 4.5.inch
    val kIntakeProtrusion = 9.inch       // Out of frame protrusion.
    val kElevatorCrossbarHeightFromGround = 46.inch
    val kIntakeCradleHeight = 6.inch
    val kArmLength = 24.5.inch
    val kBadIntakeOffset = 0.inch


    // TRANSFORMATIONS
    val kFrontBumperToCenter = Pose2d(-(kRobotLength / 2.0) - kBumperThickness, 0.meter, 0.degree)
    val kBackBumperToCenter = Pose2d((kRobotLength / 2.0) + kBumperThickness, 0.meter, 0.degree)
    val kForwardIntakeToCenter = Pose2d(-(kRobotLength / 2.0) - kIntakeProtrusion, kBadIntakeOffset, 0.degree)
    val kBackwardIntakeToCenter = Pose2d(kRobotLength / 2.0 + kIntakeProtrusion, -kBadIntakeOffset, 0.degree)
    val kCenterToCamera = Pose2d((-13).inch, (-15).inch, 180.degree)


    // VISION
    val kMaxTargetTrackingDistance = 10.inch
    val kMaxTargetTrackingLifetime = 1.second


    // DRIVE
    val kDriveNativeUnitModel = SlopeNativeUnitModel(
        97.5.inch,
        7244.nativeUnits
    )

    val kDriveSensorUnitsPerRotation = 1440.nativeUnits
    val kDriveWheelRadius = kDriveNativeUnitModel.wheelRadius(kDriveSensorUnitsPerRotation)
    val kDriveTrackWidth = 27.75.inch

    val kDriveCurrentLimit = 38.amp

    const val kDriveKp = 0.9 // Talon SRX Units
    const val kDriveKd = 9.0

    const val kDriveLeftKv = 0.1489
    const val kDriveLeftKa = 0.0716 // 0.0816
    const val kDriveLeftKs = 1.2423

    const val kDriveRightKv = 0.1475
    const val kDriveRightKa = 0.1003 //
    const val kDriveRightKs = 1.2468

    const val kDriveBeta = 2.5 // Inverse meters squared
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

    val kElevatorCurrentLimit = 25.amp

    val kElevatorClosedLoopVelocityTolerance = 1.inch.velocity
    val kElevatorClosedLoopTolerance = 1.inch
    val kElevatorAcceleration = 175.inch.acceleration
    val kElevatorCruiseVelocity = 200.inch.velocity

    const val kElevatorKp = 2.0
    const val kElevatorKd = 0.0
    const val kElevatorKf = 0.800 / 2.0
    const val kElevatorBelowSwitchKg = 0.0 / 12
    const val kElevatorAfterSwitchKg = 0.77 / 12


    // ARM
    val kArmSensorUnitsPerRotation = 1024.nativeUnits
    val kArmUpTicks = (-710).nativeUnits

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
    val kArmCruiseVelocity = 6 * 1.0.radian.velocity
    val kArmAcceleration = 4 * 1.0.radian.acceleration

    const val kArmKp = 4.0
    const val kArmKd = 0.0

    const val kArmEmptyKg = 0.015

    const val kArmHatchKg = 0.028

    const val kArmKv = 0.0 / 12.0
    const val kArmKf = 0.0 // 7.566


    // CLIMB
    val kClimbWinchRadius = 1.25.inch / 2.0
    val kClimbWinchNativeUnitsPerRotation = 4096.nativeUnits

    val kClimbWinchNativeUnitModel = NativeUnitLengthModel(
        kClimbWinchNativeUnitsPerRotation,
        kClimbWinchRadius
    )

    val kClimbWinchCurrentLimit = 20.amp

    val kClimbWinchClosedLoopTolerance = 2.inch
    val kClimbWinchClosedLoopVelocityTolerance = 1.inch.velocity
    val kClimbWinchCruiseVelocity = 1.5.feet.velocity
    val kClimbWinchAcceleration = 1.5.feet.acceleration

    const val kClimbEncoderPIDSlot = 0
    const val kClimbLevelingPIDSlot = 1

    val kClimbDistanceBetweenLegs = if(isRaceRobot) 21.inch else 21.5.inch
    val kClimbAngle = 5.degree

    val kClimbLegHeightOffset = kClimbDistanceBetweenLegs / 2.0 * Math.tan(kClimbAngle.radian)

    const val kClimbWinchKp = 0.3
    const val kClimbWinchLevelingKp = 0.0
}
