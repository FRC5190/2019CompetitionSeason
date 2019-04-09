package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.Source

object AutoClimbRoutines {

    var resettingFront = false

    private val group
        get() = sequential {
            +parallel {
                +ClimbWheelCommand(Source(0.75))
                +ClosedLoopArmCommand(160.degree)
            }.withExit { ClimbSubsystem.lidarRawAveraged < 500 }

            // Continue climb only if lidar is not faulty
            +ConditionalCommand(
                { ClimbSubsystem.lidarRawAveraged > 25 },
                sequential {
                    +ClimbWheelCommand(Source(0.5)).withTimeout(200.millisecond)

                    val resetBack = ResetWinchCommand(resetFront = false)

                    +parallel {
                        +InstantRunnableCommand { resettingFront = true }
                        +ClimbWheelCommand(Source(0.05))
                        +resetBack.withTimeout(3.second)
                    }.withExit { resetBack.wrappedValue.isCompleted }
                    +InstantRunnableCommand { resettingFront = false }

                    +ClimbWheelCommand(Source(1.0)).withExit { ClimbSubsystem.frontOnPlatform }.withTimeout(2.5.second)
                    +parallel {
                        +InstantRunnableCommand { resettingFront = true }
                        +ResetWinchCommand(resetFront = true)
                        +ClosedLoopArmCommand(105.degree).withTimeout(1.5.second)
                    }
                    +InstantRunnableCommand { resettingFront = false }
                    +DelayCommand(1.second)
                }
            )
        }

    val autoL3Climb
        get() = sequential {
            +InstantRunnableCommand { DriveSubsystem.lowGear = true }
            +ClosedLoopClimbCommand(
                22.7.inch,
                19.7.inch
            )
            val climbGroup = group
            +parallel {
                +object : FalconCommand(DriveSubsystem) {
                    init {
                        finishCondition += climbGroup.wrappedValue::isCompleted
                    }

                    override suspend fun execute() {
                        if (resettingFront) {
                            DriveSubsystem.tankDrive(-.05, -.05)
                        } else if (!ClimbSubsystem.isFrontReverseLimitSwitchClosed) {
                            DriveSubsystem.tankDrive(-.4, -.4)
                        } else {
                            DriveSubsystem.tankDrive(-.5, -.5)
                        }
                    }
                }
                +climbGroup
            }
        }

    val autoL2Climb
        get() = sequential {
            +InstantRunnableCommand { DriveSubsystem.lowGear = true }
            +ClosedLoopClimbCommand(
                8.inch,
                9.inch
            )
            val climbGroup = group
            +parallel {
                +object : FalconCommand(DriveSubsystem) {
                    init {
                        finishCondition += climbGroup.wrappedValue::isCompleted
                    }

                    override suspend fun execute() {
                        DriveSubsystem.tankDrive(-.4, -.4)
                    }
                }
                +sequential {
                    +parallel {
                        +ClimbWheelCommand(Source(0.75))
                        +ClosedLoopArmCommand(160.degree)
                    }.withExit { ClimbSubsystem.lidarRawAveraged < 500 }

                    // Continue climb only if lidar is not faulty
                    +ConditionalCommand(
                        { ClimbSubsystem.lidarRawAveraged > 25 },
                        sequential {
                            +ClimbWheelCommand(Source(0.5)).withTimeout(200.millisecond)

                            val resetBack = ResetWinchCommand(resetFront = false)

                            +parallel {
                                +InstantRunnableCommand { resettingFront = true }
                                +ClimbWheelCommand(Source(0.05))
                                +resetBack.withTimeout(3.second)
                            }.withExit { resetBack.wrappedValue.isCompleted }
                            +InstantRunnableCommand { resettingFront = false }

                            +ClimbWheelCommand(Source(1.0)).withTimeout(2.second)
                            +parallel {
                                +InstantRunnableCommand { resettingFront = true }
                                +ResetWinchCommand(resetFront = true)
                                +ClosedLoopArmCommand(105.degree).withTimeout(1.5.second)
                            }
                            +InstantRunnableCommand { resettingFront = false }
                            +DelayCommand(1.second)
                        }
                    )
                }
            }
        }
}

