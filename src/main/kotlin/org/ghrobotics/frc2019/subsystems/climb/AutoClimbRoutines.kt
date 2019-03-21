package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.Source

object AutoClimbRoutines {

    private val group get() = sequential {
        +parallel {
            +ClimbWheelCommand(Source(0.5))
            +ClosedLoopArmCommand(160.degree)
        }.withExit { ClimbSubsystem.lidarRawAveraged < 500 }

        +ClimbWheelCommand(Source(0.5)).withTimeout(200.millisecond)

        val resetBack = ResetWinchCommand(resetFront = false)

        +parallel {
            +ClosedLoopArmCommand(105.degree)
            +ClimbWheelCommand(Source(0.05))
            +resetBack
        }.withExit { resetBack.wrappedValue.isCompleted }

        +ClimbWheelCommand(Source(1.0)).withExit { ClimbSubsystem.frontOnPlatform }
        +ResetWinchCommand(resetFront = true)
        +DelayCommand(1.second)
    }

    val autoL3Climb
        get() = sequential {
            +ClosedLoopClimbCommand(
                22.2.inch,
                19.7.inch
            )
            +parallel {
                +object : FalconCommand(DriveSubsystem) {
                    init {
                        finishCondition += group.wrappedValue::isCompleted
                    }

                    override suspend fun execute() {
                        if (!ClimbSubsystem.isFrontReverseLimitSwitchClosed) {
                            DriveSubsystem.tankDrive(-.2, -.2)
                        } else {
                            DriveSubsystem.tankDrive(-.5, -.5)
                        }
                    }
                }
                +group
            }
        }

    val autoL2Climb
        get() = sequential {
            +ClosedLoopClimbCommand(
                10.inch,
                11.inch
            )
            +parallel {
                +object : FalconCommand(DriveSubsystem) {
                    init {
                        finishCondition += group.wrappedValue::isCompleted
                    }

                    override suspend fun execute() {
                        DriveSubsystem.tankDrive(-.4, -.4)
                    }
                }
                +group
            }
        }
}

