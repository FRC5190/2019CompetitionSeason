package org.ghrobotics.frc2019.robot.subsytems.drive

import org.ghrobotics.lib.mathematics.statespace.Matrix

object DriveSSMatrices {
    val A = Matrix(
        arrayOf(
            doubleArrayOf(9.79906083e-01, -9.02562503e-05),
            doubleArrayOf(-9.02562503e-05, 9.79906083e-01)
        )
    )
    val B = Matrix(
        arrayOf(
            doubleArrayOf(1.10775524e-02, 4.97572654e-05),
            doubleArrayOf(4.97572654e-05, 1.10775524e-02)
        )
    )
    val C = Matrix(
        arrayOf(
            doubleArrayOf(1.0, 0.0),
            doubleArrayOf(0.0, 1.0)
        )
    )
    val D = Matrix(2, 2)

    val K = Matrix(
        arrayOf(
            doubleArrayOf(1.01897527e+01, -3.32658976e-03),
            doubleArrayOf(-3.32658976e-03, 1.01897527e+01)
        )
    )

    val Kff = Matrix(
        arrayOf(
            doubleArrayOf(90.27447075, -0.40548766),
            doubleArrayOf(-0.40548766, 90.27447075)
        )
    )

    val L = Matrix(
        arrayOf(
            doubleArrayOf(9.79881587e-01, -9.02539941e-05),
            doubleArrayOf(-9.02539941e-05, 9.79881587e-01)
        )
    )

    val initialState = Matrix(
        arrayOf(
            doubleArrayOf(0.0),
            doubleArrayOf(0.0)
        )
    )
}