package org.ghrobotics.frc2019.robot.subsytems.drive

import org.ghrobotics.lib.mathematics.statespace.Matrix

object DriveSSMatrices {
    val A = Matrix(
        arrayOf(
            doubleArrayOf(9.74434110e-01, -1.14512517e-04),
            doubleArrayOf(-1.14512517e-04, 9.74434110e-01)
        )
    )
    val B = Matrix(
        arrayOf(
            doubleArrayOf(1.26397206e-02, 5.66147406e-05),
            doubleArrayOf(5.66147406e-05, 1.26397206e-02)
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
            doubleArrayOf(9.91858696e+00, -3.72155275e-03),
            doubleArrayOf(-3.72155275e-03, 9.91858696e+00)
        )
    )

    val Kff = Matrix(
        arrayOf(
            doubleArrayOf(79.11726018, -0.35437517),
            doubleArrayOf(-0.35437517, 79.11726018)
        )
    )

    val L = Matrix(
        arrayOf(
            doubleArrayOf(9.74336686e-01, -1.14501070e-04),
            doubleArrayOf(-1.14501070e-04, 9.74336686e-01)
        )
    )

    val initialState = Matrix(
        arrayOf(
            doubleArrayOf(0.0),
            doubleArrayOf(0.0)
        )
    )
}