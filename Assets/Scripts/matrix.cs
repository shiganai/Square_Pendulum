using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Drawing;
using System;
using System.Threading.Tasks;

public class matrix : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        float[][] test = MatrixCreate(2, 2);

        float target_Deg = 180;
        test[0][0] = Mathf.Cos(target_Deg * Mathf.Deg2Rad);
        test[1][0] = Mathf.Sin(target_Deg * Mathf.Deg2Rad);
        test[0][1] = -Mathf.Sin(target_Deg * Mathf.Deg2Rad);
        test[1][1] = Mathf.Cos(target_Deg * Mathf.Deg2Rad);

        float[][] test_Inverted = MatrixInverse(test);

        Debug.Log(MatrixAsString(test));
        Debug.Log(MatrixAsString(test_Inverted));
    }

    // Update is called once per frame
    void Update()
    {

    }
    public static float[][] MatrixCreate(int rows, int cols)
    {
        // creates a matrix initialized to all 0.0s
        // do error checking here?
        float[][] result = new float[rows][];
        for (int i = 0; i < rows; ++i)
            result[i] = new float[cols]; // auto init to 0.0
        return result;
    }
    public static string MatrixAsString(float[][] matrix)
    {
        string s = "";
        for (int i = 0; i < matrix.Length; ++i)
        {
            for (int j = 0; j < matrix[i].Length; ++j)
                s += matrix[i][j].ToString("F3").PadLeft(8) + " ";
            s += Environment.NewLine;
        }
        return s;
    }
    public static float[][] MatrixProduct(float[][] matrixA, float[][] matrixB)
    {
        // error check, compute aRows, aCols, bCols
        int aRows = matrixA.Length; int aCols = matrixA[0].Length;
        int bRows = matrixB.Length; int bCols = matrixB[0].Length;
        float[][] result = MatrixCreate(aRows, bCols);
        Parallel.For(0, aRows, i =>
        {
            for (int j = 0; j < bCols; ++j)
                for (int k = 0; k < aCols; ++k)
                    result[i][j] += matrixA[i][k] * matrixB[k][j];
        }
        );
        return result;
    }
    public static float[] HelperSolve(float[][] luMatrix, float[] b)
    {
        // solve luMatrix * x = b
        int n = luMatrix.Length;
        float[] x = new float[n];
        b.CopyTo(x, 0);
        for (int i = 1; i < n; ++i)
        {
            float sum = x[i];
            for (int j = 0; j < i; ++j)
                sum -= luMatrix[i][j] * x[j];
            x[i] = sum;
        }
        x[n - 1] /= luMatrix[n - 1][n - 1];
        for (int i = n - 2; i >= 0; --i)
        {
            float sum = x[i];
            for (int j = i + 1; j < n; ++j)
                sum -= luMatrix[i][j] * x[j];
            x[i] = sum / luMatrix[i][i];
        }
        return x;
    }
    public static float[][] MatrixInverse(float[][] matrix)
    {
        int n = matrix.Length;
        float[][] result = MatrixDuplicate(matrix);
        int[] perm;
        int toggle;
        float[][] lum = MatrixDecompose(matrix, out perm, out toggle);
        if (lum == null)
            throw new Exception("Unable to compute inverse");
        float[] b = new float[n];
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                if (i == perm[j])
                    b[j] = 1.0f;
                else
                    b[j] = 0.0f;
            }
            float[] x = HelperSolve(lum, b);
            for (int j = 0; j < n; ++j)
                result[j][i] = x[j];
        }
        return result;
    }
    public static float[][] MatrixDecompose(float[][] matrix, out int[] perm, out int toggle)
    {
        // Doolittle LUP decomposition.
        // assumes matrix is square.
        int n = matrix.Length; // convenience
        float[][] result = MatrixDuplicate(matrix);
        perm = new int[n];
        for (int i = 0; i < n; ++i) { perm[i] = i; }
        toggle = 1;
        for (int j = 0; j < n - 1; ++j) // each column
        {
            float colMax = Math.Abs(result[j][j]); // largest val in col j
            int pRow = j;
            for (int i = j + 1; i < n; ++i)
            {
                if (result[i][j] > colMax)
                {
                    colMax = result[i][j];
                    pRow = i;
                }
            }
            if (pRow != j) // swap rows
            {
                float[] rowPtr = result[pRow];
                result[pRow] = result[j];
                result[j] = rowPtr;
                int tmp = perm[pRow]; // and swap perm info
                perm[pRow] = perm[j];
                perm[j] = tmp;
                toggle = -toggle; // row-swap toggle
            }
            if (Math.Abs(result[j][j]) < 1.0E-20)
                return null; // consider a throw
            for (int i = j + 1; i < n; ++i)
            {
                result[i][j] /= result[j][j];
                for (int k = j + 1; k < n; ++k)
                    result[i][k] -= result[i][j] * result[j][k];
            }
        } // main j column loop
        return result;
    }
    public static float[][] MatrixDuplicate(float[][] matrix)
    {
        // assumes matrix is not null.
        float[][] result = MatrixCreate(matrix.Length, matrix[0].Length);
        for (int i = 0; i < matrix.Length; ++i) // copy the values
            for (int j = 0; j < matrix[i].Length; ++j)
                result[i][j] = matrix[i][j];
        return result;
    }

    public static float[][] MatrixSum(float[][] a, float[][] b)
    {
        int row_Num = a.Length;
        int col_Num = a[0].Length;

        float[][] result = matrix.MatrixCreate(row_Num, col_Num);

        for (int i = 0; i < row_Num; i++)
        {
            for (int j = 0; j < col_Num; j++)
            {
                result[i][j] = a[i][j] + b[i][j];
            }

        }
        return result;
    }
    public static float[][] MatrixDiff(float[][] a, float[][] b)
    {
        int row_Num = a.Length;
        int col_Num = a[0].Length;

        float[][] result = matrix.MatrixCreate(row_Num, col_Num);

        for (int i = 0; i < row_Num; i++)
        {
            for (int j = 0; j < col_Num; j++)
            {
                result[i][j] = a[i][j] - b[i][j];
            }

        }
        return result;
    }
    public static float[][] MatrixProduct(float[][] a, float b)
    {
        int row_Num = a.Length;
        int col_Num = a[0].Length;

        float[][] result = matrix.MatrixCreate(row_Num, col_Num);

        for (int i = 0; i < row_Num; i++)
        {
            for (int j = 0; j < col_Num; j++)
            {
                result[i][j] = a[i][j] * b;
            }

        }
        return result;
    }
}
