using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Drawing;
using System;
using System.Threading.Tasks;

public class matrix
{
    public float[,] data;
    public float[][] data_Jagg;
    int rows;
    int cols;

    public matrix(int rows, int cols)
    {
        this.rows = rows;
        this.cols = cols;
        data_Jagg = new float[rows][];
        data = new float[rows, cols];
        for (int i = 0; i < rows; ++i)
        {
            data_Jagg[i] = new float[cols]; // auto init to 0.0
        }
    }

    public matrix(float[][] data_Jagg)
    {
        rows = data_Jagg.Length;
        cols = data_Jagg[0].Length;

        //this.data_Jagg = data_Jagg;
        this.data_Jagg = new float[rows][];

        data = new float[rows, cols];

        for (int i = 0; i < rows; ++i)
        {
            this.data_Jagg[i] = new float[cols]; // auto init to 0.0
            for (int j = 0; j < cols; j++)
            {
                this.data_Jagg[i][j] = data_Jagg[i][j];
                this.data[i, j] = data_Jagg[i][j];
            }
        }
    }

    public matrix(float[,] data)
    {
        rows = data.GetLength(0);
        cols = data.GetLength(1);

        //this.data_Jagg = data_Jagg;
        this.data_Jagg = new float[rows][];

        this.data = new float[rows, cols];

        for (int i = 0; i < rows; ++i)
        {
            this.data_Jagg[i] = new float[cols]; // auto init to 0.0
            for (int j = 0; j < cols; j++)
            {
                this.data_Jagg[i][j] = data[i,j];
                this.data[i, j] = data[i,j];
            }
        }
    }

    public float this[int rows, int cols]
    {
        set
        {
            data_Jagg[rows][cols] = value;
            data[rows, cols] = value;
        }
        get
        {
            return data[rows, cols];
        }
    }

    public static string asString(matrix target_matrix)
    {
        float[][] matrix = target_matrix.data_Jagg;
        string s = "";
        for (int i = 0; i < matrix.Length; ++i)
        {
            for (int j = 0; j < matrix[i].Length; ++j)
                s += matrix[i][j].ToString("F3").PadLeft(8) + " ";
            s += Environment.NewLine;
        }
        return s;
    }
    //public static float[][] matrixProduct(float[][] matrixA, float[][] matrixB)
    //{
    //    // error check, compute aRows, aCols, bCols
    //    int aRows = matrixA.Length; int aCols = matrixA[0].Length;
    //    int bRows = matrixB.Length; int bCols = matrixB[0].Length;
    //    matrix result = new matrix(aRows, bCols);
    //    Parallel.For(0, aRows, i =>
    //    {
    //        for (int j = 0; j < bCols; ++j)
    //            for (int k = 0; k < aCols; ++k)
    //                result[i,j] += matrixA[i][k] * matrixB[k][j];
    //    }
    //    );
    //    return result.data;
    //}
    public static float[] HelperSolve(float[][] lumatrix, float[] b)
    {
        // solve lumatrix * x = b
        int n = lumatrix.Length;
        float[] x = new float[n];
        b.CopyTo(x, 0);
        for (int i = 1; i < n; ++i)
        {
            float sum = x[i];
            for (int j = 0; j < i; ++j)
                sum -= lumatrix[i][j] * x[j];
            x[i] = sum;
        }
        x[n - 1] /= lumatrix[n - 1][n - 1];
        for (int i = n - 2; i >= 0; --i)
        {
            float sum = x[i];
            for (int j = i + 1; j < n; ++j)
                sum -= lumatrix[i][j] * x[j];
            x[i] = sum / lumatrix[i][i];
        }
        return x;
    }
    public static matrix Inverse(matrix target_matrix)
    {
        float[][] matrix = target_matrix.data_Jagg;
        int n = matrix.Length;
        matrix result = duplicate(matrix);
        int[] perm;
        int toggle;
        float[][] lum = decompose(matrix, out perm, out toggle);
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
                result[j,i] = x[j];
        }

        return result;
    }
    public static float[][] decompose(float[][] matrix, out int[] perm, out int toggle)
    {
        // Doolittle LUP decomposition.
        // assumes matrix is square.
        int n = matrix.Length; // convenience
        float[][] result = duplicate(matrix).data_Jagg;
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
    public static matrix duplicate(float[][] matrix)
    {

        // assumes matrix is not null.
        matrix result = new matrix(matrix.Length, matrix[0].Length);
        for (int i = 0; i < matrix.Length; ++i) // copy the values
            for (int j = 0; j < matrix[i].Length; ++j)
                result[i,j] = matrix[i][j];
        return result;
    }

    public static matrix operator+ (matrix a, matrix b)
    {
        int row_Num = a.rows;
        int col_Num = a.cols;

        matrix result = new matrix(row_Num, col_Num);

        for (int i = 0; i < row_Num; i++)
        {
            for (int j = 0; j < col_Num; j++)
            {
                result[i,j] = a[i,j] + b[i,j];
            }

        }
        return result;
    }
    public static matrix operator -(matrix a)
    {
        int row_Num = a.rows;
        int col_Num = a.cols;

        matrix result = new matrix(row_Num, col_Num);

        for (int i = 0; i < row_Num; i++)
        {
            for (int j = 0; j < col_Num; j++)
            {
                result[i, j] = -a[i, j];
            }

        }
        return result;
    }
    public static matrix operator -(matrix a, matrix b)
    {
        int row_Num = a.rows;
        int col_Num = a.cols;

        matrix result = new matrix(row_Num, col_Num);

        for (int i = 0; i < row_Num; i++)
        {
            for (int j = 0; j < col_Num; j++)
            {
                result[i, j] = a[i, j] - b[i, j];
            }

        }
        return result;
    }
    public static matrix operator *(matrix a, float b)
    {
        int row_Num = a.rows;
        int col_Num = a.cols;

        matrix result = new matrix(row_Num, col_Num);

        for (int i = 0; i < row_Num; i++)
        {
            for (int j = 0; j < col_Num; j++)
            {
                result[i, j] = a[i, j] * b;
            }

        }
        return result;
    }
    public static matrix operator *(float b, matrix a)
    {
        int row_Num = a.rows;
        int col_Num = a.cols;

        matrix result = new matrix(row_Num, col_Num);

        for (int i = 0; i < row_Num; i++)
        {
            for (int j = 0; j < col_Num; j++)
            {
                result[i, j] = a[i, j] * b;
            }

        }
        return result;
    }
    public static matrix operator *(matrix target_matrixA, matrix target_matrixB)
    {
        float[][] matrixA = target_matrixA.data_Jagg;
        float[][] matrixB = target_matrixB.data_Jagg;
        // error check, compute aRows, aCols, bCols
        int aRows = matrixA.Length; int aCols = matrixA[0].Length;
        int bRows = matrixB.Length; int bCols = matrixB[0].Length;
        matrix result = new matrix(aRows, bCols);
        Parallel.For(0, aRows, i =>
        {
            for (int j = 0; j < bCols; ++j)
                for (int k = 0; k < aCols; ++k)
                    result[i, j] += matrixA[i][k] * matrixB[k][j];
        }
        );
        return result;
    }
    public static matrix operator /(matrix a, float b)
    {
        int row_Num = a.rows;
        int col_Num = a.cols;

        matrix result = new matrix(row_Num, col_Num);

        for (int i = 0; i < row_Num; i++)
        {
            for (int j = 0; j < col_Num; j++)
            {
                result[i, j] = a[i, j] / b;
            }

        }
        return result;
    }

    public static matrix transpose(matrix a)
    {
        int row_Num = a.rows;
        int col_Num = a.cols;
        var result = new matrix(col_Num, row_Num);

        for (int i = 0; i < row_Num; i++)
        {
            for (int j = 0; j < col_Num; j++)
            {
                result[j, i] = a[i, j];
            }

        }
        return result;

    }

    public static matrix dockRows(matrix a, matrix b)
    {
        int row_Num = a.rows + b.rows;
        int col_Num = a.cols;
        if(col_Num != b.cols)
        {
            throw new Exception("Size does not match");
        }

        var result = new matrix(row_Num, col_Num);
        for (int row_Index = 0; row_Index < a.rows; row_Index++)
        {
            for (int col_Index = 0; col_Index < col_Num; col_Index++)
            {
                result[row_Index, col_Index] = a[row_Index, col_Index];
            }
        }
        for (int row_Index = 0; row_Index < a.rows; row_Index++)
        {
            for (int col_Index = 0; col_Index < col_Num; col_Index++)
            {
                result[row_Index + a.rows, col_Index] = b[row_Index, col_Index];
            }
        }
        return result;
    }

    public static matrix trim(matrix target,int row_Start, int row_Num, int col_Start, int col_Num)
    {

        var result = new matrix(row_Num, col_Num);

        for (int row_Index = 0; row_Index < row_Num; row_Index++)
        {
            for (int col_Index = 0; col_Index < col_Num; col_Index++)
            {
                result[row_Index, col_Index] = target[row_Index + row_Start, col_Index + col_Start];
            }
        }
        return result;
    }

    public static Vector3 convert_Matrix_To_Vector3(matrix target)
    {
        return new Vector3(target[0, 0], target[2, 0], target[1, 0]);
    }

    public static matrix convert_Vector3_To_Matrix(Vector3 target)
    {
        return new matrix(new float[][] {
            new float[]{target.x },
            new float[]{target.y },
            new float[]{target.z }
        });
    }

}
