package frc.robot.subsystems.Vision;

public class Matrix {

    public static double[][] multiply(double[][] mat1, double[][] mat2) {
        int rowsmat1 = mat1.length;
        int colsmat1 = mat1[0].length;
        int rowsmat2 = mat2.length;
        int colsmat2 = mat2[0].length;
        double[][] mat = new double[rowsmat1][colsmat2];
        
        if (colsmat1 != rowsmat2) {
            System.out.println("Cannot multiply matrices. Number of columns in first matrix must be equal to the number of rows in second matrix.");
            return mat;
        }
        
        for (int i = 0; i < rowsmat1; i++) {
            for (int j = 0; j < colsmat2; j++) {
                for (int k = 0; k < colsmat1; k++) {
                    mat[i][j] += mat1[i][k] * mat2[k][j];
                }
            }
        }
        
        return mat;
    }
    
    public static double[][] add(double[][] mat1, double[][] mat2) {
        int rowsmat1 = mat1.length;
        int colsmat1 = mat1[0].length;
        int rowsmat2 = mat2.length;
        int colsmat2 = mat2[0].length;
        double[][] mat = new double[rowsmat1][colsmat2];
        if (rowsmat1 != rowsmat2 || colsmat1 != colsmat2) {
            System.out.println("Cannot add matrices. Matrices must be the same dimensions");
            return mat;
        }
        for (int i = 0; i < rowsmat1; i++) {
            for (int j = 0; j < colsmat1; j++) {
                mat[i][j] = mat1[i][j] + mat2[i][j];
            }
        }
        return mat;
    }
    
}
