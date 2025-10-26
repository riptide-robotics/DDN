package org.firstinspires.ftc.teamcode.Autonomous.Utils;

//Do not delete

public class Matrix {

    private double[][] matrix;
    private int rows;
    private int cols;

    /**
     * Creates a matrix object.
     * @param rows number of rows
     * @param cols number of columns
     */
    public Matrix(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;
        this.matrix = new double[rows][cols];
    }

    /**
     * Initializes this matrix to be a deep copy of the provided matrix.
     * @param other another matrix to deep copy
     */
    public Matrix(Matrix other) {
        double[][] o = other.getMatrix();
        this.rows = o.length;
        this.cols = o[0].length;
        this.matrix = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            System.arraycopy(o[i], 0, this.matrix[i], 0, cols);
        }
    }

    public double getElement(int row, int col) {
        return this.matrix[row][col];
    }

    public double[] getRow(int row) {
        return this.matrix[row];
    }

    public double[] getColumn(int colNum) {
        double[] col = new double[rows];
        for (int i = 0; i < rows; i++) {
            col[i] = matrix[i][colNum];
        }
        return col;
    }

    public double[][] getMatrix() {
        return matrix;
    }

    public int getRows() {return rows;}

    public int getCols() { return this.cols;}

    public void setElement(double element, int rowNum, int colNum) {
        matrix[rowNum][colNum] = element;
    }

    public void setRow(double[] row, int rowNum) {
        if (row.length != cols) throw new IllegalArgumentException("Row length mismatch");
        matrix[rowNum] = row.clone();
    }

    public void setColumn(double[] column, int colNum) {
        if (column.length != rows) throw new IllegalArgumentException("Column length mismatch");
        for (int i = 0; i < rows; i++) {
            matrix[i][colNum] = column[i];
        }
    }

    public boolean isSquare() {
        return rows == cols;
    }

    public void swapRows(int r1, int r2) {
        double[] temp = matrix[r1];
        matrix[r1] = matrix[r2];
        matrix[r2] = temp;
    }

    public void subtractRows(int r1, int r2, int outRow) {
        double[] temp1 = matrix[r1];
        double[] temp2 = matrix[r2];

        double[] out = new double[cols];
        for (int i = 0; i < cols; i++) {
            out[i] = temp1[i] - temp2[i];
        }
        matrix[outRow] = out;
    }

    public static Matrix multiply(Matrix a, Matrix b){

        if (a.getCols() != b.getRows()) { throw new IllegalArgumentException("Columns of first don't match rows of second matrix.");}

        Matrix out = new Matrix(a.getRows(), b.getCols());

        for(int aRows = 0; aRows < a.getRows(); aRows++){
            for (int bCols = 0; bCols < b.getCols(); bCols++){

                double sum = 0;

                for(int i = 0; i < a.getCols(); i++){
                    sum += a.getElement(aRows, i) * b.getElement(i, bCols);
                }

                out.setElement(sum, aRows, bCols);
            }
        }

        return out;

    }

    /**
     * Uses Gauss-Jordan algorithm to solve matrix equation Ax = b.
     * Where A is this object
     * x is the solution vector
     * b is the right-hand side column vector
     * @param b a (rows x 1) column vector
     * @return The solution vector
     */
    public Matrix AxEqualsBSolver(Matrix b) {
        if (!this.isSquare() || b.cols != 1 || b.rows != this.rows) {
            throw new IllegalArgumentException("Incompatible dimensions");
        }

        double[][] augmentedMatrix = new double[rows][cols + 1];
        for(int i = 0; i < rows; i++){
            for(int j = 0; j < cols; j++){
               augmentedMatrix[i][j] = this.getElement(i, j);
            }
            augmentedMatrix[i][cols] = b.getElement(i, 0);
        }

        for(int i = 0; i < rows; i++){
            int switchRow = i;

            // Find largest number for stable number crunching
            for(int j = i + 1; j < rows; j++){
                if (Math.abs(augmentedMatrix[j][i]) > Math.abs(augmentedMatrix[switchRow][i])){
                    switchRow = j;
                }
            }

            if(Math.abs(augmentedMatrix[switchRow][i]) < 1e-12 ){
                throw new IllegalArgumentException("This Matrix has linearly dependent columns");
            }

            if(switchRow != i){
                //switch
                double[] tempRow = augmentedMatrix[i];
                augmentedMatrix[i] = augmentedMatrix[switchRow];
                augmentedMatrix[switchRow] = tempRow;
            }

            // scale everything by the first
            double scaleFactor = augmentedMatrix[i][i];
            for(int c = i; c < cols+1; c++){
                augmentedMatrix[i][c] /= scaleFactor;
            }

            augmentedMatrix[i][i] = 1.0;

            for (int r = 0; r < rows; r++) {
                if (r == i) continue;
                double factor = augmentedMatrix[r][i];
                for (int c = i; c < cols+1; c++) {
                    augmentedMatrix[r][c] -= factor * augmentedMatrix[i][c];
                }
                augmentedMatrix[r][i] = 0.0;
            }
        }

        double[] tx = new double[rows];
        for(int c = 0; c < rows; c++){
            tx[c] = augmentedMatrix[c][cols];
        }

        Matrix x = new Matrix(rows, 1);
        x.setColumn(tx, 0);

        return x;

    }
}
