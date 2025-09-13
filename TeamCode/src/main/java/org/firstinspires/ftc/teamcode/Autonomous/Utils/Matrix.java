package org.firstinspires.ftc.teamcode.Autonomous.Utils;

//Do not delete
public class Matrix<T extends Number> {

    private T[][] matrix;
    private int rows;
    private int cols;

    public Matrix(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;

        @SuppressWarnings("unchecked")
        T[][] arr = (T[][]) new Number[rows][cols];  // you need this cast cuz of type erasure
        this.matrix = arr;
    }

    /**
     * Initializes this matrix to be a deep copy of the provided matrix.
     * @param matrix another matrix to deep copy
     */
    public Matrix(Matrix<T> matrix) {
        T[][] o = (T[][]) matrix.getMatrix();

        @SuppressWarnings("unchecked")
        T[][] copy = (T[][]) new Number[o.length][o[0].length];

        for (int i = 0; i < o.length; i++) {
            System.arraycopy(o[i], 0, copy[i], 0, o[i].length);
        }

        this.matrix = copy;
        this.rows = o.length;
        this.cols = o[0].length;
    }

    public T getElement(int row, int col) {
        return this.matrix[row][col];
    }

    public T[] getRow(int row) {
        return this.matrix[row];
    }

    public T[] getColumn(int colNum) {
        @SuppressWarnings("unchecked")
        T[] col = (T[]) new Number[rows];
        for(int i = 0; i < rows; i++) {
            col[i] = matrix[i][colNum];
        }
        return col;
    }

    public T[][] getMatrix(){
        return matrix;
    }

    public void setElement(T element, int rowNum, int colNum) {
        matrix[rowNum][colNum] = element;
    }

    public void setRow(T[] row, int rowNum) {
        matrix[rowNum] = row;
    }

    public void setColumn(T[] column, int colNum) {
        for(int i = 0; i < rows; i++){
            matrix[i][colNum] = column[i];
        }
    }

    public boolean isSquare(){
        return rows == cols;
    }

    public void swapRows(int r1, int r2){

    }

    /**
     *
     * @param b a 1xn matrix
     * @return The solution vector
     */
    public Matrix<T> AxEqualsBSolver(Matrix<T> b) {
        // find the row with the wanted column to be not zero
        // set that row to be the next row, (swap rows) make sure to swap the b vector to.
        //
    }
}