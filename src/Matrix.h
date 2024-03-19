#pragma once
#ifndef _MATRIX
#define _MATRIX

#define N 4
#include <Arduino.h>
class Matrix {
public:
    void dot(float result[][4], float A[][4], float B[][4])
    {
        int i, j, k;
        for (i = 0; i < 4; i++) {
            for (j = 0; j < 4; j++) {
                result[i][j] = 0;   // C 행렬의 (i,j) 번째 원소 초기화
                for (k = 0; k < 4; k++) {
                    result[i][j] += A[i][k] * B[k][j];   // 행렬 내적 계산식
                }
            }
        }
    }
    void dot(float result[4], float A[][4], float B[4])
    {
        int i, k;
        for (i = 0; i < 4; i++) {
            result[i] = 0;   // C 행렬의 (i,j) 번째 원소 초기화
            for (k = 0; k < 4; k++) {
                result[i] += A[i][k] * B[k];   // 행렬 내적 계산식
            }
        }
    }

    float determinant(float matrix[N][N], int n) {
        float det = 0;
        float submatrix[N][N];

        if (n == 2) {
            return ((matrix[0][0] * matrix[1][1]) - (matrix[1][0] * matrix[0][1]));
        }
        else {
            for (int x = 0; x < n; x++) {
                int sub_i = 0;
                for (int i = 1; i < n; i++) {
                    int sub_j = 0;
                    for (int j = 0; j < n; j++) {
                        if (j == x) continue;
                        submatrix[sub_i][sub_j] = matrix[i][j];
                        sub_j++;
                    }
                    sub_i++;
                }
                det = det + (matrix[0][x] * ((x % 2 == 0) ? 1 : -1) * determinant(submatrix, n - 1));
            }
        }
        return det;
    }

    float determinant(float matrix[N - 1][N - 1], int n) {
        float det = 0;
        float submatrix[N - 1][N - 1];

        if (n == 2) {
            return ((matrix[0][0] * matrix[1][1]) - (matrix[1][0] * matrix[0][1]));
        }
        else {
            for (int x = 0; x < n; x++) {
                int sub_i = 0;
                for (int i = 1; i < n; i++) {
                    int sub_j = 0;
                    for (int j = 0; j < n; j++) {
                        if (j == x) continue;
                        submatrix[sub_i][sub_j] = matrix[i][j];
                        sub_j++;
                    }
                    sub_i++;
                }
                det = det + (matrix[0][x] * ((x % 2 == 0) ? 1 : -1) * determinant(submatrix, n - 1));
            }
        }
        return det;
    }

    int inv(float inverse[N][N], float matrix[N][N]) {
        
        float det = determinant(matrix, N);
        if (det == 0) {
            return 0;
        }
        float adj_matrix[N][N];

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                int sign = ((i + j) % 2 == 0) ? 1 : -1;
                int sub_i = 0;
                int sub_j = 0;
                float submatrix[N - 1][N - 1];
                for (int x = 0; x < N; x++) {
                    if (x == i) continue;
                    sub_j = 0;
                    for (int y = 0; y < N; y++) {
                        if (y == j) continue;
                        submatrix[sub_i][sub_j] = matrix[x][y];
                        sub_j++;
                    }
                    sub_i++;
                }
                adj_matrix[i][j] = sign * determinant(submatrix, N - 1);
            }
        }

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                inverse[i][j] = adj_matrix[j][i] / det;
            }
        }
        return 1;
    }

    void add(float result[N][N], float A[4][4], float B[4][4]) {
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                result[i][j] = A[i][j] + B[i][j];
    }

    void SerialPrint(usb_serial_class &serial, float matrix[4][4]) {
        serial.print(" (");
        for (int i = 0; i < 4; i++) {
            serial.print("(");
            for (int j = 0; j < 4; j++) {
                serial.print(matrix[i][j]);
                serial.print(", ");
            }
            serial.print("), ");
        }
        serial.print(") ");
    }
    void SerialPrint(usb_serial_class &serial, float matrix[4][3]) {
        serial.print(" (");
        for (int i = 0; i < 4; i++) {
            serial.print("(");
            for (int j = 0; j < 3; j++) {
                serial.print(matrix[i][j]);
                serial.print(", ");
            }
            serial.print("), ");
        }
        serial.print(") ");
    }
    void SerialPrint(usb_serial_class &serial, float *matrix, int size) {
        serial.print(" (");
        for (int i = 0; i < size; i++) {
            serial.print(matrix[i]);
            serial.print(", ");
        }
        serial.print(") ");
    }

    void SerialPrint(HardwareSerial &serial, float matrix[4][4]) {
        serial.print(" (");
        for (int i = 0; i < 4; i++) {
            serial.print("(");
            for (int j = 0; j < 4; j++) {
                serial.print(matrix[i][j]);
                serial.print(", ");
            }
            serial.print("), ");
        }
        serial.print(") ");
    }
    void SerialPrint(HardwareSerial &serial, float matrix[4][3]) {
        serial.print(" (");
        for (int i = 0; i < 4; i++) {
            serial.print("(");
            for (int j = 0; j < 3; j++) {
                serial.print(matrix[i][j]);
                serial.print(", ");
            }
            serial.print("), ");
        }
        serial.print(") ");
    }
    void SerialPrint(HardwareSerial &serial, float *matrix, int size) {
        serial.print(" (");
        for (int i = 0; i < size; i++) {
            serial.print(matrix[i]);
            serial.print(", ");
        }
        serial.print(") ");
    }
};

#endif