/**
 * @file nls.c
 * @author 913602792@qq.com
 * @brief Nonlinear Least Squares for y = k/(x + a)^2 + b
 * @version 0.1
 * @date 2025-08-02
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

// 计算残差平方和与残差向量
double compute_residuals(int n, const double* x, const double* y, const double* p, double* r) {
    double sse = 0.0;
    double k = p[0], a = p[1], b = p[2];
    for (int i = 0; i < n; i++) {
        double denom = x[i] + a;
        // 避免除零错误
        if (fabs(denom) < 1e-5) {
            denom = (denom > 0) ? 1e-5 : -1e-5;
        }
        double model = k / (denom * denom) + b;
        r[i] = y[i] - model;
        sse += r[i] * r[i];
    }
    return sse;
}

// 计算雅可比矩阵
void compute_jacobian(int n, const double* x, const double* p, double J[][3]) {
    double k = p[0], a = p[1];
    for (int i = 0; i < n; i++) {
        double denom = x[i] + a;
        if (fabs(denom) < 1e-5) {
            denom = (denom > 0) ? 1e-5 : -1e-5;
        }
        double denom2 = denom * denom;
        double denom3 = denom2 * denom;
        J[i][0] = -1.0 / denom2;        // ∂r/∂k
        J[i][1] = 2.0 * k / denom3;     // ∂r/∂a
        J[i][2] = -1.0;                 // ∂r/∂b
    }
}

// 计算Hessian矩阵和梯度向量
void compute_hessian_gradient(int n, double J[][3], const double* r, double H[3][3], double g[3]) {
    // 清零累加器
    for (int i = 0; i < 3; i++) {
        g[i] = 0.0;
        for (int j = 0; j < 3; j++) {
            H[i][j] = 0.0;
        }
    }

    // 遍历所有数据点累加
    for (int k = 0; k < n; k++) {
        for (int i = 0; i < 3; i++) {
            g[i] += J[k][i] * r[k];
            for (int j = 0; j < 3; j++) {
                H[i][j] += J[k][i] * J[k][j];
            }
        }
    }
}

// 求解3x3线性方程组（高斯消元法）
int solve_3x3(double A[3][3], double b[3], double x[3]) {
    double aug[3][4];
    // 初始化增广矩阵
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            aug[i][j] = A[i][j];
        }
        aug[i][3] = b[i];
    }

    // 部分主元高斯消元
    for (int col = 0; col < 2; col++) {
        // 寻找主元
        int max_row = col;
        double max_val = fabs(aug[col][col]);
        for (int row = col + 1; row < 3; row++) {
            if (fabs(aug[row][col]) > max_val) {
                max_val = fabs(aug[row][col]);
                max_row = row;
            }
        }

        // 奇异矩阵检查
        if (max_val < 1e-15) return -1;

        // 行交换
        if (max_row != col) {
            for (int j = col; j < 4; j++) {
                double temp = aug[col][j];
                aug[col][j] = aug[max_row][j];
                aug[max_row][j] = temp;
            }
        }

        // 消元
        for (int row = col + 1; row < 3; row++) {
            double factor = aug[row][col] / aug[col][col];
            for (int j = col; j < 4; j++) {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // 回代求解
    if (fabs(aug[2][2]) < 1e-15) return -1;
    x[2] = aug[2][3] / aug[2][2];

    for (int i = 1; i >= 0; i--) {
        double sum = 0.0;
        for (int j = i + 1; j < 3; j++) {
            sum += aug[i][j] * x[j];
        }
        if (fabs(aug[i][i]) < 1e-15) return -1;
        x[i] = (aug[i][3] - sum) / aug[i][i];
    }
    return 0;
}

// Levenberg-Marquardt拟合主函数
int lm_fit(int n, const double* x, const double* y, double* p, int max_iter, double tol)
{
    double lambda = 0.001;                  // 初始阻尼因子
    double x_min = DBL_MAX;                  // 计算x的最小值
    for (int i = 0; i < n; i++) {
        if (x[i] < x_min) x_min = x[i];
    }

    double* r = (double*)malloc(n * sizeof(double));
    double (*J)[3] = (double(*)[3])malloc(n * 3 * sizeof(double));
    double H[3][3], H_lm[3][3], g[3];

    double SSE_old = compute_residuals(n, x, y, p, r);

    int iter;
    for (iter = 0; iter < max_iter; iter++) {
        // 1. 计算雅可比矩阵
        compute_jacobian(n, x, p, J);

        // 2. 计算Hessian矩阵和梯度
        compute_hessian_gradient(n, J, r, H, g);

        // 3. 构建阻尼Hessian: H_lm = H + lambda*I
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                H_lm[i][j] = H[i][j];
            }
            H_lm[i][i] += lambda;  // 对角线增加阻尼
        }

        // 4. 求解线性方程组 H_lm * delta = -g
        double rhs[3] = {-g[0], -g[1], -g[2]};
        double delta[3];

        if (solve_3x3(H_lm, rhs, delta) != 0) {
            lambda *= 10.0;  // 求解失败，增大阻尼
            continue;
        }

        // 5. 计算新参数
        double p_new[3];
        for (int i = 0; i < 3; i++) {
            p_new[i] = p[i] + delta[i];
        }

        // 6. 检查参数边界 (a > -x_min)
        if (p_new[1] <= -x_min + 1e-5) {
            lambda *= 10.0;
            continue;
        }

        // 7. 计算新残差
        double r_new[n];
        double SSE_new = compute_residuals(n, x, y, p_new, r_new);

        // 8. 判断是否接受新参数
        if (SSE_new < SSE_old) {
            double relative_change = (SSE_old - SSE_new) / SSE_old;

            // 更新参数和状态
            for (int i = 0; i < 3; i++) p[i] = p_new[i];
            for (int i = 0; i < n; i++) r[i] = r_new[i];
            lambda /= 10.0;
            SSE_old = SSE_new;

            // 检查收敛
            if (relative_change < tol) break;
        } else {
            lambda *= 10.0;
        }
    }
    printf("Converged after %d iterations\n", iter);

    free(r);
    free(J);
    return 0;
}

// 测试用例
int test(void) {
    // y = k/(x + a)^2 + b
    double x[] = {96,100,104,108,113,117,122,127,132,138,143,149,155,161,167,174,180,187,194,201,209,216,224,232,240,248,257,265,274,283,292,302,311,321,331,341,351,362,372,383,394,405,417,428,440,452,464,476,489,501,514,527,540,554,567,581,595,609,623,638};
    double y[] = {436.710205,437.38385,425.16214,416.550323,408.707397,396.717529,388.737915,379.677124,360.332855,350.386017,339.643829,328.499634,319.561279,311.717926,302.9935,293.305695,281.819336,271.585876,265.411407,253.208725,246.186234,236.182831,227.364685,220.565018,213.536484,206.644806,198.43985,191.110748,185.200882,178.503769,171.940582,166.764511,160.898804,155.654327,150.36615,144.590347,139.867538,134.24086,129.203415,124.835991,121.715988,116.591461,112.481178,109.30526,106.038498,102.181038,98.323502,94.395187,90.832092,87.915001,85.353912,82.131371,78.369499,76.070251,73.840935,71.420952,68.202667,66.695023,62.992683,62.187786,};
    int n = sizeof(x)/sizeof(x[0]);

    // 初始参数估计 (k, a, b)
    double p0[3] = {1.0, 0.0, 0.0};

    // 执行拟合
    lm_fit(n, x, y, p0, 100, 1e-6);

    printf("fiting result:\n");
    printf("k = %.6f\n", p0[0]);
    printf("a = %.6f\n", p0[1]);
    printf("b = %.6f\n", p0[2]);

    return 0;
}
