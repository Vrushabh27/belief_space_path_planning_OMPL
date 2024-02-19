#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>

Eigen::MatrixXd generatePDMatrix(int dim, double lb, double ub, std::mt19937& gen) {
    std::normal_distribution<> d(0, 1);
    
    // Generate a random symmetric matrix
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim, dim);
    for (int i = 0; i < dim; ++i) {
        for (int j = i; j < dim; ++j) {
            double value = d(gen);
            if (i == j)
                X(i, j) = value;
            else {
                X(i, j) = value;
                X(j, i) = value; // Ensure the matrix is symmetric
            }
        }
    }

    // Making the matrix positive definite & adjusting its trace
    Eigen::MatrixXd Y = X.selfadjointView<Eigen::Lower>().llt().matrixL();
    Eigen::MatrixXd positiveDefiniteMatrix = Y * Y.transpose(); // Now Y is positive definite
    double currentTrace = positiveDefiniteMatrix.trace();
    double traceAdjustmentFactor = (lb + (ub - lb) * d(gen)) / currentTrace; // Adjust trace to be within lb and ub
    positiveDefiniteMatrix *= traceAdjustmentFactor;

    return positiveDefiniteMatrix;
}

std::vector<Eigen::MatrixXd> generatePDMatrices(int dim, double lb, double ub, int num) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<Eigen::MatrixXd> matrices;

    for (int i = 0; i < num; ++i) {
        matrices.push_back(generatePDMatrix(dim, lb, ub, gen));
    }

    return matrices;
}

int main() {
    int dim = 3; // Dimension of the matrices
    double lb = 10.0, ub = 15.0; // Lower and upper bounds for the trace
    int num = 5; // Number of matrices to generate

    std::vector<Eigen::MatrixXd> matrices = generatePDMatrices(dim, lb, ub, num);

    for (size_t i = 0; i < matrices.size(); ++i) {
        std::cout << "Matrix " << i + 1 << ":\n" << matrices[i] << "\nTrace: " << matrices[i].trace() << "\n\n";
    }

    return 0;
}
