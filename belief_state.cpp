#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <complex>
#include <random>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> // For eigenvalues computation

// Function declarations (as provided)
void randpdm(int dim, const std::vector<double>& trace, int num, const std::string& type,
             const std::string& method, std::vector<Eigen::MatrixXd>& A);
Eigen::MatrixXd makeA(int dim, const std::vector<double>& phi, bool is_complex);
void zf_real(int n, std::vector<double>& h, std::vector<double>& g);
void zf_complex(int n, std::vector<double>& h, std::vector<double>& g);
void rndtrace(int dim, double lb, double ub, int num, const std::string& type,
              std::vector<double>& tau);

int main() {
    int dim = 3; // Dimension of the state vector and positive definite matrix

    // Sample x ∈ ℝ^dim uniformly from [-1, 1]
    Eigen::VectorXd x(dim);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> uniform_dist(-1.0, 1.0);

    for (int i = 0; i < dim; ++i) {
        x(i) = uniform_dist(gen);
    }

    // Generate a single random positive definite matrix using randpdm
    std::vector<double> trace = {1.0}; // Trace of the matrix
    int num = 1; // Number of matrices to generate
    std::string type = "real"; // "real" or "complex"
    std::string method = "rejection"; // "rejection" or "betadistr"
    std::vector<Eigen::MatrixXd> A_list; // Output matrices

    randpdm(dim, trace, num, type, method, A_list);

    // Extract the generated positive definite matrix
    Eigen::MatrixXd A = A_list[0];

    // Vectorize A by extracting upper triangular elements including the diagonal
    std::vector<double> A_vectorized;
    for (int i = 0; i < dim; ++i) {
        for (int j = i; j < dim; ++j) { // j >= i
            A_vectorized.push_back(A(i, j));
        }
    }

    // Create net vector: [x; vectorized A; x]
    std::vector<double> net_vector;
    net_vector.reserve(2 * dim + A_vectorized.size());

    // Append x to net_vector
    net_vector.insert(net_vector.end(), x.data(), x.data() + x.size());

    // Append vectorized A to net_vector
    net_vector.insert(net_vector.end(), A_vectorized.begin(), A_vectorized.end());

    // Append x again to net_vector
    net_vector.insert(net_vector.end(), x.data(), x.data() + x.size());

    // Print net vector
    std::cout << "Net vector (size " << net_vector.size() << "):\n";
    for (size_t i = 0; i < net_vector.size(); ++i) {
        std::cout << net_vector[i] << " ";
    }
    std::cout << "\n\n";

    // Print x
    std::cout << "State vector x (size " << x.size() << "):\n" << x << "\n\n";

    // Print A
    std::cout << "Positive definite matrix A (size " << dim << "x" << dim << "):\n" << A << "\n\n";

    return 0;
}

// Include all the function definitions here (as provided)

void randpdm(int dim, const std::vector<double>& trace, int num, const std::string& type,
             const std::string& method, std::vector<Eigen::MatrixXd>& A) {
    // Determine method
    bool rejection = (method == "rejection");

    // Determine type
    bool is_complex = (type == "complex");

    // Get h and g
    std::vector<double> h, g;
    if (is_complex) {
        zf_complex(dim, h, g);
    } else {
        zf_real(dim, h, g);
    }
    int phi_n = h.size();

    // Process trace
    std::vector<double> tau(num);
    if (trace.size() == 1) {
        std::fill(tau.begin(), tau.end(), trace[0]);
    } else if (trace.size() == 2) {
        if (trace[0] == trace[1]) {
            std::fill(tau.begin(), tau.end(), trace[0]);
        } else {
            rndtrace(dim, trace[0], trace[1], num, type, tau);
        }
    } else {
        throw std::invalid_argument("Trace has to be a scalar or [lb, ub]!");
    }

    // Initialize A
    A.resize(num);
    for (int i = 0; i < num; ++i) {
        A[i] = Eigen::MatrixXd::Zero(dim, dim);
    }

    // Random number generators
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> normal_dist(0.0, 1.0);
    std::uniform_real_distribution<> uniform_dist(0.0, 1.0);

    if (rejection) {
        // Rejection method
        for (int ii = 0; ii < num; ++ii) {
            std::vector<double> phi(phi_n, 0.0);
            for (int l = 0; l < phi_n; ++l) {
                double Xn;
                if (h[l] == 0) {
                    double sigma = sqrt(1.0 / g[l]);
                    while (true) {
                        Xn = normal_dist(gen) * sigma + M_PI / 2.0;
                        double Un = uniform_dist(gen);
                        double tmp;
                        if (0.0 <= Xn && Xn <= M_PI) {
                            tmp = pow(sin(Xn), g[l]);
                        } else {
                            tmp = 0.0;
                        }
                        if (Un <= tmp * exp((Xn - M_PI / 2.0) * (Xn - M_PI / 2.0) / (2.0 / g[l]))) {
                            break;
                        }
                    }
                } else {
                    double mu = atan(sqrt(g[l] / h[l]));
                    double sigma = 1.0 / (sqrt(h[l]) + sqrt(g[l]));
                    double sigmasq = sigma * sigma;
                    double a = sqrt(1.0 + g[l] / h[l]);
                    double b = sqrt(1.0 + h[l] / g[l]);
                    while (true) {
                        Xn = normal_dist(gen) * sigma + mu;
                        double Un = uniform_dist(gen);
                        double tmp;
                        if (0.0 <= Xn && Xn <= M_PI / 2.0) {
                            tmp = pow(a * cos(Xn), h[l]) * pow(b * sin(Xn), g[l]);
                        } else {
                            tmp = 0.0;
                        }
                        if (Un <= tmp * exp((Xn - mu) * (Xn - mu) / (2.0 * sigmasq))) {
                            break;
                        }
                    }
                }
                phi[l] = Xn;
            }
            A[ii] = tau[ii] * makeA(dim, phi, is_complex);
        }
    } else {
        // Beta distribution method
        for (int ii = 0; ii < num; ++ii) {
            std::vector<double> phi(phi_n, 0.0);
            // Generate random variates with beta distribution
            for (int l = 0; l < phi_n; ++l) {
                double gam_a_shape = (g[l] + 1.0) / 2.0;
                double gam_b_shape = (h[l] + 1.0) / 2.0;
                std::gamma_distribution<> gamma_a(gam_a_shape, 1.0);
                std::gamma_distribution<> gamma_b(gam_b_shape, 1.0);
                double gam_a = gamma_a(gen);
                double gam_b = gamma_b(gen);
                double y = gam_a / (gam_a + gam_b);
                phi[l] = asin(sqrt(y));
            }
            // Bernoulli distributed random variates
            for (int l = 0; l < phi_n; ++l) {
                if (h[l] == 0 && uniform_dist(gen) <= 0.5) {
                    phi[l] = M_PI - phi[l];
                }
            }
            A[ii] = tau[ii] * makeA(dim, phi, is_complex);
        }
    }
}

Eigen::MatrixXd makeA(int dim, const std::vector<double>& phi, bool is_complex) {
    if (is_complex) {
        // Complex case (not used in this example)
        Eigen::MatrixXcd T = Eigen::MatrixXcd::Zero(dim, dim);
        std::vector<double> x(phi.size() + 1, 0.0);

        double l_val = 1.0;
        for (size_t i = 0; i < phi.size(); ++i) {
            x[i] = l_val * cos(phi[i]);
            l_val *= sin(phi[i]);
        }
        x[phi.size()] = l_val;

        int idx = 0;
        for (int m = 0; m < dim; ++m) {
            int idx2 = idx + 2 * m + 1;
            for (int i = 0; i <= m; ++i) {
                double real_part = x[idx + 2 * i];
                double imag_part = (idx + 2 * i + 1 < x.size()) ? x[idx + 2 * i + 1] : 0.0;
                T(i, m) = std::complex<double>(real_part, imag_part);
            }
            idx = idx2;
        }
        return (T.adjoint() * T).real();
    } else {
        // Real case
        Eigen::MatrixXd T = Eigen::MatrixXd::Zero(dim, dim);
        std::vector<double> x(phi.size() + 1, 0.0);

        double l_val = 1.0;
        for (size_t i = 0; i < phi.size(); ++i) {
            x[i] = l_val * cos(phi[i]);
            l_val *= sin(phi[i]);
        }
        x[phi.size()] = l_val;

        int idx = 0;
        for (int m = 0; m < dim; ++m) {
            int idx2 = idx + m + 1;
            for (int i = 0; i <= m; ++i) {
                T(i, m) = x[idx + i];
            }
            idx = idx2;
        }
        return T.transpose() * T;
    }
}

void zf_real(int n, std::vector<double>& h, std::vector<double>& g) {
    int size = n * (n + 1) / 2 - 1;
    h.assign(size, 0.0);
    g.assign(size, 0.0);
    std::vector<double> a(size, 0.0);
    std::vector<double> b(size, 0.0);

    for (int k = 1; k <= n - 1; ++k) {
        int idx = k * (k + 1) / 2 - 1;
        if (idx < size) {
            h[idx] = n + 1 - k;
        }
    }

    for (int ii = 1; ii <= n - 1; ++ii) {
        for (int m = 0; m <= ii; ++m) {
            int l = ii * (ii + 1) / 2 + m - 1;
            if (l < size) {
                a[l] = ii - 1;
                b[l] = ii + 1 + m;
            }
        }
    }

    for (int i = 0; i < size; ++i) {
        g[i] = n * n - a[i] * n - b[i];
    }
}

void zf_complex(int n, std::vector<double>& h, std::vector<double>& g) {
    int size = n * n - 1;
    h.assign(size, 0.0);
    g.assign(size, 0.0);
    std::vector<double> a(size, 0.0);
    std::vector<double> b(size, 0.0);

    for (int k = 1; k <= n - 1; ++k) {
        int idx = k * k - 1;
        if (idx < size) {
            h[idx] = 2 * (n - k) + 1;
        }
    }

    for (int ii = 1; ii <= n - 1; ++ii) {
        for (int m = 0; m <= 2 * ii; ++m) {
            int l = ii * ii + m - 1;
            if (l < size) {
                a[l] = n - ii - 1;
                b[l] = (ii - 1) * n + 1 + m;
            }
        }
    }

    for (int i = 0; i < size; ++i) {
        g[i] = n * n + a[i] * n - b[i];
    }
}

void rndtrace(int dim, double lb, double ub, int num, const std::string& type,
              std::vector<double>& tau) {
    double a;
    if (type == "complex") {
        a = dim * dim;
    } else if (type == "real") {
        a = (dim * dim + dim) / 2.0;
    } else {
        throw std::invalid_argument("Unknown type. Use either 'real' or 'complex'.");
    }

    if (lb < 0) {
        throw std::invalid_argument("Trace may not be negative.");
    }

    if (ub < lb) {
        throw std::invalid_argument("Upper bound must be greater than lower bound!");
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> uniform_dist(0.0, 1.0);

    tau.resize(num);
    for (int i = 0; i < num; ++i) {
        double u = uniform_dist(gen);
        if (lb == 0) {
            tau[i] = ub * pow(u, 1.0 / a);
        } else {
            tau[i] = lb * pow(((pow(ub / lb, a) - 1) * u + 1), 1.0 / a);
        }
    }
}

