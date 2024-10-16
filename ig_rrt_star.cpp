// rrt_star_with_custom_cost.cpp

// Include OMPL headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/config.h>

// Include standard headers and Eigen
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <complex>
#include <random>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> // For eigenvalues computation
#include <boost/math/distributions/chi_squared.hpp> // For chi-square quantiles

// Namespace shortcuts
namespace ob = ompl::base;
namespace og = ompl::geometric;

// Function declarations
void randpdm(int dim, const std::vector<double>& trace, int num, const std::string& type,
             const std::string& method, std::vector<Eigen::MatrixXd>& A);
Eigen::MatrixXd makeA(int dim, const std::vector<double>& phi, bool is_complex);
void zf_real(int n, std::vector<double>& h, std::vector<double>& g);
void zf_complex(int n, std::vector<double>& h, std::vector<double>& g);
void rndtrace(int dim, double lb, double ub, int num, const std::string& type,
              std::vector<double>& tau);

// Obstacle struct
struct Obstacle
{
    std::string type; // "triangle", "square", or "pentagon"
    std::vector<Eigen::Vector2d> vertices; // List of vertices in order
};

// Function to generate random obstacles with adjustable sizes
void generateRandomObstacles(int m1, int m2, int m3, std::vector<Obstacle> &obstacles, double workspace_min, double workspace_max, double min_size, double max_size, std::mt19937 &gen)
{
    std::uniform_real_distribution<> pos_dist(workspace_min + max_size, workspace_max - max_size);
    std::uniform_real_distribution<> size_dist(min_size, max_size); // Sizes of the obstacles

    for (int i = 0; i < m1 + m2 + m3; ++i)
    {
        Obstacle obs;
        if (i < m1)
            obs.type = "triangle";
        else if (i < m1 + m2)
            obs.type = "square";
        else
            obs.type = "pentagon";

        // Generate a random center
        double cx = pos_dist(gen);
        double cy = pos_dist(gen);
        Eigen::Vector2d center(cx, cy);

        // Generate a random size (scale)
        double scale = size_dist(gen);

        int num_vertices;
        if (obs.type == "triangle")
            num_vertices = 3;
        else if (obs.type == "square")
            num_vertices = 4;
        else // pentagon
            num_vertices = 5;

        // Generate vertices of a regular polygon
        for (int j = 0; j < num_vertices; ++j)
        {
            double angle = 2 * M_PI * j / num_vertices;
            double x = center.x() + scale * cos(angle);
            double y = center.y() + scale * sin(angle);
            obs.vertices.emplace_back(x, y);
        }
        obstacles.push_back(obs);
    }
}

// Function to write obstacles to CSV
void writeObstaclesToCSV(const std::vector<Obstacle> &obstacles, const std::string &filename)
{
    std::ofstream file(filename);
    file << "type,num_vertices,vertices\n";
    for (const auto &obs : obstacles)
    {
        file << obs.type << "," << obs.vertices.size() << ",";
        for (size_t i = 0; i < obs.vertices.size(); ++i)
        {
            file << obs.vertices[i].x() << "," << obs.vertices[i].y();
            if (i < obs.vertices.size() - 1)
                file << ",";
        }
        file << "\n";
    }
    file.close();
}

// Helper function to check if a point is inside a polygon
bool isPointInPolygon(const Eigen::Vector2d &point, const std::vector<Eigen::Vector2d> &polygon)
{
    int n = polygon.size();
    int crossing_number = 0;
    for (int i = 0; i < n; ++i)
    {
        const Eigen::Vector2d &v1 = polygon[i];
        const Eigen::Vector2d &v2 = polygon[(i + 1) % n];

        if (((v1.y() > point.y()) != (v2.y() > point.y())) &&
            (point.x() < (v2.x() - v1.x()) * (point.y() - v1.y()) / (v2.y() - v1.y() + 1e-10) + v1.x()))
        {
            crossing_number++;
        }
    }
    return (crossing_number % 2 == 1);
}

// Custom valid state sampler
class MyValidStateSampler : public ob::ValidStateSampler
{
public:
    MyValidStateSampler(const ob::SpaceInformation *si, int dimension) : ValidStateSampler(si), d(dimension)
    {
        name_ = "MyValidStateSampler";
        // Calculate the size of the net vector: d + (d*(d+1))/2
        net_vector_size = d + (d * (d + 1)) / 2;
    }

    // Generate a sample in the valid part of the state space
    bool sample(ob::State *state) override
    {
        // Sample x ∈ [-5,5]^d
        Eigen::VectorXd x(d);
        for (int i = 0; i < d; ++i) {
            x(i) = rng_.uniformReal(-5.0, 5.0);
        }

        // Trace Range Specification
        std::vector<double> trace = {0.5, 1.5}; // Modify these values to set the trace range

        int num = 1; // Number of matrices to generate
        std::string type = "real"; // "real" or "complex"
        std::string method = "rejection"; // "rejection" or "betadistr"
        std::vector<Eigen::MatrixXd> A_list; // Output matrices

        randpdm(d, trace, num, type, method, A_list);

        // Extract the generated positive definite matrix
        Eigen::MatrixXd A = A_list[0];

        // Vectorize A by extracting upper triangular elements including the diagonal
        std::vector<double> A_vectorized;
        for (int i = 0; i < d; ++i) {
            for (int j = i; j < d; ++j) { // j >= i
                A_vectorized.push_back(A(i, j));
            }
        }

        // Create net vector: [x; vectorized A]
        std::vector<double> net_vector;
        net_vector.reserve(x.size() + A_vectorized.size());

        // Append x to net_vector
        net_vector.insert(net_vector.end(), x.data(), x.data() + x.size());

        // Append vectorized A to net_vector
        net_vector.insert(net_vector.end(), A_vectorized.begin(), A_vectorized.end());

        // Assign net_vector to the state
        auto *rv_state = state->as<ob::RealVectorStateSpace::StateType>();
        for (size_t i = 0; i < net_vector.size(); ++i) {
            rv_state->values[i] = net_vector[i];
        }

        return true;
    }

    // Implement the sampleNear function
    bool sampleNear(ob::State *state, const ob::State *near, double distance) override
    {
        // For now, we'll indicate that sampling near a state is not implemented.
        return false;
    }

protected:
    ompl::RNG rng_;
    int d; // Dimension of x and A
    size_t net_vector_size; // Total size of the net vector
};

// Custom optimization objective
class MyOptimizationObjective : public ob::OptimizationObjective
{
public:
    MyOptimizationObjective(const ob::SpaceInformationPtr &si, int dimension, double alpha, const Eigen::MatrixXd &W)
        : ob::OptimizationObjective(si), d(dimension), alpha(alpha), W(W)
    {
        description_ = "Custom Optimization Objective";
    }

    // State cost (not used in this example)
    ob::Cost stateCost(const ob::State *) const override
    {
        return ob::Cost(0.0);
    }

    // Cost between two states
    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
    {
        // Extract x_k, x_{k+1}, P_k, P_{k+1}
        Eigen::VectorXd x_k(d), x_k1(d);
        Eigen::MatrixXd P_k(d, d), P_k1(d, d);

        extractState(s1, x_k, P_k);
        extractState(s2, x_k1, P_k1);

        // Compute D_travel = || x_{k+1} - x_k ||
        double D_travel = (x_k1 - x_k).norm();

        // Compute \hat{P}_{k+1} = P_k + || x_{k+1} - x_k || * W
        Eigen::MatrixXd P_hat = P_k + D_travel * W;

        // Compute D_info using the analytical solution
        double D_info = computeDInfo(P_hat, P_k1);

        // Total cost D = D_travel + alpha * D_info
        double D_total = D_travel + alpha * D_info;

        return ob::Cost(D_total);
    }

private:
    int d; // Dimension
    double alpha;
    Eigen::MatrixXd W;

    // Helper function to extract x and P from a state
    void extractState(const ob::State *state, Eigen::VectorXd &x, Eigen::MatrixXd &P) const
    {
        const auto *rv_state = state->as<ob::RealVectorStateSpace::StateType>();
        // Extract x
        for (int i = 0; i < d; ++i) {
            x(i) = rv_state->values[i];
        }
        // Extract P_vectorized
        size_t idx = d;
        std::vector<double> P_vectorized;
        P_vectorized.reserve(d * (d + 1) / 2);
        for (int i = 0; i < d; ++i) {
            for (int j = i; j < d; ++j) {
                P_vectorized.push_back(rv_state->values[idx++]);
            }
        }
        // Reconstruct P from P_vectorized
        P = Eigen::MatrixXd::Zero(d, d);
        idx = 0;
        for (int i = 0; i < d; ++i) {
            for (int j = i; j < d; ++j) {
                P(i, j) = P_vectorized[idx];
                if (i != j) {
                    P(j, i) = P_vectorized[idx]; // Since P is symmetric
                }
                ++idx;
            }
        }
    }

    // Helper function to compute D_info using the analytical solution
    double computeDInfo(const Eigen::MatrixXd &P_hat, const Eigen::MatrixXd &P_k1) const
    {
        // Compute the eigenvalues of P_{k+1}^{-1} * P_hat
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_k1.inverse() * P_hat);
        Eigen::VectorXd sigma = es.eigenvalues();

        // Ensure eigenvalues are positive
        for (int i = 0; i < sigma.size(); ++i) {
            if (sigma(i) <= 0) {
                sigma(i) = 1e-6; // Small positive value
            }
        }

        // Compute S^* = diag(min{1, sigma_i})
        Eigen::VectorXd S_star = sigma.unaryExpr([](double val) { return std::min(1.0, val); });

        // Compute log-det of P_hat and Q^*_{k+1}
        double log_det_P_hat = std::log((P_hat).determinant());

        // Compute log-det of Q^*_{k+1}
        double log_det_Q_star = std::log(P_k1.determinant()) + S_star.array().log().sum();

        double D_info = 0.5 * (log_det_P_hat - log_det_Q_star);

        return D_info;
    }
};

// Implement the randpdm function and dependencies

void randpdm(int dim, const std::vector<double>& trace, int num, const std::string& type,
             const std::string& method, std::vector<Eigen::MatrixXd>& A)
{
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

Eigen::MatrixXd makeA(int dim, const std::vector<double>& phi, bool is_complex)
{
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

void zf_real(int n, std::vector<double>& h, std::vector<double>& g)
{
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

void zf_complex(int n, std::vector<double>& h, std::vector<double>& g)
{
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
              std::vector<double>& tau)
{
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

// State validity checker with collision checking
class MyStateValidityChecker : public ob::StateValidityChecker
{
public:
    MyStateValidityChecker(const ob::SpaceInformationPtr &si, int dimension, const std::vector<Obstacle> &obstacles)
        : ob::StateValidityChecker(si), d(dimension), obstacles_(obstacles)
    {
        // Precompute chi-square value for the desired confidence level
        double confidence_level = 0.8;
        boost::math::chi_squared chi_squared_dist(d);
        chi_square_val_ = boost::math::quantile(chi_squared_dist, confidence_level);
    }

    bool isValid(const ob::State *state) const override
    {
        // Extract x and P from the state
        Eigen::VectorXd x(d);
        Eigen::MatrixXd P(d, d);
        extractState(state, x, P);

        // Check if P is positive definite
        Eigen::LLT<Eigen::MatrixXd> lltOfP(P);
        if (lltOfP.info() == Eigen::NumericalIssue) {
            // Not positive definite
            return false;
        }

        return isEllipsoidCollisionFree(x, P);
    }

    // Function to check if an ellipsoid characterized by (x, P) is collision-free
    bool isEllipsoidCollisionFree(const Eigen::VectorXd &x, const Eigen::MatrixXd &P) const
    {
        // Compute covariance matrix Σ = P^{-1}
        Eigen::MatrixXd Sigma = P.inverse();

        // Ensure Sigma is symmetric
        Sigma = (Sigma + Sigma.transpose()) / 2.0;

        // Compute eigenvalues and eigenvectors of Sigma
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Sigma);
        Eigen::VectorXd eigenvalues = es.eigenvalues();
        Eigen::MatrixXd eigenvectors = es.eigenvectors();

        // Ensure eigenvalues are positive
        for (int i = 0; i < eigenvalues.size(); ++i) {
            if (eigenvalues(i) <= 0) {
                // Not a valid covariance matrix
                return false;
            }
        }

        // Compute scaling factors
        Eigen::VectorXd axes_lengths = eigenvalues.array().sqrt();

        // Number of points to sample along the ellipse
        int N = 36; // Increased number of points for better collision detection

        double scaling_factor = sqrt(chi_square_val_);

        for (int i = 0; i < N; ++i)
        {
            double theta = 2 * M_PI * i / N;
            // Point on unit circle
            Eigen::Vector2d unit_circle_point(cos(theta), sin(theta));
            // Scale and rotate
            Eigen::Vector2d ellipse_point = x + scaling_factor * eigenvectors * axes_lengths.asDiagonal() * unit_circle_point;
            // Check if point is inside any obstacle
            for (const auto &obs : obstacles_)
            {
                if (isPointInPolygon(ellipse_point, obs.vertices))
                {
                    // Collision detected
                    return false;
                }
            }
        }

        // Ellipsoid is collision-free
        return true;
    }

    // Helper function to extract x and P from a state
    void extractState(const ob::State *state, Eigen::VectorXd &x, Eigen::MatrixXd &P) const
    {
        const auto *rv_state = state->as<ob::RealVectorStateSpace::StateType>();
        // Extract x
        for (int i = 0; i < d; ++i) {
            x(i) = rv_state->values[i];
        }
        // Extract P_vectorized
        size_t idx = d;
        std::vector<double> P_vectorized;
        P_vectorized.reserve(d * (d + 1) / 2);
        for (int i = 0; i < d; ++i) {
            for (int j = i; j < d; ++j) {
                P_vectorized.push_back(rv_state->values[idx++]);
            }
        }
        // Reconstruct P from P_vectorized
        P = Eigen::MatrixXd::Zero(d, d);
        idx = 0;
        for (int i = 0; i < d; ++i) {
            for (int j = i; j < d; ++j) {
                P(i, j) = P_vectorized[idx];
                if (i != j) {
                    P(j, i) = P_vectorized[idx]; // Since P is symmetric
                }
                ++idx;
            }
        }
    }

    // Make extractState public to allow access from MyMotionValidator
    public:
        int d;
        const std::vector<Obstacle> &obstacles_;
        double chi_square_val_; // Chi-square value for the desired confidence level
};

// Custom motion validator
class MyMotionValidator : public ob::MotionValidator
{
public:
    MyMotionValidator(const ob::SpaceInformationPtr &si, int dimension, const std::vector<Obstacle> &obstacles, const Eigen::MatrixXd &W)
        : ob::MotionValidator(si), si_(si.get()), d(dimension), obstacles_(obstacles), W(W)
    {
        validityChecker_ = std::make_shared<MyStateValidityChecker>(si, d, obstacles_);
    }

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override
    {
        // Number of interpolation steps
        int steps = 50; // Increased steps for better collision detection

        // Extract x_k, x_{k+1}, P_k, P_{k+1}
        Eigen::VectorXd x_k(d), x_k1(d);
        Eigen::MatrixXd P_k(d, d), P_k1(d, d);

        validityChecker_->extractState(s1, x_k, P_k);
        validityChecker_->extractState(s2, x_k1, P_k1);

        double total_distance = (x_k1 - x_k).norm();

        // Interpolate along the path
        for (int i = 1; i <= steps; ++i)
        {
            double t = static_cast<double>(i) / steps;
            Eigen::VectorXd x = x_k + t * (x_k1 - x_k);
            double distance = t * total_distance;

            // Compute P(t) = P_k + (t * ||x_{k+1} - x_k||) * W
            Eigen::MatrixXd P = P_k + distance * W;

            // Ensure P is symmetric
            P = (P + P.transpose()) / 2.0;

            // Check if P is positive definite
            Eigen::LLT<Eigen::MatrixXd> lltOfP(P);
            if (lltOfP.info() == Eigen::NumericalIssue) {
                // Not positive definite
                return false;
            }

            // Check if the ellipsoid at (x, P) is collision-free
            if (!validityChecker_->isEllipsoidCollisionFree(x, P))
            {
                // Collision detected
                return false;
            }
        }

        // All intermediate states are valid
        return true;
    }

    bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> & /*lastValid*/) const override
    {
        // For simplicity, we can assume that partial motions are invalid
        return checkMotion(s1, s2);
    }

private:
    ob::SpaceInformation *si_;
    int d;
    const std::vector<Obstacle> &obstacles_;
    Eigen::MatrixXd W;
    std::shared_ptr<MyStateValidityChecker> validityChecker_;
};

int main()
{
    // Set the number of times to run the planner
    int max_number = 5; // Modify this value as needed

    // Random number generator for obstacle generation
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int run_number = 1; run_number <= max_number; ++run_number)
    {
        std::cout << "Run " << run_number << " of " << max_number << std::endl;

        // Define the dimension d
        int d = 2; // 2D for easy plotting

        // Define alpha and W for the cost function
        double alpha = 0.2; // You can set this to any positive value
        Eigen::MatrixXd W = 0.001 * Eigen::MatrixXd::Identity(d, d); // Define W as an identity matrix

        // Calculate the size of the net vector: d + (d*(d+1))/2
        size_t net_vector_size = d + (d * (d + 1)) / 2;

        // Create the state space
        auto space = std::make_shared<ob::RealVectorStateSpace>(net_vector_size);

        // Set the bounds of the space
        ob::RealVectorBounds bounds(net_vector_size);
        // Set bounds for x ∈ [-5, 5]^d
        for (int i = 0; i < d; ++i) {
            bounds.setLow(i, -5.0);
            bounds.setHigh(i, 5.0);
        }
        // Adjusted bounds for P's elements
        for (size_t i = d; i < net_vector_size; ++i) {
            bounds.setLow(i, -10.0);  // Allow negative values
            bounds.setHigh(i, 10.0);  // Arbitrary upper bound
        }
        space->setBounds(bounds);

        // Create a SimpleSetup object
        og::SimpleSetup ss(space);

        // Generate obstacles
        int m1 = 3; // Number of triangles
        int m2 = 0; // Number of squares
        int m3 = 0; // Number of pentagons

        std::vector<Obstacle> obstacles;

        double workspace_min = -5.0;
        double workspace_max = 5.0;

        // Set obstacle size range
        double min_obstacle_size = 0.5; // Minimum size of obstacles
        double max_obstacle_size = 1.5; // Maximum size of obstacles

        generateRandomObstacles(m1, m2, m3, obstacles, workspace_min, workspace_max, min_obstacle_size, max_obstacle_size, gen);

        // Save obstacles to CSV file with unique name
        std::string obstacles_filename = "obstacles_" + std::to_string(run_number) + ".csv";
        writeObstaclesToCSV(obstacles, obstacles_filename);

        // Set state validity checking for this space
        auto validityChecker = std::make_shared<MyStateValidityChecker>(ss.getSpaceInformation(), d, obstacles);
        ss.setStateValidityChecker(validityChecker);

        // Set the custom valid state sampler
        ob::ValidStateSamplerAllocator samplerAllocator = [d](const ob::SpaceInformation *si) {
            return std::make_shared<MyValidStateSampler>(si, d);
        };
        ss.getSpaceInformation()->setValidStateSamplerAllocator(samplerAllocator);

        // Set the custom motion validator
        auto motionValidator = std::make_shared<MyMotionValidator>(ss.getSpaceInformation(), d, obstacles, W);
        ss.getSpaceInformation()->setMotionValidator(motionValidator);

        // Define start and goal states
        ob::ScopedState<> start(space);
        ob::ScopedState<> goal(space);

        // Sample valid start and goal states using the sampler and ensure they are valid
        auto sampler = ss.getSpaceInformation()->allocValidStateSampler();

        bool valid_start_found = false;
        bool valid_goal_found = false;

        // Sample start state
        int max_attempts = 1000;
        int attempts = 0;
        while (!valid_start_found && attempts < max_attempts)
        {
            if (!sampler->sample(start.get())) {
                std::cerr << "Failed to sample a valid start state." << std::endl;
                return 1;
            }
            if (ss.getStateValidityChecker()->isValid(start.get()))
            {
                valid_start_found = true;
            }
            attempts++;
        }
        if (!valid_start_found)
        {
            std::cerr << "Unable to find a valid start state after " << max_attempts << " attempts." << std::endl;
            continue;
        }

        // Sample goal state
        attempts = 0;
        while (!valid_goal_found && attempts < max_attempts)
        {
            if (!sampler->sample(goal.get())) {
                std::cerr << "Failed to sample a valid goal state." << std::endl;
                return 1;
            }
            if (ss.getStateValidityChecker()->isValid(goal.get()))
            {
                valid_goal_found = true;
            }
            attempts++;
        }
        if (!valid_goal_found)
        {
            std::cerr << "Unable to find a valid goal state after " << max_attempts << " attempts." << std::endl;
            continue;
        }

        ss.setStartAndGoalStates(start, goal);

        // Create an instance of your custom optimization objective
        auto optObj = std::make_shared<MyOptimizationObjective>(ss.getSpaceInformation(), d, alpha, W);
        ss.setOptimizationObjective(optObj);

        // Use the RRT* planner
        auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
        ss.setPlanner(planner);

        // Attempt to solve the problem within a given time (seconds)
        ob::PlannerStatus solved = ss.solve(20.0); // Increased time to allow for more computation

        if (solved)
        {
            std::cout << "Found solution for run " << run_number << std::endl;

            // Get the solution path
            og::PathGeometric path = ss.getSolutionPath();

            // Output the path data to a CSV file
            std::string path_filename = "path_data_rrt_" + std::to_string(run_number) + ".csv";
            std::ofstream pathFile(path_filename);
            pathFile << "x,y,P11,P12,P22\n";

            for (size_t i = 0; i < path.getStateCount(); ++i)
            {
                const ob::State *state = path.getState(i);
                Eigen::VectorXd x(d);
                Eigen::MatrixXd P(d, d);

                // Extract x and P
                const auto *rv_state = state->as<ob::RealVectorStateSpace::StateType>();
                // Extract x
                for (int j = 0; j < d; ++j) {
                    x(j) = rv_state->values[j];
                }
                // Extract P_vectorized
                size_t idx = d;
                std::vector<double> P_vectorized;
                P_vectorized.reserve(d * (d + 1) / 2);
                for (int j = 0; j < d; ++j) {
                    for (int k = j; k < d; ++k) {
                        P_vectorized.push_back(rv_state->values[idx++]);
                    }
                }
                // Reconstruct P from P_vectorized
                P = Eigen::MatrixXd::Zero(d, d);
                idx = 0;
                for (int j = 0; j < d; ++j) {
                    for (int k = j; k < d; ++k) {
                        P(j, k) = P_vectorized[idx];
                        if (j != k) {
                            P(k, j) = P_vectorized[idx]; // Since P is symmetric
                        }
                        ++idx;
                    }
                }

                // Write data to CSV file
                pathFile << x(0) << "," << x(1) << "," << P(0,0) << "," << P(0,1) << "," << P(1,1) << "\n";
            }

            pathFile.close();

            std::cout << "Path data saved to '" << path_filename << "'." << std::endl;
        }
        else
        {
            std::cout << "No solution found for run " << run_number << "." << std::endl;
        }
    }

    return 0;
}
