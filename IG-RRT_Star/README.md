# Collision Checking Algorithm for 2D case: OMPL C++ Code (ri_rrt_star_2d.cpp)

This README provides a comprehensive overview of the collision-checking algorithm implemented in the provided OMPL C++ code. The algorithm determines whether an **ellipse** collides with a **polygon** (triangle, square, or pentagon) by transforming the ellipse into a unit circle and applying the **Separating Axis Theorem (SAT)** for efficient collision detection.

---

## Table of Contents

1. [Overview](#overview)
2. [Algorithm Steps and Code Implementation](#algorithm-steps-and-code-implementation)
    - [1. Affine Transformation of the Ellipse to a Unit Circle](#1-affine-transformation-of-the-ellipse-to-a-unit-circle)
    - [2. Transforming the Polygon](#2-transforming-the-polygon)
    - [3. Collision Detection Using SAT](#3-collision-detection-using-sat)
    - [4. Helper Functions](#4-helper-functions)

---

## Overview

The collision detection algorithm implemented in the OMPL C++ code follows a structured approach to determine collisions between an ellipse and various polygons (triangles, squares, pentagons). The primary steps involve:

1. **Affine Transformation**: Translating, rotating, and scaling the ellipse to convert it into a unit circle.
2. **Polygon Transformation**: Applying the same transformation to the polygon to maintain relative positions.
3. **Collision Detection**: Utilizing the Separating Axis Theorem (SAT) to efficiently detect collisions between the transformed unit circle and polygon.


---

## Algorithm Steps and Code Implementation

### 1. Affine Transformation of the Ellipse to a Unit Circle

**Algorithm Step:**

- **Translate**, **Rotate**, and **Scale** the ellipse to transform it into a unit circle centered at the origin.

**Code Implementation:**

```cpp
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

// Compute the scaling factors (axes lengths) of the ellipse
Eigen::VectorXd axes_lengths = eigenvalues.array().sqrt();

// Compute the rotation angle θ from the eigenvectors
double theta = std::atan2(eigenvectors(1, 0), eigenvectors(0, 0));

// Construct the affine transformation matrix M
Eigen::Matrix2d T_inv; // Inverse of the transformation matrix
T_inv = eigenvectors * axes_lengths.asDiagonal();

// Transformation matrix M = S * R
Eigen::Matrix2d M = T_inv.inverse();
```

**Explanation:**

1. **Covariance Matrix (\( \Sigma \))**: Inverts the matrix \( P \) from the state to obtain the covariance matrix \( \Sigma \).
2. **Eigen Decomposition**: Decomposes \( \Sigma \) into its eigenvalues and eigenvectors to determine the ellipse's orientation (\( \theta \)) and scaling factors (\( a \) and \( b \)).
3. **Affine Transformation Matrix (\( \mathbf{M} \))**: Constructs the transformation matrix \( \mathbf{M} \) by inverting the product of eigenvectors and the diagonal scaling matrix, effectively transforming the ellipse into a unit circle.

### 2. Transforming the Polygon

**Algorithm Step:**

- Apply the same affine transformation \( \mathbf{M} \) to each vertex of the polygon to maintain relative positions with respect to the transformed ellipse.

**Code Implementation:**

```cpp
// For each obstacle, apply the collision detection
for (const auto &obs : obstacles_)
{
    // Transform the polygon
    std::vector<Eigen::Vector2d> transformed_polygon;
    for (const auto &vertex : obs.vertices)
    {
        // Translate the vertex relative to the ellipse center
        Eigen::Vector2d translated_vertex = vertex - center;

        // Apply the affine transformation
        Eigen::Vector2d transformed_vertex = M * translated_vertex;

        transformed_polygon.push_back(transformed_vertex);
    }

    // Check collision between the unit circle and the transformed polygon
    if (isCirclePolygonColliding(transformed_polygon))
    {
        // Collision detected
        return false;
    }
}
```

**Explanation:**

1. **Translation**: Each vertex of the polygon is translated by subtracting the ellipse's center \( \mathbf{C} \), aligning the ellipse center with the origin.
2. **Affine Transformation**: The translated vertex is then transformed using the matrix \( \mathbf{M} \), scaling and rotating the polygon to align with the unit circle.
3. **Transformed Polygon**: The result is a new polygon \( \mathbf{V}_i' \) in the same space as the unit circle, prepared for collision detection.

### 3. Collision Detection Using SAT

**Algorithm Step:**

- Utilize the Separating Axis Theorem (SAT) to determine if a separating axis exists between the transformed unit circle and the transformed polygon. If such an axis exists, no collision occurs; otherwise, a collision is detected.

**Code Implementation:**

```cpp
// Function to check collision between unit circle and transformed polygon using SAT
bool isCirclePolygonColliding(const std::vector<Eigen::Vector2d> &polygon) const
{
    // Potential separating axes are the normals to the polygon edges
    std::vector<Eigen::Vector2d> axes;

    int n = polygon.size();
    for (int i = 0; i < n; ++i)
    {
        Eigen::Vector2d edge = polygon[(i + 1) % n] - polygon[i];
        Eigen::Vector2d normal(-edge.y(), edge.x()); // Normal vector
        normal.normalize();
        axes.push_back(normal);
    }

    // For each axis, project the polygon and the circle
    for (const auto &axis : axes)
    {
        // Project polygon
        double min_poly = std::numeric_limits<double>::infinity();
        double max_poly = -std::numeric_limits<double>::infinity();
        for (const auto &vertex : polygon)
        {
            double projection = axis.dot(vertex);
            if (projection < min_poly)
                min_poly = projection;
            if (projection > max_poly)
                max_poly = projection;
        }

        // Project circle (unit circle centered at origin)
        double min_circle = -1.0;
        double max_circle = 1.0;

        // Check for overlap
        if (max_poly < min_circle || max_circle < min_poly)
        {
            // Separating axis found, no collision
            return false;
        }
    }

    // Additional checks for special cases
    // 1. Is the center of the circle inside the polygon?
    if (isPointInPolygon(Eigen::Vector2d(0, 0), polygon))
    {
        // Collision detected
        return true;
    }

    // 2. Is any vertex of the polygon inside the circle?
    for (const auto &vertex : polygon)
    {
        if (vertex.norm() <= 1.0)
        {
            // Collision detected
            return true;
        }
    }

    // No separating axis found and no special cases detected, collision exists
    return true;
}
```

**Explanation:**

1. **Identifying Separating Axes**:
    - **Normals to Polygon Edges**: For each edge of the polygon, compute the perpendicular (normal) vector. These normals serve as potential separating axes.
  
2. **Projection onto Axes**:
    - **Polygon Projection**: Project all vertices of the transformed polygon onto the current axis and determine the minimum and maximum projection values.
    - **Circle Projection**: Since the circle is a unit circle centered at the origin, its projection onto any axis is \([-1, 1]\).
  
### 4. Helper Functions

**Algorithm Step:**

- Implement auxiliary functions to support the main algorithm, such as point-in-polygon tests.

**Code Implementation:**

```cpp
// Helper function to check if a point is inside a polygon (using ray casting)
bool isPointInPolygon(const Eigen::Vector2d &point, const std::vector<Eigen::Vector2d> &polygon) const
{
    int n = polygon.size();
    int crossings = 0;

    for (int i = 0; i < n; ++i)
    {
        const Eigen::Vector2d &v1 = polygon[i];
        const Eigen::Vector2d &v2 = polygon[(i + 1) % n];

        if (((v1.y() > point.y()) != (v2.y() > point.y())) &&
            (point.x() < (v2.x() - v1.x()) * (point.y() - v1.y()) / (v2.y() - v1.y() + 1e-10) + v1.x()))
        {
            crossings++;
        }
    }

    return (crossings % 2 == 1);
}
```

**Explanation:**

- **Ray Casting Algorithm**: Determines whether a given point lies inside a polygon by casting a horizontal ray from the point to infinity and counting the number of times it intersects with the edges of the polygon. An odd count indicates that the point is inside the polygon, while an even count indicates it is outside.

