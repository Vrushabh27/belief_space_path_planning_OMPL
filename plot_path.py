# plot_path.py

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import csv

def plot_confidence_ellipse(x, P_inv, chi_square_val, ax, **kwargs):
    # Eigenvalues and eigenvectors of the covariance matrix
    cov = np.linalg.inv(P_inv)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    # Sort the eigenvalues and eigenvectors
    order = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    # Ensure eigenvalues are positive
    if np.any(eigenvalues <= 0):
        print("Non-positive eigenvalues detected. Skipping this ellipse.")
        return

    # Calculate the angle between the x-axis and the largest eigenvector
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))

    # Width and height of the ellipse
    width = 2 * np.sqrt(eigenvalues[0] * chi_square_val)
    height = 2 * np.sqrt(eigenvalues[1] * chi_square_val)

    # Draw the ellipse
    ellipse = patches.Ellipse(
        xy=(x[0], x[1]),
        width=width,
        height=height,
        angle=angle,
        **kwargs
    )
    ax.add_patch(ellipse)

def main():
    # Read the path data from CSV
    data = []
    with open('path_data.csv', 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            P11 = float(row['P11'])
            P12 = float(row['P12'])
            P22 = float(row['P22'])
            data.append({'x': x, 'y': y, 'P11': P11, 'P12': P12, 'P22': P22})

    # Extract positions and covariances
    positions = np.array([[d['x'], d['y']] for d in data])
    covariances = []
    for d in data:
        P = np.array([[d['P11'], d['P12']],
                      [d['P12'], d['P22']]])
        covariances.append(P)

    # Plotting
    fig, ax = plt.subplots(figsize=(8, 8))

    # Plot the path
    ax.plot(positions[:,0], positions[:,1], 'o-', label='Path')

    # Plot start and goal
    ax.plot(positions[0,0], positions[0,1], 'go', markersize=10, label='Start')
    ax.plot(positions[-1,0], positions[-1,1], 'ro', markersize=10, label='Goal')

    # Plot confidence ellipsoids
    chi_square_val = 0.8
    for i, (x, P) in enumerate(zip(positions, covariances)):
        # Inverse of P
        P_inv = np.linalg.inv(P)
        # Plot the ellipse
        plot_confidence_ellipse(x, P_inv, chi_square_val, ax, edgecolor='b', facecolor='none', linewidth=1)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title('Path with Confidence Ellipsoids')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True)
    plt.show()

if __name__ == '__main__':
    main()

