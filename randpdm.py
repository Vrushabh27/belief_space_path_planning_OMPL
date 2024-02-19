import numpy as np
def randp(dim, trace, num):
    """
    Randomly generates positive definite matrices with constant or
    bounded trace according to a uniform distribution.

    Parameters:
    - dim: Matrix Dimension.
    - trace: Trace of the matrices. Either a scalar for a constant trace
             or a list/tuple with [lb, ub] for a bounded trace with lb < tr(A) <= ub.
    - num: Number of matrices to generate.

    Returns:
    - A 3-dimensional array containing the generated matrices.
    """
    def zf_real(n):
        # Number of elements in the arrays based on the formula n*(n+1)/2 - 1
        size = int(n * (n + 1) / 2 - 1)
        c = np.zeros(size)
        a = np.zeros(size)
        b = np.zeros(size)

        # Generate the c, a, and b arrays
        k = np.arange(1, n)  # k values from 1 to n-1 (1-based indexing to match MATLAB logic)
        
        for ii in k:
            # Calculate the index range for each ii value
            m = np.arange(0, ii + 1)
            l = ii * (ii + 1) // 2 + m - 1  # Adjust for 0-based indexing by subtracting 1

            # Assign values to a and b with proper indexing
            a[l] = ii - 1
            b[l] = ii + 1 + m

        # Correctly setting c's values where needed
        c[k * (k + 1) // 2 - 1] = n + 1 - k  # Adjust for 0-based indexing

        # Compute the s array based on a, b, and n
        s = n ** 2 - a * n - b

        return c, s


    def rndtrace(dim, lb, ub, num):
        a = (dim ** 2 + dim) / 2
        if lb < 0:
            raise ValueError('Trace may not be negative.')
        if ub < lb:
            raise ValueError('Upper bound must be greater than lower bound!')

        u = np.random.rand(num)
        if lb == 0:
            tau = ub * u ** (1 / a)
        else:
            tau = lb * (((ub / lb) ** a - 1) * u + 1) ** (1 / a)
        return tau

    def makeA(dim, phi, complex_flag):
        T = np.zeros((dim, dim), dtype=np.complex if complex_flag else np.float64)
        x = np.zeros(len(phi) + 1)
        l = 1
        for idx in range(len(phi)):
            x[idx] = l * np.cos(phi[idx])
            l *= np.sin(phi[idx])
        x[-1] = l

        if complex_flag:
            idx = 0
            for m in range(1, dim + 1):
                idx2 = idx + 2 * m - 1
                real_part = x[idx:idx2:2]
                imag_part = np.concatenate((x[idx + 1:idx2:2], [0]))
                T[:m, m - 1] = real_part + 1j * imag_part
                idx = idx2
        else:
            idx = 0
            for m in range(1, dim + 1):
                idx2 = idx + m
                T[:m, m - 1] = x[idx:idx2]
                idx = idx2

        return np.dot(T.T, T)

    h, g = zf_real(dim)
    phi_n = len(h)
    
    if isinstance(trace, (list, tuple)):
        if trace[0] == trace[1]:
            tau = np.full(num, trace[0])
        else:
            tau = rndtrace(dim, trace[0], trace[1], num)
    else:
        tau = np.full(num, trace)

    A = np.zeros((dim, dim, num))
    for ii in range(num):
        phi = np.zeros(phi_n)
        for l in range(phi_n):
            if h[l] == 0:
                sigma = np.sqrt(1 / g[l])
                while True:
                    Xn = np.random.randn() * sigma + np.pi / 2
                    Un = np.random.rand()
                    if 0 <= Xn and Xn <= np.pi:
                        tmp = np.sin(Xn) ** g[l]
                    else:
                        tmp = 0

                    if Un <= tmp * np.exp((Xn - np.pi / 2) ** 2 / (2 / g[l])):
                        break
            else:
                mu = np.arctan(np.sqrt(g[l] / h[l]))
                sigma = 1 / (np.sqrt(h[l]) + np.sqrt(g[l]))
                sigmasq = sigma ** 2

                a = np.sqrt(1 + g[l] / h[l])
                b = np.sqrt(1 + h[l] / g[l])

                while True:
                    Xn = np.random.randn() * sigma + mu
                    Un = np.random.rand()

                    if 0 <= Xn and Xn <= np.pi / 2:
                        tmp = (a * np.cos(Xn)) ** h[l] * (b * np.sin(Xn)) ** g[l]
                    else:
                        tmp = 0

                    if Un <= tmp * np.exp((Xn - mu) ** 2 / (2 * sigmasq)):
                        break
            phi[l] = Xn

        A[:, :, ii] = tau[ii] * makeA(dim, phi, False)

    return A
A=randp(2,[1,2],1)
print(A)
