import numpy
import matplotlib.pyplot as plt

def question1(A, B):

    l= numpy.shape(B)[1]
    m= numpy.shape(A)[1]
    n= numpy.shape(A)[0]
    
    C = numpy.zeros((n,l))
    for i in range(n):
        for j in range(l):
            for k in range(m):
                C[i][j] += A[i][k]*B[k][j]
                
    return C

def question2(A, B):
    n = len(A)
    
    A_transposed = numpy.zeros((n, n))
    B_transposed = numpy.zeros((n, n))
    
    for i in range(n):
        for j in range(n):
            A_transposed[j][i] = A[i][j]
            B_transposed[j][i] = B[i][j]
    C = A_transposed + B_transposed
    
    return C

def question3(A, b):
    augmented_matrix = numpy.hstack((A, b))
    
    rank_A = numpy.linalg.matrix_rank(A)
    rank_augmented = numpy.linalg.matrix_rank(augmented_matrix)
    
    if rank_A == rank_augmented and rank_A == A.shape[1]:
        x = numpy.linalg.solve(A, b)
        return x
    elif rank_A == rank_augmented and rank_A < A.shape[1]:
        return 0
    else:
        return 0

def question4(A):
    eigenvalues = numpy.linalg.eigvals(A)
    if all(numpy.real(eigenvalues) < 0):
        try:
            A_inv = numpy.linalg.inv(A)
            return A_inv
        except numpy.linalg.LinAlgError:
            return 0
    else:
        return 0


import matplotlib.pyplot as plt
import numpy as np

def question5(N=10, deltaT=0.01):
    num_steps = int(N / deltaT)
    x = numpy.zeros(num_steps)
    x[0] = 1.0
    t = numpy.zeros(num_steps)
    t[0] = 0

    for i in range(1, num_steps):
        t[i] = i * deltaT
        x_prime = -2 * x[i - 1]**3 + numpy.sin(0.5 * t[i]) * x[i - 1]  # Fix the calculation here
        x[i] = x[i - 1] + x_prime * deltaT

    time_values = numpy.arange(0, N, deltaT)

    plt.figure(figsize=(10, 6))
    plt.plot(time_values, x, label='x(t)')
    plt.xlabel('Time (seconds)')
    plt.ylabel('x(t)')
    plt.title('Integration of x\' = -2x^3 + sin(0.5t)x')
    plt.legend()
    plt.grid(True)
    plt.show()

    return x.reshape(1000,1)

if __name__ == '__main__':

    # QUESTION 1
    A = [[3, 6], [4, 5]]
    B = [[6, 4], [3, 7]]
    result = question1(A, B)
    print("Answer for the question 1 is = ", result)

    # QUESTION 2
    A = [[3, 6], [4, 5]]
    B = [[6, 4], [3, 7]]
    result = question2(A, B)
    print("Answer for the question 2 is = ", result)

    # QUESTION 3
    A = numpy.array([[1, 2, 3],
                     [4, 5, 6],
                     [7, 8, 9]])

    b = numpy.array([[1],
                     [2],
                     [3]])

    result = question3(A, b)

    if result == 0:
        print("The equation has no solution or infinite solutions.")
    else:
        print("The solution to the equation Ax = b is:")
        print("Answer for the question 3 is = ", result)

    # QUESTION 4
    A = numpy.array([[1, 2],
                     [3, 4]])

    result = question4(A)

    if result == 0:
        print("The matrix does not have all negative eigenvalues or is not invertible.")
    else:
        print("The inverse of the matrix A is:")
        print("Answer for the question 4 is = ", result)
    
    #QUESTION 5
    result = question5(10,0.01)
    print(result)