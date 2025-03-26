import sympy as sym
import numpy as np

class Robot(object):

    def __init__(self):

        q1, q2, q3, q4, q5, q6, theta, gamma = sym.symbols('q1 q2 q3 q4 q5 q6, theta, gamma')
        self.theta = sym.Array([q1, q2, q3, q4, q5, q6])
        self.d = sym.Array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])
        self.alpha = sym.Array([sym.pi/2, 0, 0, sym.pi/2, -sym.pi/2, 0])
        self.l = sym.Array([0, -0.425, -0.39225, 0, 0, 0])

        self.Ta = sym.Matrix([[0, -sym.sin(gamma), sym.cos(gamma)*sym.cos(theta)],
                              [0, sym.cos(gamma), sym.cos(theta)*sym.sin(gamma)],
                              [1, 0, -sym.sin(theta)]])

        for i in range(6):
            setattr(self, f"T{i}{i+1}", self.create_DH_matrix(self.theta[i], self.d[i], self.alpha[i], self.l[i]))

        self.T02 = self.T01 @ self.T12
        self.T03 = self.T01 @ self.T12 @ self.T23
        self.T04 = self.T01 @ self.T12 @ self.T23 @ self.T34
        self.T05 = self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45
        self.T06 = self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.T56
        self.jacobian_A_P = self.construct_analytical_jacobian_position()
        # self.jacobian_A_O = self.construct_analytical_jacobian_oriantation()
        # self.jacobian_A = self.construct_full_analytical_jacobian()
        self.position = self.get_position_vector()

        # self.jacobian_function = sym.lambdify((self.theta, theta, gamma), self.jacobian_A, modules='numpy')


    def create_DH_matrix(self, theta, d, alpha, l):
        matrix = sym.Matrix([[sym.cos(theta), -sym.sin(theta)*sym.cos(alpha), sym.sin(theta)*sym.sin(alpha), l*sym.cos(theta)],
                             [sym.sin(theta), sym.cos(theta)*sym.cos(alpha), -sym.cos(theta)*sym.sin(alpha), l*sym.sin(theta)],
                             [0, sym.sin(alpha), sym.cos(alpha), d],
                             [0, 0, 0, 1]])
        return matrix
    
    def construct_analytical_jacobian_position(self):
        p = self.T06[:3,3]
        jacobian_A_P = p.jacobian(self.theta)
        return jacobian_A_P

    def get_ez_unit_vectors(self):
        self.e00 = sym.Matrix([0, 0, 1])
        self.e01 = self.T01[:3,2]
        self.e02 = self.T02[:3,2]
        self.e03 = self.T03[:3,2]
        self.e04 = self.T04[:3,2]
        self.e05 = self.T05[:3,2]

    def construct_geometric_jacobian_orientation(self):

        self.get_ez_unit_vectors()
        self.jacobian_G_O = sym.Matrix.hstack(self.e00, self.e01, self.e02, self.e03, self.e04, self.e05)
    
    def construct_analytical_jacobian_oriantation(self):
        self.construct_geometric_jacobian_orientation()
        jacobian_A_O = self.Ta.inv() @ self.jacobian_G_O
        return jacobian_A_O
    
    def construct_full_analytical_jacobian(self):
        jacobian_A = sym.Matrix.vstack(self.jacobian_A_P, self.jacobian_A_O)
        return jacobian_A
    
    def get_position_vector(self):
        p = self.T06[:3,3]
        return p
    
r = Robot()


function_code_position = f"""
import numpy as np

def ur_position_vector(q):
    q1, q2, q3, q4, q5, q6 = q

    return np.array({r.position.tolist()})
"""
function_code_position = function_code_position.replace("sin", "np.sin").replace("cos", "np.cos")

with open("ur_position_vector.py", "w") as f:
    f.write(function_code_position)

function_code_jacobian = f"""
import numpy as np

def jacobian_A_P(q):
    q1, q2, q3, q4, q5, q6 = q
    
    return np.array({r.jacobian_A_P.tolist()})
"""
function_code_jacobian = function_code_jacobian.replace("sin", "np.sin").replace("cos", "np.cos")

with open("jacobian_A_P_function.py", "w") as f:
    f.write(function_code_jacobian)

print("Jacobian function saved to jacobian_function.py")


