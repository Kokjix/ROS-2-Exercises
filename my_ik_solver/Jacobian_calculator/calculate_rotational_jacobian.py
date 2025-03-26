import sympy as sym

# ZYX rotation in current frame 

gamma, theta, phi = sym.symbols('gamma theta phi')

R_z = sym.Matrix([[sym.cos(gamma), -sym.sin(gamma), 0],
                      [sym.sin(gamma), sym.cos(gamma), 0],
                      [0, 0, 1]])

R_y = sym.Matrix([[sym.cos(theta), 0, sym.sin(theta)],
                      [0, 1, 0],
                      [-sym.sin(theta), 0, sym.cos(theta)]])

R_x = sym.Matrix([[1, 0, 0],
                    [0, sym.cos(phi), -sym.sin(phi)],
                    [0, sym.sin(phi), sym.cos(phi)]])

Re = R_z @ R_y @ R_x

# first_column1 = R_z.diff(gamma) @ R_y @ R_x @ R_x.T @ R_y.T @ R_z.T
first_column = Re.diff(gamma) @ Re.T
second_column = Re.diff(theta) @ Re.T
third_column = Re.diff(phi) @ Re.T

sym.pprint(sym.simplify(first_column))
sym.pprint(sym.simplify(second_column))
sym.pprint(sym.simplify(third_column))