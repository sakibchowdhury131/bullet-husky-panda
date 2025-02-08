import cmath

def solve_quadratic(a, b, c):
    # Calculate the discriminant
    discriminant = cmath.sqrt(b**2 - 4*a*c)
    # Calculate the two solutions
    root1 = (-b + discriminant) / (2*a)
    root2 = (-b - discriminant) / (2*a)

    return root1, root2


