def ball_distance(neck_angle, pixel):
    pixel_squared = pixel ** 2
    coefficients = {
        35: (4.41E-05, -0.0848, 27.9),
        40: (5.48E-05, -0.0968, 33),
        45: (8.33E-05, -0.117, 39),
        50: (1.31E-04, -0.158, 51.7),
        55: (1.86E-04, -0.2, 62),
        60: (2.4E-04, -0.245, 73.4),
        65: (3.46E-04, -0.327, 91.5),
        70: (5.41E-04, -0.482, 127),
        75: (1.31E-04, -0.2443, 102.302),
        80: (1.37E-03, -1.1, 247),
        85: (1.63E-03, -1.37, 316),
        90: (2.04E-03, -1.75, 411),
        95: (2.46E-03, -2.16, 518),
        100: (3.24E-03, -3.02, 75)
    }
    
    if neck_angle in coefficients:
        a, b, c = coefficients[neck_angle]
        distance = (a * pixel_squared) + (b * pixel) + c
    else:
        raise ValueError("Invalid neck angle")
    
    return distance

if __name__ == "__main__":
    distance = ball_distance(65, 480)
    print(distance)
    distance = ball_distance(80, 480)
    print(distance)
    distance = ball_distance(100, 480)
    print(distance)
