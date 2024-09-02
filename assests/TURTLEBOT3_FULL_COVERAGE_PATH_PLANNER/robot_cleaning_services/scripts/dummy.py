from numpy import max
polygon_vertices = [[8.5, -2.75], [-5.40,-2.75], [-5.40, 3.25], [8.5, 3.25]]
polygon_vertices = [i.append(10) for i in polygon_vertices]
print(polygon_vertices)