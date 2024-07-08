import math

# Reference point in Cartesian coordinates and corresponding latitude and longitude
reference_cartesian = (1, 2)
reference_geo = (47.39798846525458, 8.546177170566494)

# Conversion factors
meters_per_degree_latitude = 111000
meters_per_degree_longitude = 111000 * math.cos(math.radians(reference_geo[0]))

def convert_cartesian_to_geo(cartesian_points):
    converted_points = []
    
    for point in cartesian_points:
        # Calculate differences in meters
        delta_x = point[0] - reference_cartesian[0]
        delta_y = point[1] - reference_cartesian[1]
        
        # Convert to changes in latitude and longitude
        delta_lat = delta_x / meters_per_degree_latitude
        delta_lon = delta_y / meters_per_degree_longitude
        
        # Calculate new latitude and longitude
        new_lat = reference_geo[0] + delta_lat
        new_lon = reference_geo[1] + delta_lon
        
        converted_points.append((new_lat, new_lon))
    
    return converted_points

# Example usage:
cartesian_points = [(0, 0), (5, 5)]
geo_points = convert_cartesian_to_geo(cartesian_points)

for cartesian, geo in zip(cartesian_points, geo_points):
    print(f"Cartesian: {cartesian} -> Geo: {geo}")
