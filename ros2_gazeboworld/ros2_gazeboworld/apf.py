import numpy as np
import math

def algorithm(current_position, goal, lidar_data):
    """Algoritmo que vai calcular os campos e retornar a velocidade"""

    #Parâmetros de modelagem
    k_atrativo = 1.0
    k_repulsivo = 6.0
    threshold = 1.0

    #Campo atrativo
    dx = goal[0] - current_position[0]
    dy = goal[1] - current_position[1]
    distance_to_goal = math.sqrt(dx**2 + dy**2)
    atractive_field = [k_atrativo * dx / distance_to_goal, k_atrativo * dy / distance_to_goal]


    #Campo repulsivo
    repulsive_field = np.array([0.0, 0.0])
    for angle, distance in enumerate(lidar_data):
        if distance < threshold and distance > 0:
            angle_rad = math.radians(angle)
            repulse_x = -k_repulsivo * (1.0 / distance - 1.0 / threshold) * math.cos(angle_rad)
            repulse_y = -k_repulsivo * (1.0 / distance - 1.0 / threshold) * math.sin(angle_rad)
            repulsive_field = np.array([repulse_x, repulse_y])
    
    #Campo resultante
    total_field = np.array(atractive_field) + repulsive_field

    #Conversão pra velocidades
    linear = np.clip(np.linalg.norm(total_field), 0, 0.5)
    angular = math.atan2(total_field[1], total_field[0])

    return linear, angular