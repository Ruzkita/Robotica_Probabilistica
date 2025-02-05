import numpy as np
import math

def algorithm(current_position, goal, lidar_data, angle_min, angle_increment, current_orientation):
    """Controle baseado totalmente no vetor de força resultante"""

    # Parâmetros de modelagem
    k_atrativo = 1.0    # Atração para o objetivo
    k_repulsivo = 3.0   # Repulsão de obstáculos
    threshold = 1.2     # Distância mínima para considerar um obstáculo
    goal_threshold = 0.5  # Distância mínima para considerar o objetivo alcançado
    max_linear_speed = 0.4  # Velocidade linear máxima
    max_angular_speed = 0.4  # Velocidade angular máxima

    # Se já estiver próximo do objetivo, para
    if abs(current_position.x - goal[0]) < goal_threshold and abs(current_position.y - goal[1]) < goal_threshold:
        return 0.0, 0.0

    # Vetor para o objetivo (campo atrativo)
    dx = goal[0] - current_position.x
    dy = goal[1] - current_position.y
    distance_to_goal = math.sqrt(dx**2 + dy**2)

    attractive_field = np.array([k_atrativo * dx / distance_to_goal, k_atrativo * dy / distance_to_goal])

    # Campo repulsivo (somente se obstáculos estiverem próximos)
    repulsive_field = np.array([0.0, 0.0])
    num_angles = len(lidar_data)

    for i in range(num_angles):
        distance = lidar_data[i]
        if 0 < distance < threshold:  # Obstáculo detectado dentro do alcance
            angle_rad = angle_min + i * angle_increment  # Ângulo do feixe LiDAR
            repulse_factor = k_repulsivo * ((1.0 / distance - 1.0 / threshold) ** 2)
            
            repulse_x = -repulse_factor * math.cos(angle_rad)
            repulse_y = -repulse_factor * math.sin(angle_rad)
            repulsive_field += np.array([repulse_x, repulse_y])  # Soma as forças repulsivas

    # Campo resultante
    total_field = attractive_field + repulsive_field

    # Cálculo do ângulo desejado baseado no campo resultante
    desired_angle = math.atan2(total_field[1], total_field[0])
    current_angle = current_orientation

    # Cálculo da diferença de ângulo ajustada para o intervalo [-pi, pi]
    angle_diff = (desired_angle - current_angle + np.pi) % (2 * np.pi) - np.pi

    # Velocidade linear proporcional à magnitude do vetor resultante
    linear_velocity = np.clip(np.linalg.norm(total_field), 0, max_linear_speed)

    # Velocidade angular proporcional à diferença de ângulo, sem forçar realinhamento
    angular_velocity = np.clip(angle_diff, -max_angular_speed, max_angular_speed)

    # Debug: Verificar os vetores de força e as velocidades
    print(f"Total field: {total_field}")
    print(f"Linear velocity: {linear_velocity}, Angular velocity: {angular_velocity}")

    return linear_velocity, angular_velocity
