import numpy as np
import math

def algorithm(current_position, goal, lidar_data, angle_min, angle_increment, current_orientation):
    """Algoritmo de campos potenciais com controle de rotação ajustado"""
    
    # Parâmetros de modelagem
    k_atrativo = 1.0   # Atração para o objetivo
    k_repulsivo = 4.0   # Repulsão de obstáculos
    threshold = 1.0     # Distância mínima para considerar um obstáculo
    goal_threshold = 0.5  # Distância mínima para considerar o objetivo alcançado
    alignment_threshold = 0.5  # Ângulo mínimo de alinhamento
    max_linear_speed = 0.5  # Velocidade linear máxima
    max_angular_speed = 0.5  # Velocidade angular máxima

    if current_position.x - goal_threshold < goal[0] < current_position.x + goal_threshold and current_position.y - goal_threshold < goal[1] < current_position.y + goal_threshold: 
        return 0.0, 0.0

   
    dx = goal[0] - current_position.x
    dy = goal[1] - current_position.y
    distance_to_goal = math.sqrt(dx**2 + dy**2)

    desired_angle = math.atan2(dy, dx) 
    current_angle = current_orientation 


    angle_diff = desired_angle - current_angle
    angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi  # Ajusta para [-pi, pi]

    # Debug: Verificar a diferença de ângulo
    print(f"Desired angle: {desired_angle}, Current angle: {current_angle}, Angle diff: {angle_diff}")
    
    # Se o robô já estiver suficientemente alinhado, começa o movimento linear
    if abs(angle_diff) > alignment_threshold:
        # Corrigir a rotação para alinhar com o objetivo
        angular_velocity = np.clip(np.sign(angle_diff) * 0.5, -max_angular_speed, max_angular_speed)  # Girar na direção correta
        linear_velocity = 0.0  # Não move linearmente enquanto gira
    else:
        # Quando alinhado, tentar mover em direção ao objetivo
        linear_velocity = np.clip(distance_to_goal, 0, max_linear_speed)  # Limitar velocidade linear
        angular_velocity = 0.0  # Não precisa mais corrigir a rotação, pode se mover

        # Campo repulsivo (somente se necessário, quando obstáculos estão próximos)
        repulsive_field = np.array([0.0, 0.0])
        num_angles = len(lidar_data)  # Número de leituras do lidar
        
        for i in range(num_angles):
            distance = lidar_data[i]
            if distance < threshold and distance > 0:  # Obstáculo detectado
                angle_rad = angle_min + i * angle_increment  # Cálculo do ângulo correspondente
                repulse_x = -k_repulsivo * (1.0 / distance - 1.0 / threshold) * math.cos(angle_rad)
                repulse_y = -k_repulsivo * (1.0 / distance - 1.0 / threshold) * math.sin(angle_rad)
                repulsive_field += np.array([repulse_x, repulse_y])  # Soma as forças repulsivas

        # Debug: Verificar o campo repulsivo
        print(f"Repulsive field: {repulsive_field}")
        
        # Campo resultante
        total_field = np.array([k_atrativo * dx / distance_to_goal, k_atrativo * dy / distance_to_goal]) + repulsive_field

        # Cálculo da velocidade linear com base no campo resultante
        linear_velocity = np.clip(np.linalg.norm(total_field), 0, max_linear_speed)  # Velocidade linear limitada

        # Ajuste da velocidade angular (não mais interferindo se já estiver alinhado)
        angular_velocity = 0.0  # Se alinhado, não há necessidade de corrigir a rotação

    # Limitar as velocidades
    linear_velocity = np.clip(linear_velocity, 0, max_linear_speed)  # Velocidade linear limitada
    angular_velocity = np.clip(angular_velocity, -max_angular_speed, max_angular_speed)  # Velocidade angular limitada

    # Debug: Verificar as velocidades
    print(f"Linear velocity: {linear_velocity}, Angular velocity: {angular_velocity}")
    
    return linear_velocity, angular_velocity
