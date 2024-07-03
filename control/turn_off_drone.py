from pymavlink import mavutil
import time

connection_string = '/dev/ttyACM0'  # Cambia esto al puerto serie correcto en tu sistema
baud_rate = 57600  # La tasa de baudios puede variar; consulta la documentación de tu hardware

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection(connection_string, baud=baud_rate)


# Esperar el primer latido para asegurarse de que la conexión está establecida
the_connection.wait_heartbeat()
print(f"Latido recibido del sistema (system {the_connection.target_system} component {the_connection.target_component})")

# Armar el dron
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component, 
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
    0, 1, 0, 0, 0, 0, 0, 0
)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(f"Armar ACK: {msg}")

# Esperar un breve momento para asegurarse de que el dron está armado
time.sleep(5)

# Desarmar el dron
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component, 
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
    0, 0, 0, 0, 0, 0, 0, 0
)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(f"Desarmar ACK: {msg}")
