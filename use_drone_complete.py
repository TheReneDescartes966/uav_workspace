from pymavlink import mavutil
import time

def set_arming_check():
    the_connection.mav.param_set_send(
        the_connection.target_system,
        the_connection.target_component,
        b'ARMING_CHECK',
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    
    msg = the_connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
    print(f"Set arming check: {msg}")
    time.sleep(2)

def set_mode(mode):
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode
    )
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(f"Set mode to {mode} ACK: {msg}")
    time.sleep(2)

def command_set_home():
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, 0, 0, 0, 0, 0, 0, 0  # Altitud del punto de inicio en metros
    )
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(f"Set home: {msg}")
    time.sleep(2)

def command_land_drone():
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(f"Aterrizar ACK: {msg}")
    time.sleep(2)

def command_take_off_drone():
    # Cambiar a modo GUIDED antes de despegue

    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 0  # Altitud de despegue en metros
    )
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(f"Drone TAKEOFF ACK: {msg}")
    time.sleep(2)

def command_disarm_drone():
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(f"Desarmar ACK: {msg}")
    time.sleep(2)

def command_arm_drone():
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(f"Armar ACK: {msg}")
    time.sleep(2)

if __name__ == '__main__':
    connection_string = '/dev/ttyUSB0'  # Cambia esto al puerto serie correcto en tu sistema
    baud_rate = 57600  # La tasa de baudios puede variar; consulta la documentación de tu hardware
    #udpin:localhost:14551
    # Start a connection listening on a UDP port
    the_connection = mavutil.mavlink_connection(connection_string, baud=baud_rate)

    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, 
                                                          the_connection.target_component))
    
    print("Bienvenido a nuestro drone, escoge una de nuestras opciones:")
    print("\n")
    print("Son:")

    while True:
        mode = input("Ingrese un modo de vuelo: ")

        if mode == "0":
            command_set_home()
        elif mode == "1":
            set_arming_check()
            command_arm_drone()
        elif mode == "2":   
            set_mode(2)  # Modo ALT_HOLD
        elif mode == "3":
            command_take_off_drone()
        elif mode == "4":
            command_land_drone()
        elif mode == "5":
            command_disarm_drone()
        elif mode == "6":
            set_arming_check()
        elif mode == "7":
            command_arm_drone()
            
        msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
        print(msg)
#Vamos a cambiar el orden para evitar conflictos de configuracion


