from pymavlink import mavutil
import time


def command_land_drone():

    the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(f"Aterrizar ACK: {msg}")



def command_guide_mode():

    the_connection.mav.set_mode_send(
    the_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4  # Código para GUIDED
    )
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print("Cambio a modo GUIDED ACK: %s" % msg)

def command_take_off_drone():

    the_connection.mav.command_long_send(
        the_connection.target_system, 
        the_connection.target_component, 
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
        0, 0, 0, 0, 0, 0, 0, 1)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print("Drone TAKEOFF ACK: %s" % msg)


def command_disarm_drone(): #Metodo para el desarme del drone. Basicamente para apagarlo

    the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component, 
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
    0, 0, 0, 0, 0, 0, 0, 0 
    )                                                #El comando es MAV_CMD_COMPONENT_ARM_DISARM
    #Todos los 0 ceros, parametrizan el aterrizaje
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(f"Desarmar ACK: {msg}")


def command_arm_drone():

    the_connection.mav.command_long_send(
    the_connection.target_system, the_connection.target_component, 
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
      0, 1, 0, 0, 0, 0, 0, 0)
    
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(f"Armar ACK: {msg}")

if __name__ == '__main__':

    connection_string = '/dev/ttyACM0'  # Cambia esto al puerto serie correcto en tu sistema
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

    #Llamar el SITL (sim_vehicle.py -v ArduCopter -f quad --console --map --out=udp:127.0.0.1:14551)

    while True:
        mode = input("Ingrese un modo de vuelo: " )

        if mode == "1":
            command_arm_drone()
            time.sleep(1)

        elif mode == "2":
            command_guide_mode()
            time.sleep(1)

        elif mode == "3":
            command_take_off_drone()
            time.sleep(1)

        elif mode == "4":
            command_land_drone()
            time.sleep(1)

        elif mode == "5":
            command_disarm_drone()
            time.sleep(1)



