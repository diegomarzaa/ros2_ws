import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

import os
import json
import getch # ES NECESARIO INSTALAR ESTE PAQUETE: pip install keyboard

velocity = 0.5

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('mover_robot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.movements = []

    def mover(self, vel = 0.0, ang = 0.0):
        msg = Twist()
        msg.linear.x = vel
        msg.angular.z = ang
        print(f'Robot moviendose a {vel} m/s y girando a {ang} rad/s')
        self.publisher_.publish(msg)

    def detener(self):
        msg = Twist()
        msg.linear.x = 0.0
        print("Robot detenido")
        self.publisher_.publish(msg)


def guardar_movimientos_fichero(filename, movements):
    # Escribe los movimientos al archivo JSON
    with open(f"src/guardado_movimientos_paquete/guardado_movimientos_paquete/{filename}", "w") as f:
        f.write(json.dumps(movements, indent=4))

def cargar_movimientos_fichero(filename):
    if os.path.exists(filename):
        with open(f"src/guardado_movimientos_paquete/guardado_movimientos_paquete/{filename}", "r") as f:
            return json.load(f)
    else:
        return []
    

def texto_a_movimiento(texto:str, velocity:float) -> tuple:
    if texto == "avanzar":
        return (velocity, 0.0)
    elif texto == "retroceder":
        return (-velocity, 0.0)
    elif texto == "izquierda":
        return (0.0, velocity)
    elif texto == "derecha":
        return (0.0, -velocity)
    elif texto == "detener":
        return (0.0, 0.0)
    else:
        raise Exception("Texto no reconocido")
    

def reproducir_movimientos(node, movimientos):
    for movimiento in movimientos:
        print(f"Reproduciendo movimiento: {movimiento}")
        vel, ang = texto_a_movimiento(movimiento["mov"], movimiento["vel"])
        node.mover(vel, ang)
        time.sleep(movimiento["tiempo"])
        node.detener()

def crear_tupla_movimiento(velocity, movimiento_actual, start_time):
    moviendo = start_time is not None
    
    if moviendo:
        end_time = time.time()
        tiempo = end_time - start_time
        return (movimiento_actual, velocity, tiempo)
    else:
        return (movimiento_actual, velocity, 0.0)


def guardar_1_movimiento(node, datos_movimiento: tuple):
    mov = datos_movimiento[0]
    vel = datos_movimiento[1]
    tiempo = datos_movimiento[2]

    print(f"Guardando movimiento: {mov}")
    print(f"tiempo: {tiempo}")

    node.movements.append({
        "mov": mov,
        "vel": vel,
        "tiempo": tiempo
    })
    


def control_teclado(node):
    print("Presiona las teclas WASD para controlar el robot. Q para cancelar, Enter para guardar")
    start_time = None # Variable to store the start time of a movement
    movimiento_actual = None

    while True: # Loop until Q is pressed

        # Con esta podemos saber que movimiento se está realizando
        key = getch.getch() # Get the pressed key
        key_to_movement = {
            'w': "avanzar",
            's': "retroceder",
            'a': "izquierda",
            'd': "derecha",
            ' ': "detener",
        }

        # AL PULSAR UNA TECLA:
        # 1) Guardamos el movimiento anterior
        
        if movimiento_actual is not None:
            datos_movimiento = crear_tupla_movimiento(velocity, movimiento_actual, start_time)
            guardar_1_movimiento(node, datos_movimiento)
            print(f"movimiento guardado: {datos_movimiento}")
        
        # 3) ACTUAR SEGÚN LA TECLA PULSADA

        if key in key_to_movement:
            movimiento_actual = key_to_movement[key]
            start_time = time.time()
            vel, ang = texto_a_movimiento(movimiento_actual, velocity)
            node.mover(vel, ang)
        
        elif key == 'q':
            print("Cancelando...")
            break
        
        elif key == '\n':
            print("Guardando movimientos...")
            nombre_fichero = input("Introduce el nombre del fichero (sin el .json del final): ")
            guardar_movimientos_fichero(f"{nombre_fichero}.json", node.movements)
            break

        else:
            print("Tecla no reconocida")
            continue


def main(args=None):

    # Inicializa el nodo
    rclpy.init(args=args)
    guardado_movimientos_node = MoveRobotNode()
    choice = "..."  # Temporal

    while choice != '':
        # Muestra el menú y lee la opción elegida
        choice = input("\nEscoge qué hacer: \n1: Mover con teclado\n2: Cargar movimientos\nEnter: Quit\n")

        # GUARDA LOS MOVIMIENTOS EN UN ARCHIVO JSON
        if choice == '1':
            control_teclado(guardado_movimientos_node)


        # CARGA LOS MOVIMIENTOS DESDE UN ARCHIVO JSON
        elif choice == '2':
            nombre_fichero = input("Introduce el nombre del fichero (sin el .json del final): ")
            movimientos = cargar_movimientos_fichero(f"{nombre_fichero}.json")
            reproducir_movimientos(guardado_movimientos_node, movimientos)

    # SE HA PULSADO ENTER
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
