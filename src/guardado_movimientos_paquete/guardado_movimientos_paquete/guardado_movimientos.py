import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import argparse # Para poder leer argumentos desde la terminal (NO EN USO)
import os
import json
import getch # ES NECESARIO INSTALAR ESTE PAQUETE: pip install keyboard


class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('mover_robot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.movements = []

    def avanzar(self, vel):
        msg = Twist()
        msg.linear.x = vel
        print(f'Robot avanzando a {vel} m/s')
        self.publisher_.publish(msg)

    def girar(self, vel):
        msg = Twist()
        msg.angular.z = vel
        print(f'Robot girando a {vel} rad/s')
        self.publisher_.publish(msg)

    def detener(self):
        msg = Twist()
        msg.linear.x = 0.0
        print("Robot detenido")
        self.publisher_.publish(msg)


def guardar_movimientos(filename, movements):
    # Escribe los movimientos al archivo JSON
    with open(filename, "w") as f:
        f.write(json.dumps(movements, indent=4))

def guardar_movimientos(filename, movements):
    # Escribe los movimientos al archivo JSON
    with open(f"src/guardado_movimientos_paquete/guardado_movimientos_paquete/{filename}", "w") as f:
        f.write(json.dumps(movements, indent=4))

def cargar_movimientos(filename):
    if os.path.exists(filename):
        with open(f"src/guardado_movimientos_paquete/guardado_movimientos_paquete/{filename}", "r") as f:
            return json.load(f)
    else:
        return []


def main(args=None):

    rclpy.init(args=args)
    guardado_movimientos_node = MoveRobotNode()
    choice = "..."  # Temporal

    while choice != '':
        choice = input("\nEscoge qué hacer: \n0: Normal input\n1: Input Saver\n2: Input Loader\nEnter: Quit\n")

        # MOVIMIENTO NORMAL
        if choice == '0':
            try:
                velocity = float(input("Introduce:\n- Velocidad (ej. 0.5)\n"))
            except:
                print("Entrada no válida")
                continue
            
            guardado_movimientos_node.avanzar(velocity)
            time.sleep(1)
            guardado_movimientos_node.detener()
            

        # GUARDA LOS MOVIMIENTOS EN UN ARCHIVO JSON
        elif choice == '1':
            print("Presiona las teclas WASD para controlar el robot y Q para salir")
            start_time = None # Variable to store the start time of a movement
            end_time = None # Variable to store the end time of a movement
            tiempo = None # Variable to store the duration of a movement
            velocity = 0.5
            tipo_movimiento = None

            while True: # Loop until Q is pressed
                
                key = getch.getch() # Get the pressed key

                if key == 'w':
                    if start_time is None: # If this is the first movement, record the start time
                        start_time = time.time()
                    guardado_movimientos_node.avanzar(velocity)
                    mov = "avanzar"
                
                elif key == 's':
                    if start_time is None:
                        start_time = time.time()
                    guardado_movimientos_node.avanzar(-velocity)
                    mov = "retroceder"
                
                elif key == 'a':
                    if start_time is None:
                        start_time = time.time()
                    guardado_movimientos_node.girar(velocity)
                    mov = "izquierda"
                
                elif key == 'd':
                    if start_time is None:
                        start_time = time.time()
                    guardado_movimientos_node.girar(-velocity)
                    mov = "derecha"
                
                elif key == 'q':
                    break
                
                # No 
                else:
                    guardado_movimientos_node.detener()
                    if start_time is not None:

                        print(f"Guardando movimiento: {mov}")
                        end_time = time.time()
                        print(f"start_time: {start_time}")
                        print(f"end_time: {end_time}")
                        print(f"tiempo: {end_time - start_time}")
                        tiempo = end_time - start_time
                        
                        guardado_movimientos_node.movements.append({
                            "mov": mov,
                            "vel": velocity,
                            "tiempo": tiempo
                        })
                    start_time = None # Reset the start time for the next movement


            guardar_movimientos("movimientos.json", guardado_movimientos_node.movements)

        # CARGA LOS MOVIMIENTOS DESDE UN ARCHIVO JSON
        elif choice == '2':
            movimientos = cargar_movimientos("movimientos.json")
            
            for movimiento in movimientos:
                if movimiento["mov"] == "avanzar":
                    guardado_movimientos_node.avanzar(movimiento["vel"])
                elif movimiento["mov"] == "retroceder":
                    guardado_movimientos_node.avanzar(-movimiento["vel"])
                elif movimiento["mov"] == "izquierda":
                    guardado_movimientos_node.girar(movimiento["vel"])
                elif movimiento["mov"] == "derecha":
                    guardado_movimientos_node.girar(-movimiento["vel"])
                else:
                    print("Movimiento no reconocido")
                    continue

                print(f"Esperando {movimiento['tiempo']} segundos")
                time.sleep(movimiento["tiempo"])
                guardado_movimientos_node.detener()

    # SE HA PULSADO ENTER
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
