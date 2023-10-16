import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

import os
import json
import getch    # ES NECESARIO INSTALAR ESTE PAQUETE: pip install keyboard

class MoveRobotNode(Node):
    def __init__(self):
        """
        - start_timer y end_timer:

        Sirven para calcular el tiempo que ha durado un movimiento.
        Cuando se inicia un movimiento, se guarda el tiempo en start_timer, y cuando se acaba, se guarda en end_timer.
        Luego, se calcula la diferencia entre ambos, y se guarda en el array de movimientos.
        """

        super().__init__('mover_robot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.lista_movimientos = []

        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0

        self.velocidad_defecto = 0.25

        self.start_timer = None
        self.end_timer = None


    def mover(self, vel = 0.0, ang = 0.0):
        msg = Twist()
        # ACTUALIZAR VELOCIDADES
        msg.linear.x = vel
        msg.angular.z = ang
        self.publisher_.publish(msg)
        print(f'Robot moviendose a {vel} m/s y girando a {ang} rad/s')
        # ACTUALIZAR INFORMACIÓN
        self.current_linear_velocity = vel
        self.current_angular_velocity = ang


    def detener(self):
        self.mover(0.0, 0.0)
        print("Robot detenido")



    # -------- MOVIMIENTOS ---------

    def agregar_movimiento(self, movimiento: tuple, tiempo: float):
        """
        - movimiento: tupla (velocidad_lineal, velocidad_angular)
        - tiempo: tiempo que ha durado el movimiento
        """

        print(f"Guardado movimiento: {movimiento}. Con una duración de {tiempo} segundos")

        self.lista_movimientos.append({
            "mov": movimiento,
            "tiempo": tiempo
        }) 


    def reproducir_lista_movimientos(self):
        """
        - movimientos: lista de movimientos
            - movimiento: diccionario con los datos de un movimiento
                - mov: tupla (velocidad_lineal, velocidad_angular)
                - tiempo: tiempo que ha durado el movimiento
        """
        for movimiento in self.lista_movimientos:
            print(f"Reproduciendo movimiento: {movimiento}")
            vel, ang = movimiento["mov"]
            self.mover(vel, ang)
            time.sleep(movimiento["tiempo"])
            self.detener()


    # ----------- CONTROL TECLADO -----------

    def tecla_a_movimiento(self, tecla:str) -> tuple:
        """
        - recibe una tecla

        - Devolvemos una tupla, ej: (0.5, 0), con la velocidad lineal y angular
        """
        # teclas validas
        key_to_movement = {
            'w': (self.velocidad_defecto, 0.0),
            's': (-self.velocidad_defecto, 0.0),
            'a': (0.0, self.velocidad_defecto),
            'd': (0.0, -self.velocidad_defecto),
            ' ': (0.0, 0.0),
        }
        if tecla in key_to_movement:
            return key_to_movement[tecla]
        else:
            return None


    def movimiento_actual_finalizado(self):
        """
        Con esta función indicamos que hemos acabado de realizar el movimiento actual
        - Guardamos en el array de movimientos el movimiento actual, con su duración
           - No se guarda si no había pasado un tiempo mínimo.

        - No reiniciamos el movimiento actual, en caso de querer seguir manteniendo traslacion o rotacion y cambiar el otro
        """


        if self.start_timer is None:
            # Esto es para evitar que se intente guardar un movimiento anterior al primero de todos
            print("Primera vez, ignorando")
            return
        
        self.end_timer = time.time()
        tiempo = self.end_timer - self.start_timer

        # GUARDAMOS EN EL ARRAY
        self.agregar_movimiento((self.current_linear_velocity, self.current_angular_velocity), tiempo)  #TODO INCORPORAR LA TUPLA EN LA FUNCION DE LA CLASE
        
        # Reiniciamos el contador
        self.start_timer = None
        self.end_timer = None



    def control_teclado(self) -> str :
        """
        - Return: Tecla de interrupción (Enter o Q)
        """

        print("Presiona las teclas WASD para controlar el robot. Q para cancelar, Enter para guardar")

        while True:
            # AL PULSAR UNA TECLA:
            key = getch.getch()

            # SALIR???
            if key == 'q' or key == '\n':
                self.detener()
                return key
            
            # Asignamos + Comprobamos si es una tecla válida
            vel, ang = self.tecla_a_movimiento(key)

            if vel is not None:
                # ---------- GUARDAR MOVIMIENTO ANTERIOR
                self.movimiento_actual_finalizado()

                # ---------- ACTUALIZAR MOVIMIENTO ACTUAL
                if vel == 0.0 and ang == 0.0:
                    self.detener()
                    continue

                self.current_linear_velocity += vel
                self.current_angular_velocity += ang
                # INICIAMOS EL CONTADOR
                self.start_timer = time.time()
                # MOVEMOS EL ROBOT
                self.mover(self.current_linear_velocity, self.current_angular_velocity)


    # ----------- FICHEROS -----------
    #TODO: VALIDAR NOMBRES, QUE NO SE REPITAN, NO EN BLANCO, ETC

    def guardar_movimientos_fichero(self, filename:str):
        """
        - Guarda los movimientos en un archivo JSON
        """

        with open(f"src/guardado_movimientos_paquete/guardado_movimientos_paquete/{filename}.json", "w") as f:
            f.write(json.dumps(self.lista_movimientos, indent=4))
            print(f"Guardado en {filename}.json")
            

    def cargar_movimientos_fichero(self, filename:str):
        """
        - Return: lista de movimientos
        """
        path = os.path.dirname(os.path.abspath(__file__))

        if os.path.exists(f"{path}/{filename}.json"):
            print(f"Cargando movimientos desde {filename}.json")
            try:
                with open(f"{path}/{filename}.json", "r") as f:
                    self.lista_movimientos = json.loads(f.read())
                    print(f"Lista de movimientos cargada: {self.lista_movimientos}")
            except Exception as e:
                print(f"Error al cargar el fichero {filename}.json", e)
                self.lista_movimientos = []
        else:
            print(f"El fichero {filename}.json no existe")
            self.lista_movimientos = []
    

# ----------------------------
# ----------- MAIN -----------
# ----------------------------

def main(args=None):
    # Inicializa el nodo
    rclpy.init(args=args)
    guardado_movimientos_node = MoveRobotNode()
    choice = "..."  # Temporal

    while choice != '':
        choice = input("\nEscoge qué hacer: \n1: Mover con teclado\n2: Cargar movimientos\nEnter: Quit\n")

        # ----- MOVIMIENTO CON TECLADO
        if choice == '1':
            # Al llamar a esta linea, empezaremos a mover el robot con el teclado, hasta que se pulse enter o q
            tecla_salida_pulsada = guardado_movimientos_node.control_teclado()

            # CANCELAR
            if tecla_salida_pulsada == 'q':
                print("Cancelando...")
                continue
            
            # GUARDA LOS MOVIMIENTOS EN UN ARCHIVO JSON
            elif tecla_salida_pulsada == '\n':
                print("Guardando...")
                nombre_fichero = input("Introduce el nombre del fichero (sin el .json del final): ")
                guardado_movimientos_node.guardar_movimientos_fichero(nombre_fichero)

        # ------ CARGA LOS MOVIMIENTOS DESDE UN ARCHIVO JSON
        elif choice == '2':
            # Los colocamos en la lista del nodo, y usando la función se reproducirá.
            nombre_fichero = input("Introduce el nombre del fichero (sin el .json del final): ")
            guardado_movimientos_node.cargar_movimientos_fichero(nombre_fichero)
            guardado_movimientos_node.reproducir_lista_movimientos()

    # SE HA PULSADO ENTER
    rclpy.shutdown()


if __name__ == '__main__':
    main()
