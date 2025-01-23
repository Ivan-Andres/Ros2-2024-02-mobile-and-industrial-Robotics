#librerias python para la ejecución de multiples nodos

#librerias de launch, importamos el modulo launch description
from launch import LaunchDescription
#importamos accion de nodos, que carga los nodos
from launch_ros.actions import Node

# funcion de definicion de cada nodo
def generate_launch_description():

    #definicion de cada nodo
    # funcion que crea los nodos y los crea en un objeto de nombretalker
    talker = Node(
            #paquete del nodo
            package='demo_nodes_cpp',
            #nombre del ejecutable del nodo
            executable='talker'
            )

    # Definicion del nodo (pkg_name, node_name)
    listener = Node(
            package='demo_nodes_cpp',
            executable='listener'
            )

    #retorno "generate_launch_description", separar por comas, el objeto de cada nodo
    # creamos una lista que retornara las funciones de los nodos creados
    return LaunchDescription([talker,listener])