🤖 SCARA Tray Planner
Este repositorio implementa la planificación y ejecución de trayectorias en un robot SCARA utilizando ROS2. Incluye nodos para el parser de archivos DXF, la resolución de cinemática inversa/directa, la interpolación de trayectorias y la visualización en RViz2 mediante un gemelo digital del robot.
📂 Estructura del repositorio
SCARA_tray_planner/
│
├── csv/                           # Archivos de trayectorias│   ├── dxf_waypoints_v5.csv       # Waypoints obtenidos desde el DXF│   └── interpolated_waypoints.csv # Waypoints interpolados (quintica)│
├── dxf_models/                    # Modelos CAD de entrada│   └── figura.dxf
│
├── models/                        # Archivos del gemelo digital│   ├── rviz_config.rviz           # Configuración pre-guardada de RViz2│   └── scara_digital_twin.urdf.xacro  # Modelo URDF del SCARA│
├── launch/                        
│   └── robot_launch_scara_trayp.launch.py   # Launch principal│
├── SCARA_tray_planner/            # Nodos principales│   ├── dxf_parser_node2.py        # Lee DXF y genera CSV con figuras e IDs│   ├── inverse_kinematics.py      # Calcula cinemática inversa y publica ángulos│   ├── trajectory_planner.py      # Interpola trayectorias (quintica)│   └── direct_kinematics.py       # Calcula cinemática directa│
└── README.md
⚙️ Instalación
Clona el repositorio en tu workspace de ROS2 y compílalo:
cd ~/ros2_ws/src
git clone <URL_DEL_REPO>cd ~/ros2_ws
colcon buildsource install/setup.bash
▶️ Ejecución de los nodos
Paso 0: Compilar el workspace
colcon buildsource install/setup.bash
Paso 1: Lanzar el robot y el parser DXF
En una terminal:
ros2 launch SCARA_tray_planner robot_launch_scara_trayp.launch.py
Este launch hace lo siguiente:
Inicia el robot_state_publisher → publica la geometría del SCARA en RViz2 usando scara_digital_twin.urdf.xacro.
Abre RViz2 con la configuración rviz_config.rviz.
Ejecuta el nodo dxf_parser_node2.py → convierte un archivo DXF en un CSV estructurado con tipo de figura e ID único por cada figura.
Paso 2: Correr la cinemática inversa
En otra terminal:
ros2 run SCARA_tray_planner inverse_kinematics
Este nodo:
Se suscribe al tópico /trayectory con mensajes tipo Twist, que contienen la posición del efector final.
Calcula la cinemática inversa para los ángulos del SCARA.
Publica resultados en:
/joint_state → de tipo JointState (posiciones de las articulaciones en RViz2).
/actual_pos → de tipo Twist (usado en la cinemática directa).
👉 Aquí deberías ver en RViz2 el SCARA moverse en función de los datos recibidos.
Paso 3: Correr la planificación de trayectorias
En una tercera terminal:
ros2 run SCARA_tray_planner trajectory_planner
Este nodo:
Lee un CSV de waypoints generado desde el parser DXF.
Aplica interpolación quíntica para suavizar la trayectoria.
Usa el tipo de figura y ID para garantizar trayectorias cerradas y controlar el eje prismático.
Publica un Path en RViz2 con ejes coordenados en el sistema fijo, que representan la pieza a realizar.
👉 En RViz2 deberías ver al robot seguir la trayectoria interpolada.
 
