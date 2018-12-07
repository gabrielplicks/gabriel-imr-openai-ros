# gabriel-rmi-openai-ros
Utilização do pacote OpenAI ROS para aprendizado por reforço utilizando robôs em Gazebo

## Ambiente utilizado:
- Ubuntu 16.04
- ROS Indigo
- Gazebo Simulator

## Dependências:
- OpenAI_ROS package (https://bitbucket.org/theconstructcore/openai_ros.git)
  - Contém a integração com OpenAI e ambientes para treinar os robôs
- OpenAI_Gym_Construct package (https://bitbucket.org/theconstructcore/open_ai_gym_construct.git)
  - Contém os modelos da simulação e os worlds para o Gazebo
- Spawn_robot_tools package (https://bitbucket.org/theconstructcore/spawn_robot_tools.git)
  - Contém modelos utilizados em simulação
- VMRC package (https://bitbucket.org/theconstructcore/vmrc.git)
  - Contém os modelos da simulação para o barco do RobotX Challenge
- Hector_Gazebo packages (https://github.com/tu-darmstadt-ros-pkg/hector_gazebo)
  - Contém plugins e arquivos de world utilizados na simulação

Todas as dependências já estão incluídas neste repositório, bastando somente clonar para o seu catkin_ws.

## Utilização:

1. Clone este repositório para o seu catkin_ws.
2. Execute o comando catkin_make
3. 
