import mujoco

xml = '''<mujoco model="Quadruped URDF">
  <compiler angle="radian" meshdir="meshes/"/>

  <option>
    <flag energy="enable" multiccd="enable"/>
  </option>

  <asset>
    <mesh name="Body_bin" file="Body_bin.STL"/>
    <mesh name="Front Right Thigh" file="Front Right Thigh.STL"/>
    <mesh name="Front Right Shin" file="Front Right Shin.STL"/>
    <mesh name="Front Left Thigh" file="Front Left Thigh.STL"/>
    <mesh name="Front Left Shin" file="Front Left Shin.STL"/>
    <mesh name="Rear Right Thigh" file="Rear Right Thigh.STL"/>
    <mesh name="Rear Right Shin" file="Rear Right Shin.STL"/>
    <mesh name="Rear Left Thigh" file="Rear Left Thigh.STL"/>
    <mesh name="Rear Left Shin" file="Rear Left Shin.STL"/>
  </asset>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
     rgb2=".2 .3 .4" width="300" height="300" mark="none"/>
    <material name="grid" texture="grid" texrepeat="6 6"
     texuniform="true" reflectance=".2"/>
     <material name="wall" rgba='.5 .5 .5 1'/>
  </asset>

  <visual>
    <headlight diffuse="0.9 0.9 0.9" specular="0.6 0.6 0.6"/>
    <global offwidth="1920" offheight="1080"/>
  </visual>

  <worldbody>
    <geom name="ground" type="plane" size="100 100 100" material="grid" contype="1" conaffinity="1" friction=".7"/> 
    <body name="Body" pos="0 0 .25">
    <site name="imu" pos="0 0 0"/>
    <joint name="Body Free" type="free" stiffness="0" damping="0" frictionloss="0" armature="0"/>
      <geom name="Body" type="mesh" contype="1" conaffinity="1" group="1" density="0" rgba="0.363396 0.999339 0 1" mesh="Body_bin"/>
      <inertial pos="0.0004967 -0.0019144 -0.00656282" quat="0.499113 0.500689 -0.500109 0.500088" mass="5" diaginertia="0.03707987 0.037216486 0.00367331"/>
      <body name="Front Right Thigh" pos="-0.15605 -0.1796 0" quat="0.286214 0.646591 -0.286219 -0.646591">
        <inertial pos="2.43741e-05 -0.0142051 -0.00547111" quat="0.499113 0.500689 -0.500109 0.500088" mass="0.5" diaginertia="0.00102412 0.000703444 0.000379087"/>
        <joint name="Front Right Hip" type="hinge" pos="0 0 0" axis="0 0 1" armature=".00048" damping="1"/>
        <geom name="Front Right Thigh" type="mesh" contype="1" conaffinity="1" group="1" density="0" rgba=".580992 .393846 .712274 1" mesh="Front Right Thigh"/>
        <body name="Front Right Shin" pos="0 -0.13625 0.014875">
          <inertial pos="0.124878 -0.00732498 0.000175056" quat="0.0207105 0.70682 -0.0207132 0.706787" mass="0.25" diaginertia="5.097e-05 4.99084e-05 3.51177e-05"/>
          <joint name="Front Right Knee" type="hinge" pos="0 0 0" axis="0 0 1" armature=".00048" damping="1"/>
          <geom name="Front Right Shin" type="mesh" contype="1" conaffinity="1" group="1" density="0" rgba="0.741616 0.534085 0.405905 1" mesh="Front Right Shin"/>
        </body>
      </body>
      <body name="Front Left Thigh" pos="0.15605 -0.1796 0" quat="0.286214 0.646591 -0.286219 -0.646591">
        <inertial pos="-2.69567e-06 -0.0142673 0.00542296" quat="0.500442 0.499547 -0.499649 0.500361" mass="0.5" diaginertia="0.00102412 0.000703378 0.000379152"/>
        <joint name="Front Left Hip" type="hinge" pos="0 0 0" axis="0 0 1" armature=".00048" damping="1"/>
        <geom name = "Front Left Thigh" type="mesh" contype="1" conaffinity="1" group="1" density="0" rgba=".580992 .393846 .712274 1" mesh="Front Left Thigh"/>
        <body name="Front Left Shin" pos="0 -0.13625 -0.015125">
          <inertial pos="0.124878 -0.00732498 0.000177556" quat="0.0207105 0.70682 -0.0207132 0.706787" mass="0.25" diaginertia="5.097e-05 4.99084e-05 3.51177e-05"/>
          <joint name="Front Left Knee" type="hinge" pos="0 0 0" axis="0 0 1" armature=".00048" damping="1"/>
          <geom name="Front Left Shin" type="mesh" contype="1" conaffinity="1" group="1" density="0" rgba="0.741616 0.534085 0.405905 1" mesh="Front Left Shin"/>
        </body>
      </body>
      <body name="Rear Right Thigh" pos="-0.15605 0.1796 0" quat="0.286214 0.646591 -0.286219 -0.646591">
        <inertial pos="2.43741e-05 -0.0142051 -0.00547111" quat="0.499113 0.500689 -0.500109 0.500088" mass="0.5" diaginertia="0.00102412 0.000703444 0.000379087"/>
        <joint name="Rear Right Hip" type="hinge" pos="0 0 0" axis="0 0 1" armature=".00048" damping="1"/>
        <geom name="Rear Right Thigh" type="mesh" contype="1" conaffinity="1" group="1" density="0" rgba=".580992 .393846 .712274 1" mesh="Rear Right Thigh"/>
        <body name="Rear Right Shin" pos="0 -0.13625 0.015125">
          <inertial pos="0.124878 -0.00732498 0.000177556" quat="0.0207105 0.70682 -0.0207132 0.706787" mass="0.25" diaginertia="5.097e-05 4.99084e-05 3.51177e-05"/>
          <joint name="Rear Right Knee" type="hinge" pos="0 0 0" axis="0 0 1" armature=".00048" damping="1"/>
          <geom name="Rear Right Shin" type="mesh" contype="1" conaffinity="1" group="1" density="0" rgba="0.741616 0.534085 0.405905 1" mesh="Rear Right Shin"/>
        </body>
      </body>
      <body name="Rear Left Thigh" pos="0.15605 0.1796 0" quat="0.286215 0.646592 -0.286215 -0.646592">
        <inertial pos="-2.69567e-06 -0.0142673 0.00542296" quat="0.500442 0.499547 -0.499649 0.500361" mass="0.5" diaginertia="0.00102412 0.000703378 0.000379152"/>
        <joint name="Rear Left Hip" type="hinge" pos="0 0 0" axis="0 0 1" armature=".00048" damping="1"/>
        <geom name="Rear Left Thigh" type="mesh" contype="1" conaffinity="1" group="1" density="0" rgba=".580992 .393846 .712274 1" mesh="Rear Left Thigh"/>
        <body name="Rear Left Shin" pos="0 -0.13625 -0.015125">
          <inertial pos="0.124878 -0.00732498 0.000175056" quat="0.0207105 0.70682 -0.0207132 0.706787" mass="0.25" diaginertia="5.097e-05 4.99084e-05 3.51177e-05"/>
          <joint name="Rear Left Knee" type="hinge" pos="0 0 0" axis="0 0 1" armature=".00048" damping="1"/>
          <geom name="Rear Left Shin" type="mesh" contype="1" conaffinity="1" group="1" density="0" rgba="0.741616 0.534085 0.405905 1" mesh="Rear Left Shin"/>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="fr hip" joint="Front Right Hip"/>
    <position name="act fr hip pos" joint="Front Right Hip" kp="100" />
    <velocity name="act fr hip vel" joint="Front Right Hip" kv="1"/>
    
    <motor name="fr knee" joint="Front Right Knee"/>
    <position name="act fr knee pos" joint="Front Right Knee" kp="100" />
    <velocity name="act fr knee vel" joint="Front Right Knee" kv="1"/>
    
    <motor name="fl hip" joint="Front Left Hip"/>
    <position name="act fl hip pos" joint="Front Left Hip" kp="100" />
    <velocity name="act fl hip vel" joint="Front Left Hip" kv="1"/>
    
    <motor name="fl knee" joint="Front Left Knee"/>
    <position name="act fl knee pos" joint="Front Left Knee" kp="100" />
    <velocity name="act fl knee vel" joint="Front Left Knee" kv="1"/>
      
    <motor name="rr hip" joint="Rear Right Hip"/>
    <position name="act rr hip pos" joint="Rear Right Hip" kp="100" />
    <velocity name="act rr hip vel" joint="Rear Right Hip" kv="1"/>
      
    <motor name="rr knee" joint="Rear Right Knee"/>
    <position name="act rr knee pos" joint="Rear Right Knee" kp="100" />
    <velocity name="act rr knee vel" joint="Rear Right Knee" kv="1"/>
      
    <motor name="rl hip" joint="Rear Left Hip"/>
    <position name="act rl hip pos" joint="Rear Left Hip" kp="100" />
    <velocity name="act rl hip vel" joint="Rear Left Hip" kv="1"/>
      
    <motor name="rl knee" joint="Rear Left Knee"/> 
    <position name="act rl knee pos" joint="Rear Left Knee" kp="100" />
    <velocity name="act rl knee vel" joint="Rear Left Knee" kv="1"/>
      
  </actuator>

  <sensor>
    <framequat name="Body_Quat" objtype="site" objname="imu" />
    <gyro name="Body_Gyro" site="imu" />
    <accelerometer name="Body_Acc" site="imu" />

    <jointpos name="sens fr hip pos" joint="Front Right Hip" />
    <jointvel name="sens fr hip vel" joint="Front Right Hip" />

    <jointpos name="sens fr knee pos" joint="Front Right Knee" />
    <jointvel name="sens fr knee vel" joint="Front Right Knee" />

    <jointpos name="sens fl hip pos" joint="Front Left Hip" />
    <jointvel name="sens fl hip vel" joint="Front Left Hip" />

    <jointpos name="sens fl knee pos" joint="Front Left Knee" />
    <jointvel name="sens fl knee vel" joint="Front Left Knee" />

    <jointpos name="sens rr hip pos" joint="Rear Right Hip" />
    <jointvel name="sens rr hip vel" joint="Rear Right Hip" />

    <jointpos name="sens rr knee pos" joint="Rear Right Knee" />
    <jointvel name="sens rr knee vel" joint="Rear Right Knee" />

    <jointpos name="sens rl hip pos" joint="Rear Left Hip" />
    <jointvel name="sens rl hip vel" joint="Rear Left Hip" />

    <jointpos name="sens rl knee pos" joint="Rear Left Knee" />
    <jointvel name="sens rl knee vel" joint="Rear Left Knee" />   
  </sensor>


</mujoco>'''

model = mujoco.MjModel.from_xml_string(xml)
mujoco.mj_saveModel(model, "Quadruped MJB.mjb")

mujoco.mj_saveXML