import random

WORLD_TEMPLATE = """
<sdf version='1.6'>
  <world name='small_track'>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>-40 40 20 0 0.667643 -1</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    {lighting}

    <!-- MODELS -->
    <include>
      <uri>model://{ground_plane}</uri>
    </include>

    <model name='track'>
        {models}

        <static>0</static>
        <allow_auto_disable>1</allow_auto_disable>
    </model>

  </world>
</sdf>
"""

BLUE_CONE_TEMPLATE = """
        <include>
            <pose>{x} {y} {z} 0 0 0</pose>
            <uri>model://blue_cone</uri>
            <name>blue_cone_{id}</name>
            <covariance x="0.01" y="0.01" xy="0.0"/>
        </include>
"""

YELLOW_CONE_TEMPLATE = """
        <include>
            <pose>{x} {y} {z} 0 0 0</pose>
            <uri>model://yellow_cone</uri>
            <name>yellow_cone_{id}</name>
            <covariance x="0.01" y="0.01" xy="0.0"/>
        </include>
"""

BIG_CONE_TEMPLATE = """
        <include>
            <pose>{x} {y} {z} 0 0 0</pose>
            <uri>model://big_cone</uri>
            <name>big_cone_{id}</name>
            <covariance x="0.01" y="0.01" xy="0.0"/>
        </include>
"""


class ConeGenerator:
    # Outputs cones in model formats shown above.
    # Referenced in map.py to create the map in a gazebo template (WORLD_TEMPLATE) above

    @staticmethod
    def _get_cone(template, x, y, z, id):
        if id is None:
            id = random.randrange(0, 1e9)  # Assigns random id to cone
        return template.format(x=x, y=y, z=z, id=id)  # returns cone pose

    @staticmethod
    def get_big_cone(x=0, y=0, z=0.15, id=None):
        # returns generated big cone in BIG_CONE_TEMPLATE format
        return ConeGenerator._get_cone(BIG_CONE_TEMPLATE, x, y, z, id)

    @staticmethod
    def get_blue_cone(x=0, y=0, z=0.15, id=None):
        # returns generated blue cone in BLUE_CONE_TEMPLATE format
        return ConeGenerator._get_cone(BLUE_CONE_TEMPLATE, x, y, z, id)

    @staticmethod
    def get_yellow_cone(x=0, y=0, z=0.15, id=None):
        # returns generated yellow cone in YELLOW_CONE_TEMPLATE format
        return ConeGenerator._get_cone(YELLOW_CONE_TEMPLATE, x, y, z, id)
