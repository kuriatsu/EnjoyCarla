#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import glob
import os
import sys
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings
import random
import math
import time

class SpawnActor(object):

    def __init__(self):
        self.client = None
        self.world = None
        self.ego_vehicle = None
        self.scenario = None
        self.intrusion_thres = 3.0
        self.trigger_index = 1
        self.blueprintVehicles = None
        self.blueprintWalkers = None
        self.blueprintWalkerController = None
        self.control_actor_list = []
        self.trafficlight_list = []

    def readFile(self, filename):

        tree = ET.parse(filename)
        root = tree.getroot()
        edit_tag_list = ['transform', 'location', 'goal']

        for tag in edit_tag_list:
            for itr in root.iter(tag):
                itr.text = [float(val) for val in itr.text.split(',')]
        return root


    def getEgoCar(self):

        for carla_actor in self.world.get_actors():
            if carla_actor.type_id.startswith("vehicle"):
                if carla_actor.attributes.get('role_name') == 'ego_vehicle':
                    self.ego_vehicle = carla_actor


    def checkTrigger(self):

        if self.scenario is not None:
            ego_pose = self.ego_vehicle.get_transform()
            trigger = self.scenario[self.trigger_index]
            distance = (ego_pose.location.x - trigger[0].text[0]) ** 2 \
                       + (ego_pose.location.y - trigger[0].text[1]) ** 2

            if distance < self.intrusion_thres:
                print(self.trigger_index)
                self.spawnActor(trigger.findall('spawn'))
                self.moveActor(trigger.findall('move'))
                self.killActor(trigger.findall('kill'))
                self.controlTrafficLight(trigger.findall("trafficlight"))
                self.transportActor(trigger.findall('transport'))
                self.trigger_index += 1
                if self.trigger_index == len(self.scenario):
                    sys.exit()


    def getBlueprint(self, blueprint_list, name):

        if name == 'random':
            blueprint = random.choice(blueprint_list)
        else:
            try:
                blueprint = blueprint_list.find(name)
            except ValueError:
                blueprint = random.choice(blueprint_list)
                warnings.warn('spcecified blueprint is not exist : {}'.format(actor.find('blueprint').text))

        return blueprint

    def getTraffcLight(self):

        trafficlight_list = []

        for carla_actor in self.world.get_actors():
            if carla_actor.type_id == 'traffic.traffic_light':
                trafficlight = {}
                trafficlight['id'] = carla_actor.id
                trafficlight['location'] = carla_actor.get_location()
                trafficlight_list.append(trafficlight)

        return trafficlight_list


    def spawnActor(self, spawn_list):
        # we spawn the actor
        batch = []
        ai_walkers_list = []

        for spawn in spawn_list:
            actor_ids = {}

            # set blueprint for walker
            if 'walker' in spawn.find('type').text:

                if spawn.find('type').text == 'ai_walker':
                    ai_walkers_list.append(spawn)

                blueprint = self.getBlueprint(self.blueprintWalkers, spawn.find('blueprint').text)
                # set as not invencible
                if blueprint.has_attribute('is_invincible'):
                    blueprint.set_attribute('is_invincible', 'false')

            # set blueprint for vehicle
            elif 'vehicle' in spawn.find('type').text:
                blueprint = self.getBlueprint(self.blueprintVehicles, spawn.find('blueprint').text)

                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)

                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)

                blueprint.set_attribute('role_name', 'driver')

            # append command to spawn actor to batch
            buf = spawn.find('transform').text
            transform = carla.Transform(carla.Location(buf[0], buf[1], buf[2]), carla.Rotation(buf[3], buf[4], buf[5]))
            batch.append(carla.command.SpawnActor(blueprint, transform))
            print("spawn: " + spawn.attrib.get("id"));

        # conduct spawn actor
        results = self.client.apply_batch_sync(batch)
        for i in range(len(results)):
            spawn = spawn_list[i]
            if results[i].error:
                warnings.warn(results[i].error)
                print("removes ", spawn.attrib.get('id'))
                if spawn in ai_walkers_list:
                    ai_walkers_list.remove(spawn)
                for trigger in self.scenario.findall('trigger'):
                    for move in trigger.findall('move'):
                        if move.attrib.get('id') == spawn.attrib.get('id'):
                            trigger.remove(move)
                    for kill in trigger.findall('kill'):
                        if kill.attrib.get('id') == spawn.attrib.get('id'):
                            trigger.remove(kill)

            else:
                world_id = ET.SubElement(spawn_list[i], 'world_id')
                world_id.text = results[i].actor_id

        # append command to spawn ai_controller to batch
        batch = []
        for ai_walker in ai_walkers_list:
            batch.append(carla.command.SpawnActor(self.blueprintWalkerController, carla.Transform(), ai_walker.find('world_id').text))
        results = self.client.apply_batch_sync(batch, True)

        # conduct spawn ai controller
        for i in range(len(results)):
            if (results[i].error):
                warnings.warn(results[i].error)
            else:
                ai_controller_id = ET.SubElement(ai_walkers_list[i], 'ai_controller_id')
                ai_controller_id.text = results[i].actor_id
        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        self.world.wait_for_tick()


    def moveActor(self, move_list):

        def moveAiWalker(world_id, speed):
            try:
                actor = self.world.get_actor(world_id)
                actor.start()
                actor.go_to_location(self.world.get_random_location_from_navigation())
                actor.set_max_speed(speed)
            except:
                print('cannot start AI walker. world_id: ', world_id)
                return

        def moveAiVehicle(world_id):
            try:
                actor = self.world.get_actor(world_id)
                actor.set_autopilot(True)
            except:
                print('cannot start AI vehicle. world_id: ', world_id)
                return

        def moveInnocentActor(world_id, type, speed, goals):
            control_actor = {}
            actor = self.world.get_actor(world_id)
            control_actor['actor'] = actor
            control_actor['type'] = type
            for goal in goals:
                control_actor['goal'].append(carla.Location(goal[0], goal[1], goal[2]))

            control_actor['speed'] = speed
            self.control_actor_list.append(control_actor)

        for move in move_list:
            for spawn in self.scenario.iter('spawn'):
                if move.attrib.get('id') == spawn.attrib.get('id'):
                    type = spawn.find('type').text
                    if type == 'ai_walker':
                        moveAiWalker(spawn.find('ai_controller_id').text, float(move.find('speed').text))
                    elif type == 'ai_vehicle':
                        moveAiVehicle(spawn.find('world_id').text)
                    elif type == 'walker' or type == 'vehicle':
                        moveInnocentActor(spawn.find('world_id').text, type, float(move.find('speed').text), move.findall('goal'))
                    print("move: " + spawn.attrib.get("id"));


    def controlActor(self):
        # debug = self.world.debug
        for control_actor in self.control_actor_list:
            if control_actor['actor'].is_alive == False:
                self.control_actor_list.remove(control_actor)
                print("Innocent actor ", control_actor['actor'].id, "is dead")
                continue
            else:
                actor = control_actor['actor']
                transform = actor.get_transform()
                vector = carla.Vector3D(control_actor['goal'][0].x - transform.location.x,
                                        control_actor['goal'][0].y - transform.location.y,
                                        control_actor['goal'][0].z)
                dist = math.sqrt(vector.x ** 2 + vector.y ** 2)
                vector.x = vector.x / dist
                vector.y = vector.y / dist
                # debug.draw_arrow(begin=transform.location ,end=transform.location + carla.Location(vector.x, vector.y, 0.0), life_time=0.5)

            if control_actor['type'] == 'walker':
                if dist < 1.0 && len(control_actor['goal']) > 1:
                    control_actor['goal'].pop[0]

                control = carla.WalkerControl(direction=vector, speed=control_actor.get('speed'))
                control_actor['actor'].apply_control(control)

            elif control_actor['type'] == 'vehicle':
                # calc vel and rotation
                if dist < 1.0:
                    control_actor['actor'].set_velocity(carla.Vector3D(0.0,0.0,0.0))
                    self.control_actor_list.remove(control_actor)
                else:
                    alpha = transform.rotation.yaw - math.atan(vector.y / vector.x)
                    # alpha = math.atan(vector.y / vector.x) - transform.rotation.yaw
                    omega = 2 * control_actor['speed'] * math.sin(alpha) / dist

                    control = carla.VehicleControl()
                    velocity = carla.Vector3D()
                    velocity.x = vector.x * control_actor['speed']
                    velocity.y = vector.y  * control_actor['speed']
                    velocity.z = 0.0
                    control_actor['actor'].set_velocity(velocity)
                    control_actor['actor'].set_angular_velocity(carla.Vector3D(0.0, 0.0, omega))


    def transportActor(self, transport_list):
        batch = []
        for transport in transport_list:
            for spawn in self.scenario.iter('spawn'):
                if transport.attrib.get('id') == spawn.attrib.get('id'):
                    transform = transport.find('transform').text
                    batch.append(carla.command.ApplyTransform(spawn.attrib.get('world_id'), carla.Transform(transform[0], transform[1], transform[2], transform[3], transform[4], transform[5]))

        results = self.client.apply_batch_sync(batch, True)

        for result in results:
            if (result.error):
                warning.warn(result.error)


    def killActor(self, death_note):
        batch = []
        for death in death_note:
            for spawn in self.scenario.iter('spawn'):
                if death.attrib.get('id') == spawn.attrib.get('id'):
                    if spawn.find('type').text == 'ai_walker':
                        ai_controller_id = spawn.find('ai_controller_id').text
                        self.world.get_actor(ai_controller_id).stop()
                        batch.append(carla.command.DestroyActor(ai_controller_id))

                    batch.append(carla.command.DestroyActor(spawn.find('world_id').text))
                    spawn.remove(spawn.find('world_id'))
                    print("killed: " + spawn.attrib.get("id"));

        self.client.apply_batch(batch)


    def controlTrafficLight(self, light_list):
        for controll_light in light_list:
            for trafficlight in self.trafficlight_list:
                if ((controll_light.find('location').text[0] - trafficlight.get('location').x) ** 2 + (controll_light.find('location').text[1] - trafficlight['location'].y) ** 2 < 9.0):

                    actor = self.world.get_actor(trafficlight.get('id'))
                    print('control trafficlight')

                    if (controll_light.find('state').text == 'red'):
                        actor.set_state(carla.TrafficLightState.Red)
                        actor.set_red_time(float(controll_light.find('time').text))

                    elif (controll_light.find('state').text == 'green'):
                        actor.set_green_time(float(controll_light.find('time').text))
                        actor.set_state(carla.TrafficLightState.Green)

                    continue

    def game_loop(self, args):
        print('hello game_loop')
        try:
            print('hello try')

            self.client = carla.Client(args.host, args.port)
            self.client.set_timeout(2.0)
            self.world = self.client.get_world()
            self.blueprintVehicles = self.world.get_blueprint_library().filter('vehicle.*')
            self.blueprintWalkers = self.world.get_blueprint_library().filter('walker.pedestrian*')
            self.blueprintWalkerController = self.world.get_blueprint_library().find('controller.ai.walker')
            self.scenario = self.readFile(args.scenario_file)
            self.trafficlight_list = self.getTraffcLight()
            # self.getEgoCar()
            self.spawnActor(self.scenario[0].findall('spawn'))

            while True:
                self.world.wait_for_tick()
                if self.ego_vehicle is None:
                    self.getEgoCar()
                else:
                    self.checkTrigger()
                    self.controlActor()

                time.sleep(0.1)

        finally:
            batch = []
            for carla_actor in self.world.get_actors():
                if carla_actor.type_id.startswith("vehicle") or carla_actor.type_id.startswith("walker") or carla_actor.type_id.startswith("controller"):
                    if carla_actor.attributes.get('role_name') != 'ego_vehicle':
                        batch.append(carla.command.DestroyActor(carla_actor.id))

            self.client.apply_batch(batch)
            print('collapsed')


def main():

    argparser = argparse.ArgumentParser( description = __doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--scenario_file',
        metavar='S',
        default='/home/mad-carla/share/catkin_ws/src/carla_helper/scenario.xml',
        help='scenario file (default: scenario.xml)')
    args = argparser.parse_args()

    try:
        spawn_actor = SpawnActor()
        spawn_actor.game_loop(args)
    finally:
        print('finish')

if __name__ == '__main__':
    main()
