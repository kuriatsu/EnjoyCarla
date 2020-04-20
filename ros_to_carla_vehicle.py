#!/usr/bin/env python

class TestEgoCar(object):

	def __init__(self):
		self.client = None
		self.world = None
		self.ego_vehicle = None
		self.ego_pose = None


	def getEgoCar(self):

		for carla_actor in self.world.get_actors():
			if carla_actor.type_id.startswith("vehicle"):
				if carla_actor.attributes.get('role_name') == 'ego_vehicle':
					self.ego_vehicle = carla_actor

	def game_loop(self, args):

		try:

			self.client = carla.Client(args.host, args.port)
			self.client.set_timeout(2.0)
			self.world = self.client.get_world()
			self.getEgoCar()

			while True:

				self.world.wait_for_tick()
				self.ego_pose = self.ego_vehicle.get_transform()
				time.sleep(0.1)

		finally:
			print('exit game loop')

def main():
	argparser = argparse.ArgumentParser( description = __doc__ )
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
	args = argparser.parse_args()


	try:
		test_ego_car = TestEgoCar()
		test_ego_car.game_loop(args)
	finally:
		print('EXIT')

if __name__ == '__main__':

    try:
        main()