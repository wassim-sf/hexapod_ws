"""Start the simulator: python -m hexapod_sim"""
import argparse
import sys

from hexapod_kinematics import RobotModel

from .app import MODES, HexapodSim


def main() -> int:
    parser = argparse.ArgumentParser(description='Browser simulator for the hexapod.')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--host', default='127.0.0.1', help='use 0.0.0.0 to allow other machines to connect')
    parser.add_argument('--params', default=None, help='path to robot_parameters.yaml')
    args = parser.parse_args()
    sim = HexapodSim(RobotModel.load(args.params), host=args.host, port=args.port, mode=MODES[0])
    print(f'Hexapod simulator: open http://localhost:{args.port} (Ctrl-C to stop)', flush=True)
    try:
        sim.run()
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
