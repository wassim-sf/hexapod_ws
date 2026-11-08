"""Command line: python -m hexapod_kinematics --gait tripod --vx 0.015 --cycles 3 -o tripod.csv"""
import argparse
import sys

from . import GaitParams, RobotModel, VelocityCommand, generate_gait, write_csv
from .gait import PHASES


def main() -> int:
    parser = argparse.ArgumentParser(description='Generate a gait, print its checks, and optionally write CSV.')
    parser.add_argument('--gait', choices=sorted(PHASES), default='tripod')
    parser.add_argument('--step-length', type=float, default=None, help='metres')
    parser.add_argument('--step-height', type=float, default=None, help='metres')
    parser.add_argument('--period', type=float, default=None, help='cycle period in seconds')
    parser.add_argument('--duty', type=float, default=None, help='duty factor, 0 to 1')
    parser.add_argument('--vx', type=float, default=0.015, help='m/s')
    parser.add_argument('--vy', type=float, default=0.0, help='m/s')
    parser.add_argument('--yaw-rate', type=float, default=0.0, help='rad/s')
    parser.add_argument('--cycles', type=float, default=3.0)
    parser.add_argument('--dt', type=float, default=0.02, help='seconds')
    parser.add_argument('--params', default=None, help='path to robot_parameters.yaml')
    parser.add_argument('-o', '--output', default=None, help='CSV output file')
    args = parser.parse_args()

    optional = {'step_length': args.step_length, 'step_height': args.step_height,
                'cycle_period': args.period, 'duty_factor': args.duty}
    params = GaitParams(gait=args.gait, **{k: v for k, v in optional.items() if v is not None})
    model = RobotModel.load(args.params)
    trajectory = generate_gait(model, params, VelocityCommand(args.vx, args.vy, args.yaw_rate), args.cycles, args.dt)
    print(trajectory.summary())
    if args.output:
        write_csv(model, trajectory, args.output)
        print(f'wrote {args.output}: {len(trajectory.t)} rows')
    return 0


if __name__ == '__main__':
    sys.exit(main())
