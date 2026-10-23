#!/usr/bin/env python3
"""One entry point for worlds, continuous demos, benchmarks and results."""
import argparse
import os
import sys

from avoidance_worlds import ROOT, WORLDS
from world_appearance import THEMES
from transport_worlds import TRANSPORT_WORLDS

ALIASES = dict(plaza='plaza', courtyard='small_open', cafe='small_clutter',
               station='medium_open', clinic='medium_doorway', concourse='large_pillars', office='large_office')


def main():
    parser = argparse.ArgumentParser(description=__doc__, epilog='Examples: ./geoff worlds | ./geoff demo cafe --people 4 | ./geoff results')
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('worlds', help='list all demo environments')
    demo = commands.add_parser('demo', help='continuous visual demo with moving waypoint goals', add_help=False)
    demo.add_argument('world', nargs='?', default='plaza', choices=[*ALIASES, *WORLDS, *TRANSPORT_WORLDS])
    commands.add_parser('benchmark', help='collect a scored world/seed matrix', add_help=False)
    commands.add_parser('transport', help='experimental RGB-D Dex3 contact-only pickup/carry/placement', add_help=False)
    commands.add_parser('results', help='open the local results GUI server', add_help=False)
    args, extra = parser.parse_known_args()
    if args.command == 'worlds':
        if extra:
            parser.error('worlds takes no extra arguments')
        print('NAME        ENVIRONMENT                 SIZE        DATASET ID')
        print('plaza       Original pedestrian plaza   original    plaza')
        for alias, world in ALIASES.items():
            if world != 'plaza':
                x, y = WORLDS[world]['size']
                print(f'{alias:<11} {THEMES[world][0]:<27} {x} x {y} m    {world}')
        for name, world in TRANSPORT_WORLDS.items():
            print(f'{name:<18} Experimental transport fixture   {world["size"][0]} x {world["size"][1]} m')
        return
    if args.command == 'demo':
        script, flags = 'onboard_visual_session.py', ['--world', ALIASES.get(args.world, args.world)]
    elif args.command == 'benchmark':
        script, flags = 'run_avoidance_matrix.py', []
    elif args.command == 'transport':
        script, flags = 'run_transport_demo.py', []
    else:
        script, flags = 'avoidance_results_gui.py', []
    os.execv(sys.executable, [sys.executable, str(ROOT / 'scripts' / script), *flags, *extra])


if __name__ == '__main__':
    main()
