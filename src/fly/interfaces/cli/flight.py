# connect/status, takeoff, land, RTL

import asyncclick as click

from fly.interfaces.cli.session import get_connected_drone


@click.group(help="Connect to and manually fly the drone (connect, takeoff, land, move).")
def flight():
    pass
