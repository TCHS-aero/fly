import asyncclick as click

from fly.interfaces.cli.session import log_file_option
from fly.logging import logQuery
from fly.logging.flightLog import FlightLog
from fly.utils.geo import Point


async def _load(log_file: str) -> FlightLog:
    log = FlightLog(log_file)
    await log.load_existing()
    return log

def _print_entries(entries: list[dict]) -> None:
    if not entries:
        print("-- No matching entries.")
        return
    for e in entries:
        print(
            f"   [{e['seq']}] {e['filename']} wp={e['wp_index']} "
            f"lat={e['lat']:.6f} lon={e['lon']:.6f} alt={e['alt_rel']}m  ts={e['ts']}"
        )


@click.group(help="Query the flight log (filename <-> GPS/telemetry index.")
def log():
    pass


@log.command(name="list", help="List every entry in the flight log.")
@log_file_option
async def log_list(log_file):
    fl = await _load(log_file)
    _print_entries(fl.all_entries())


@log.command(name="near", help="Entries within a radius (meters) of a lat/lon.")
@log_file_option
@click.option("--lat", type=float, required=True)
@click.option("--lon", type=float, required=True)
@click.option("--radius", type=float, default=25.0, show_default=True, help="Radius in meters.")
async def log_near(log_file, lat, lon, radius):
    fl = await _load(log_file)
    _print_entries(logQuery.entries_near(fl, Point(lat, lon), radius))


@log.command(name="by-waypoint", help="Entries captured at a given waypoint index.")
@log_file_option
@click.option("--wp", type=int, required=True)
async def log_by_waypoint(log_file, wp):
    fl = await _load(log_file)
    _print_entries(logQuery.entries_by_waypoint(fl, wp))


@log.command(name="window", help="Entries captured between two ISO-8601 UTC timestamps.")
@log_file_option
@click.option("--start", required=True, help="ISO-8601 UTC, e.g. 2026-07-27T00:00:00Z")
@click.option("--end", required=True, help="ISO-8601 UTC, e.g. 2026-07-27T23:59:59Z")
async def log_window(log_file, start, end):
    fl = await _load(log_file)
    _print_entries(logQuery.entries_in_window(fl, start, end))


@log.command(name="nearest", help="The single closest entry to a lat/lon")
@log_file_option
@click.option("--lat", type=float, required=True)
@click.option("--lon", type=float, required=True)
async def log_nearest(log_file, lat, lon):
    fl = await _load(log_file)
    entry = logQuery.nearest_entry(fl, Point(lat, lon))
    if entry is None:  # this check is required since nearest_entry can return None instead of an empty list
        print("-- Log is empty.")
        return
    _print_entries([entry])
