import json
import os

import aiofiles
import asyncclick as click

from fly.interfaces.cli.mission import waypoint_from_kwargs, waypoint_options
from fly.poi.poiManager import POIManager

_STATUS_CHOICES = click.Choice(["candidate", "queued", "delivered", "dismissed"])


async def _load_registry(registry: str) -> POIManager:
    pois = POIManager(registry_path=registry)
    await pois.load()
    return pois

def _require_poi(pois: POIManager, poi_id: int) -> dict:
    p = pois.get(poi_id)
    if not p:
        print(f"-- No POI with id {poi_id}.")
        raise SystemExit(1)
    return p

@click.group(help="Inspect the POI registry and generate approach waypoints.")
def poi():
    pass


@poi.command(name="list", help="List POIs in the registry.")
@click.option("--registry", default="poi_registry.json", show_default=True)
@click.option("--status", default=None, type=_STATUS_CHOICES)
async def poi_list(registry, status):
    pois = await _load_registry(registry)
    entries = pois.get_all(status=status)
    if not entries:
        print("-- No POIs on record.")
    for p in entries:
        print(
            f"   #{p['poi_id']} [{p['status']}] lat={p['lat']:.6f} lon={p['lon']:.6f} "
            f"conf={p['confidence_avg']:.2f} detections={p['detection_count']}"
        )


@poi.command(name="show", help="Show full detail for one POI.")
@click.option("--registry", default="poi_registry.json", show_default=True)
@click.option("--id", "poi_id", type=int, required=True)
async def poi_show(registry, poi_id):
    pois = await _load_registry(registry)
    p = _require_poi(pois, poi_id)
    print(json.dumps(p, indent=2))


@poi.command(name="status", help="Update a POI's lifecycle status.")
@click.option("--registry", default="poi_registry.json", show_default=True)
@click.option("--id", "poi_id", type=int, required=True)
@click.option("--set", "new_status", required=True, type=_STATUS_CHOICES)
async def poi_status(registry, poi_id, new_status):
    pois = await _load_registry(registry)
    try:
        await pois.update_status(poi_id, new_status)
    except KeyError as e:
        print(f"-- {e}")
        raise SystemExit(1) from e
    print(f"-- POI #{poi_id} -> {new_status}")


@poi.command(name="approach", help="Generate approach + delivery waypoints for a POI.")
@click.option("--registry", default="poi_registry.json", show_default=True)
@click.option("--id", "poi_id", type=int, required=True)
@click.option("--out", "out_file", default=None, help="Write the waypoints as a mission_waypoints.json-style file instead of printing.")
@click.option("--rtl/--no-rtl", default=True, show_default=True, help="RTL flag to embed if --out is used.")
@waypoint_options
async def poi_approach(registry, poi_id, out_file, rtl, **kwargs):
    pois = await _load_registry(registry)
    p = _require_poi(pois, poi_id)

    kwargs["lat"] = p["lat"]
    kwargs["lon"] = p["lon"]
    kwargs["is_fly_through"] = False
    wps = waypoint_from_kwargs(kwargs)

    if out_file:
        if os.path.exists(out_file):
            async with aiofiles.open(out_file, "r") as f:
                existing = json.loads(await f.read())
            existing.extend(wps)
            async with aiofiles.open(out_file, "w") as f:
                await f.write(json.dumps(existing, indent=2))
            total = len(existing) - 1  # subtract RTL
            print(f"-- Appended {len(wps)} waypoint(s) to {out_file} ({total} total).")
        else:
            async with aiofiles.open(out_file, "w") as f:
                await f.write(json.dumps([rtl, *wps], indent=2))
            print(f"-- Wrote {len(wps)} waypoint(s) to {out_file}.")
    else:
        print(json.dumps(wps, indent=2))
