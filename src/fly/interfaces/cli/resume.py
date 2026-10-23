import asyncclick as click

from fly.core.resume_manager import ResumeManager
from fly.interfaces.cli.session import state_file_option


@click.group(help="Inspect or clear locally-saved flight-resume state.")
def resume():
    pass


@resume.command(name="status", help="Show whether a resumable (interrupted) flight is on record.")
@state_file_option
def resume_status(state_file):
    rm = ResumeManager(state_file=state_file)
    found = rm.load()
    if not found:
        print(f"-- No resumable flight on record in {state_file}.")
        return
    print(f"-- Resumable flight found in {state_file}.")
    print(f"   last_waypoint  : {rm.last_waypoint}")
    print(f"   survey_complete: {rm.survey_complete}")
    print(f"   resume_index   : {rm.resume_index()} (pass to `mission edit`/`set_current_mission_target`)")


@resume.command(name="clear", help="Clear saved resume state (e.g. after a manual recovery).")
@state_file_option
@click.option("--yes", is_flag=True, default=False, help="Skip the confirmation prompt.")
def resume_clear(state_file, yes):
    rm = ResumeManager(state_file=state_file)
    if not rm.path.exists():
        print(f"-- {state_file} does not exist; nothing to clear.")
        return

    if not yes:
        ans = input(f"Clear resume state in {state_file}? This is irreversible. y/n: ").strip().lower()
        if ans != "y":
            print("-- Cancelled.")
            return

    rm.path.unlink()
    print(f"-- Cleared {state_file}.")
