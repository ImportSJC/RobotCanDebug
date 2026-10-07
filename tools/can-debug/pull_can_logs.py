"""Copy CAN dropout diagnostic logs from the roboRIO into can-debug-logs/ (committed, not git-ignored).

Usage (from the repo root, laptop on the robot network):
    python tools/can-debug/pull_can_logs.py                # all candiag .txt files
    python tools/can-debug/pull_can_logs.py --wpilog 1     # plus the newest AdvantageKit .wpilog
    python tools/can-debug/pull_can_logs.py --host 172.22.11.2   # over USB

The roboRIO's lvuser has an empty password: if scp asks for one, just press Enter.
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DEST = REPO / "can-debug-logs"
RIO_DIAG = "/home/lvuser/logs/candiag"
RIO_LOGS = "/home/lvuser/logs"


def ssh_opts():
    # Re-imaged roboRIOs change host keys; don't let a stale known_hosts entry block the copy.
    return ["-o", "StrictHostKeyChecking=no", "-o", f"UserKnownHostsFile={os.devnull}", "-o", "ConnectTimeout=5"]


def run(cmd):
    print("> " + " ".join(cmd))
    return subprocess.run(cmd)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="10.14.5.2", help="roboRIO address (default 10.14.5.2; USB is 172.22.11.2)")
    p.add_argument("--wpilog", type=int, default=0, metavar="N", help="also copy the newest N .wpilog files")
    args = p.parse_args()

    DEST.mkdir(exist_ok=True)
    target = f"lvuser@{args.host}"

    if run(["scp", *ssh_opts(), f"{target}:{RIO_DIAG}/*.txt", str(DEST)]).returncode != 0:
        sys.exit("scp failed: is the laptop connected to the robot and has the diagnostics code run at least once?")

    if args.wpilog > 0:
        listing = subprocess.run(["ssh", *ssh_opts(), target, f"ls -t {RIO_LOGS}/*.wpilog | head -n {args.wpilog}"],
                                 capture_output=True, text=True)
        names = [n for n in listing.stdout.split() if n.endswith(".wpilog")]
        if not names:
            print("no .wpilog files found on the roboRIO")
        for name in names:
            run(["scp", *ssh_opts(), f"{target}:{name}", str(DEST)])
            size = (DEST / Path(name).name).stat().st_size / 1e6
            if size > 90:
                print(f"WARNING: {Path(name).name} is {size:.0f} MB; GitHub rejects files over 100 MB. "
                      "Trim it with the WPILog Janitor before committing.")

    print(f"\nCopied into {DEST}. Commit them with:\n  git add can-debug-logs && git commit -m \"CAN debug logs\" && git push")


if __name__ == "__main__":
    main()
