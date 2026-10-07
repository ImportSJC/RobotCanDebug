# CAN debug logs

This folder holds CAN dropout diagnostic logs copied off the roboRIO. It is deliberately **not** git-ignored,
so you can commit the logs and push them for review.

To pull the logs, connect to the robot and run this from the repo root (or run the VS Code task
`PullCanDiagLogs`):

```
python tools/can-debug/pull_can_logs.py            # candiag_*.txt only (small)
python tools/can-debug/pull_can_logs.py --wpilog 1 # plus the newest .wpilog
```

Then commit and push them:

```
git add can-debug-logs && git commit -m "CAN debug logs" && git push
```

See `docs/can-dropout-diagnostics.md` for what the logs contain and the test procedure.
