# Sortify Task Queue

Background tasks for parallel work while the main chat focuses on
hands-on demo tuning. Each task file is self-contained with a system
prompt the subagent can read straight, the spec, the affected files,
and the Done When checklist.

## Active

| File | Title | Status |
|------|-------|--------|
| [web_dashboard.md](web_dashboard.md) | Localhost web dashboard with MJPEG stream, live tune, command box, macros | Done |
| [dropped_cube_filter_fix.md](dropped_cube_filter_fix.md) | Bridge re-targets just-dropped cube on next SEEKING_BLOCK | Done |

## How to use

Subagent reads its assigned `.md` file, follows the system prompt and
the spec, edits the files listed in "Files Affected", verifies the
Done When checklist, and updates the Status checkbox before exiting.

Don't put implementation chatter in this file — chatter goes in the
main chat; the task files are deliverable specs.
