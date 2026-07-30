# Version 1.2 — Import Hell

## The Panic

We copied our code to the Raspberry Pi (fresh install, nothing installed).
We ran `python pi/main.py`. Error.

```
ImportError: No module named 'sensors'
```

Wait what? The sensors folder is right there. `pi/sensors/` exists. We can
see it. Why can't Python see it?

## Why This Happened (The Real Explanation)

On my laptop, I was running the code from inside the `pi/` folder:
```
~/project/pi/ $ python main.py
```

In this case, Python adds the current directory (`pi/`) to the module
search path. So when the code does `from sensors.camera import PiCamera`,
Python finds `pi/sensors/camera/`. It works.

On the Pi, we ran from the project root:
```
~/wro/ $ python pi/main.py
```

Now the current directory is `~/wro/`, not `~/wro/pi/`. Python looks for
`sensors/` in `~/wro/sensors/` which does not exist. Sensors is at
`~/wro/pi/sensors/`.

## The Fix (Two Parts)

**Fix 1:** Add the project root to the Python path at the top of main.py:
```python
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
```

This is like telling Python: "Hey, if you cannot find a module, also look
in the parent folder of pi/."

**Fix 2:** Use full package paths everywhere:
```python
from pi.sensors.camera import PiCamera
from pi.fusion.ukf import RobotUKF
```

Instead of:
```python
from sensors.camera import PiCamera
```

## Why I Did Not Catch This Earlier

Because I was lazy. I always ran the code from inside the `pi/` directory
during development. The Pi runs it from outside. Different environments
reveal different bugs.

This is why professionals use Docker or virtual environments. They run
their code in the same environment as production. We did not do that.
We paid the price with 3 hours of debugging.

## Other Changes in This Version

While fixing imports, I also fixed all relative imports across the project.
Every file in `pi/` had to be checked.

The pattern is:
- Files inside `pi/` use `from pi.subsystem.module import Class`
- This works whether you run from `pi/` or from the project root

## What I Wish Someone Told Me

Python's import system is not magic. It is just `sys.path` + file lookup.
If you understand `sys.path`, you understand imports. Draw a picture:
- `sys.path` = list of folders to search
- When you `import foo`, Python checks each folder for `foo.py` or `foo/__init__.py`
- The current working directory is ALWAYS the first entry (if running from shell)

## Evidence

There are no special marks in the code for this fix because it is just
import lines. But the proof is: the code runs on both my laptop and the Pi.
Before this version, it only ran on my laptop. That is the difference
between "works on my machine" and "works on the robot."
