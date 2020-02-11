# SquidCuber

This is SquidCuber, the (as of February 2020) fastest Lego-based Rubik's Cube solver in the world with an average time of about 1 second flat.
Achieving this (almost unthinkably) low time with the rather slow official Lego motors took incredible amounts of tuning and various general advancements in cube robot optimization (some already introduced during the development of this robot's predecessor, [`mirrcub3r`](https://github.com/efrantar/mirrcub3r)).
Most likely, this is now the actual limit of what is doable with current Lego hardware. Below GIF shows the best time I have gotten in a single solve, a detailed showcase of SquidCuber can be found here: https://youtu.be/wLzn1w8vgM4.

**TODO: Add GIF.**

I started rewriting [`rob-twophase`](https://github.com/efrantar/rob-twophase) (the solving algorithm powering the machine at hand) some time at the end of last December, mostly to clean up the code but also to speed it up even more and include a few additional features.
Actual work on this robot only started in the beginning of January when I was messing around with Lego Technic to come up with a more stable (and elegant) construction.
About three weeks later the model was standing and programming could start.
Although SquidCuber of course strongly draws from the knowledge gained by working on prior robots for many months, there have been countless, seemingly subtle yet actually crucially important improvements.
If any single one of them was missing, the robot would not be able to get the times it does.
Furthermore, I had to completely redesign the color recognition algorithm (as the extremely strong reflections completely broke all my previous approaches).
While it is now infinitely more complex (featuring full constraint propagation as well as a learning component), it does not just work much better but is also over 5 times faster.
Overall, SquidCuber is a big step up over it's predecessor.

## Details

SquidCuber consists of 12 motors joined pairwise (with up-gearing) to form much faster and stronger double-motors.
Each of the 6 motor pairs is directly connected to one of the centers of the cube (to make that possible Lego bricks have been glued on the center caps, similarly to the modifactions done by most recent official Guinness World Record robots).
2 parallel sides are powered by 1 of 3 Lego Mindstorm controllers each.
The general Lego construction built form ~1300 Technic bricks is designed to ensure maximum stability as well as precise centering of the cube (with respect to the motor connections), permitting much more aggressive turning without any loss of consistency.
2 web-cams are precisely positioned in such a way that the can see 3 sides of the cube respectively, allowing the robot to scan the full starting state in a single "look".
As the cameras have really steep viewing angles onto the cube (and the cube's surface texture is not helping either), there are often very strong reflections on the plastic making correct scanning extremely challenging (requiring a highly advanced custom color matching algorithm).
Almost the entire control happens on the PC running Python scripts.
It communicates with the Mindstorm controllers (as well as the cameras) over USB via so called "Direct Command" (using Christoph Gaukel's fantastic `ev3-python3` API), i.e. bit-pattern as accepted by the internal Mindstorm firmware.
This admmittedly rather inconvenient manner of programming is surprisingly critical to ensure minimum latency and utilization of the best motor drivers (the official Lego ones) giving signifcant speed-ups (in terms of actual solving performance) over any programming frameworks running in extra VMs.
Solutions are computed using version 2.0 of my very own [`rob-twophase`](https://github.com/efrantar/rob-twophase) algorithm, an extremely efficient implementation of Herbert Kociemba's two-phase algorithm (written in C++) specifically designed for the use by ultra high-speed robots.
It is not just the fastest solver currently available online, but the only one that has the robot mechanics (i.e. that parallel faces can be turned at the same time and that 180 degree moves take twice as long as 90 degree ones) deeply built in and thus finds up to 20% faster solutions to execute on average.

The following is a list of (in my opinion) the most interesting features/optimizations employed by SquidCuber and where to find them:

* A highly robust color matching algorithm (`scan/`) able to consistently handle the very challenging scan conditions of this robot based on machine learning (`scan/train.py`) and full constraint propagation (`scan/match.cpp`).
* Control of a non-trivial robot with Lego Direct Commands, i.e. bit-patterns as accepted by the internal Mindstorm engine. (`cmd.py`)
* A movement system that heavily utilizes the corner cutting capabilities of a modern speed-cube by differentiating more than 20 different turn transition cases with meticulous individual tuning (`control.py`).
* Determining optimal directions of 180 degree turns (which can always be executed either clockwise or counter-clockwise) with respect to the corner cutting (`control.py`).
* Rating the multiple solutions return by the solver according to timing data collected during previous solves to select the overall fastest solution (`control.py`).

Please note that the code here focusses exclusively on the essentials, i.e. non-trivial techniques for reaching maximum speed while maintaining solid consistency, and thus does not include a pretty graphical user interface or a highly abstracted API.

## Disclaimer

This repository is primarily intended as a resource for people looking to learn some advanced tricks for speeding up their own solvers (and of course also people interested in the techniques underlying high-speed cube solvers in general).
It is **not** meant to provide software that can be run as is, hence also no building instructions for the hardware are included (there are other robot designs developed specifically for being rebuilt).

I hope my work will inspire the development of many new cube robots! :)
