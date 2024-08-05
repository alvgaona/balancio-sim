# Balancio Sim

<div align="center">
  <img alt="Balancio Animation" src="./resources/balancio.gif">
</div>

This is the implementation of a self-balancing robot called [Balancio]
in NVIDIA Isaac Sim.

The robot is an educational robot which can be built by any student with
standard hardward, firmware and software.
It's extremely affordable and it opens the door to many applications with
a self-balancing robot.

## Pixi

The very first thing you'll need is a package manager called Pixi, it's fast and easy to use.
The reason behind using Pixi is that you can get ROS2 working super quick, and it can even run ROS2
in mostly any OS, e.g., Windows, Linux or macOS.
Follow this [link][pixi-installation] for installation instructions.

Once you have it, make sure you're in the repo folder in your machine, and run the Pixi install command.
Wait a couple of minutes until everything is fetched.

```text
pixi install
```

If you want to test this, you can run the following ROS2 command.

```text
pixi run ros2 topic list
```

The output should look like this:

```text
/parameter_events
/rosout
```

## Isaac Sim

It's a prerequisite you have Isaac Sim running on your machine without errors.
You can get it using the Omniverse Launcher that you can download from [here][omniverse-launcher];
you'll need an NVIDIA account.
This example shall work either on Windows or Linux.

### Assets

If you'd like to just import the robot itself in any other world you may
have created, just download the [USD file for Balancio][balancio-usd] only.

Now if you want the world we've created, where you can find both an NVIDIA Jetbot,
and the Balancio ready to send command via ROS2, you should download [this USD file][balancio-sim].
Then, you can open it directly in Isaac Sim.

### Running Isaac Sim

You can spin up Isaac Sim through the Ominverse Launcher UI but ROS2 won't work directly like that,
so there's a script in the `scripts/` folder named `isaac_sim_run.ps1`
or `isaac_sim_run.sh`, depending if you're on Windows or Linux, respectively.

You need to run either script from within the Pixi environment, so first run:

```text
pixi shell
```

And then just do either:

**Windows Powershell**

```text
. .\scripts\isaac_sim_run.ps1
```

**Linux Shell**

```text
./scripts/isaac_sim_run.sh
```

This will open up Isaac Sim with ROS2 ready to be used.

## ROS2

If you went through the Pixi section, you should have ROS2 on your machine.
So you can run the `pid` node in the repo to send the commands to Isaac Sim, and
control Balancio.

```text
pixi run ros2 run balancio pid --ros-args \
-p kp:=2.3 \
-p ki:=2.0 \
-p kd:=0.05 \
-p set_point:=-0.017 \
-p sum_constraint:=1.0
```

And in another shell, you can subscribe to the `/cmd_vel` topic and check that you're actually getting
the commmand.

```text
pixi run ros2 topic echo /cmd_vel
```

[Balancio]: https://github.com/udesa-ai/balancio-kit/tree/main
[balancio-usd]: https://storage.googleapis.com/nvidia-omniverse-content/Isaac/Robots/balancio.usd
[balancio-sim]: https://storage.googleapis.com/nvidia-omniverse-content/Isaac/Environments/balancio_sim.usd
[omniverse-launcher]: https://www.nvidia.com/en-us/omniverse/download/
[pixi-installation]: https://pixi.sh/latest/#installation
