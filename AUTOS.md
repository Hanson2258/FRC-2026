# Autos
We use FRC PathPlanner to create our autos. To use PathPlanner reference the [documentation](pathplanner.dev/home.html). <br>
Some import (semi-advanced) aspects to know are: <br>
1. [Linked Waypoints](https://pathplanner.dev/gui-editing-paths-and-autos.html#c5imgi_32)
2. [Path Optimization](https://pathplanner.dev/gui-editing-paths-and-autos.html#path-optimization)
3. Adding Named Commands (Below)

### Adding Named Commands
In the RobotContainer's constructor, add a line of code like this:
```
NamedCommands.registerCommand("Intake On", Commands.runOnce(() -> intake.setIntakingMode(), intake));
```
This registers a named command that can be used in autos. To add the named command in PathPlanner you can create a new `Named Command` in an auto, then type in the name of the key
(In this example it would be `"Intake On"`)

## Naming Conventions
To provide a much more consistant auto making process, we use the below naming conventions:

### Paths

`[common phrase if any]-[path-type]-[starting position if needed]-[extra info]` <br>
`Ex. GoTo-InnerLadder-From-Outpost`

Common Phrases:
1. `GoTo`
2. `From`
3. `StartingPosition`

### Linked Waypoints

Normal linked waypoints: <br>
`[path name]--[S or E for "Start" and "End", If not start or end, use W1 ("Waypoint 1"), if more linked waypoints are added, increase the end number ("W2", "W3", etc.)]` <br>
`Ex. "GoTo-Outpost-Center--S"`

Commonly Used Position with Variations: (Ex. Starting Position) <br>
`[name]-[position]` <br>
`Ex. "StartingPosition-Center"`

### Autos

`[auto type]-[starting position]-[extra info]` <br>
`Ex. "Sweep-Left-Half-Center"`

Ensure autos are consistant in naming because they should be easy to pickout from a large list.
