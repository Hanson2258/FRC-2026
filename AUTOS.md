# Autos

## Available Autos
The first green dot is the starting position, and the red dot is the end position. All the other green dots are waypoints

### Depot-Center
Travels from the center starting position to the depot, picks up all the depot balls, then shoots. <br>
<img width="348" height="173" alt="image" src="https://github.com/user-attachments/assets/0a39e213-73bc-4970-bcda-217d1b09f4b2" />

### Depot-CenterLeft
Travels from the center left starting position to the depot, picks up all the depot balls, then shoots. <br>
<img width="343" height="172" alt="image" src="https://github.com/user-attachments/assets/22220caa-fedc-4641-a244-8b687e8c6d47" />

### Depot-Left
Travels from the left starting position to the depot, picks up all the depot balls, then shoots. <br>
<img width="344" height="167" alt="image" src="https://github.com/user-attachments/assets/38b5b257-1ad5-46b9-ab46-923f4614265d" />

### Outpost-Center
Travels from the center starting position to the outpost to pickup balls, then shoots. <br>
<img width="349" height="166" alt="image" src="https://github.com/user-attachments/assets/9d7b29fb-e681-4e79-9e41-d2f8f97d8eb9" />

### Outpost-CenterRight
Travels from the center right starting position to the outpost to pickup balls, then shoots. <br>
<img width="347" height="170" alt="image" src="https://github.com/user-attachments/assets/d447c792-079e-4a4c-ad2f-99d6d10f3955" />

### Outpost-Right
Travels from the right starting position to the outpost to pickup balls, then shoots. <br>
<img width="347" height="167" alt="image" src="https://github.com/user-attachments/assets/8a428214-f9cf-4667-8ee5-787ed5f71c72" />

### Shooting-Center
Travels from the center starting position then moves back (while rotating 180°), then shoots <br>
<img width="345" height="170" alt="image" src="https://github.com/user-attachments/assets/5e697a79-4ce3-42c2-b7d1-d13aa4916a66" />

### Shooting-CenterLeft
Travels from the center left starting position then moves back (while rotating ~150°), then shoots <br>
<img width="353" height="165" alt="image" src="https://github.com/user-attachments/assets/c837241b-27ab-4f08-bbb9-56dc8f030984" />

### Shooting-CenterRight
Travels from the center right starting position then moves back (while rotating ~150°), then shoots <br>
<img width="349" height="173" alt="image" src="https://github.com/user-attachments/assets/dce1f28b-8360-4797-8b54-708f3313c170" />

### Sweep-Left-Full-Alliance
Starts at the left starting position, them moves to the neutral zone's balls. Then sweeps *all* the balls only on the alliance side. Then tries to shoot, but currently does not have enough time. <br>
<img width="349" height="175" alt="image" src="https://github.com/user-attachments/assets/d3e95844-5e04-4236-b9fc-affce6974e71" />

### Sweep-Left-Full-Center
Starts at the left starting position, them moves to the neutral zone's balls. Then sweeps *all* the balls cutting through the middle. Then tries to shoot, but currently does not have enough time. Very agressive auto. <br>
<img width="349" height="172" alt="image" src="https://github.com/user-attachments/assets/ba3813ca-8183-4fb0-81ff-74ab767248e1" />

### Sweep-Left-Half-Alliance
Starts at the left starting position, them moves to the neutral zone's balls. Then sweeps *half* the balls only on the alliance side. Then tries to shoot, but currently *may* not have enough time. <br>
<img width="353" height="178" alt="image" src="https://github.com/user-attachments/assets/0385c737-852f-44cf-b448-e6c163d2e3db" />

### Sweep-Left-Half-Center
Starts at the left starting position, them moves to the neutral zone's balls. Then sweeps *half* the balls cutting through the middle. Then tries to shoot, but currently *may* not have enough time. Agressive auto. <br>
<img width="351" height="173" alt="image" src="https://github.com/user-attachments/assets/6301acd1-0056-4b55-bb05-34ce5d350c11" />

### Sweep-Right-Full-Alliance
Starts at the right starting position, them moves to the neutral zone's balls. Then sweeps *all* the balls only on the alliance side. Then tries to shoot, but currently does not have enough time. <br>
<img width="352" height="173" alt="image" src="https://github.com/user-attachments/assets/e740fa54-8c3f-4124-b801-432c6aed56d4" />

### Sweep-Right-Full-Center
Starts at the right starting position, them moves to the neutral zone's balls. Then sweeps *all* the balls cutting through the middle. Then tries to shoot, but currently does not have enough time. Very agressive auto. <br>
<img width="350" height="167" alt="image" src="https://github.com/user-attachments/assets/9987baa2-1743-4199-9ab7-7f032116680f" />

### Sweep-Right-Half-Alliance
Starts at the right starting position, them moves to the neutral zone's balls. Then sweeps *half* the balls only on the alliance side. Then tries to shoot, but currently *may* not have enough time. <br>
<img width="353" height="176" alt="image" src="https://github.com/user-attachments/assets/9c306724-3541-4e94-8ee8-b64bc368fdb4" />

### Sweep-Right-Half-Center
Starts at the right starting position, them moves to the neutral zone's balls. Then sweeps *half* the balls cutting through the middle. Then tries to shoot, but currently *may* not have enough time. Agressive auto. <br>
<img width="354" height="175" alt="image" src="https://github.com/user-attachments/assets/cf6905c0-b278-4311-b00c-d6cafd6ad787" />

### Hang-Center `WIP`
DO NOT USE, WIP

### Hang-Center-Shoot `WIP`
DO NOT USE, WIP

## Making Autos
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
