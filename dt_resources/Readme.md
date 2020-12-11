# dt_resources

A package that group all the director task's resources

## Dependencies

 - **Ontologenius** : *Execution*
 - **Mementar**     : *Execution*
 - **ar_track_alvar** : *Execution*

# Install

```
git clone -b proba https://github.com/sarthou/ontologenius.git
git clone -b tempo https://github.com/sarthou/mementar.git
git clone -b kinetic-devel https://github.com/sarthou/ar_track_alvar.git
```

# Usage

```
roslaunch dt_resources ontologenius_mementar.launch
roslaunch dt_resources ar_track_alvar_robot.launch
```

OR

```
roslaunch dt_resources ontologenius_mementar_multi.launch
roslaunch dt_resources ar_track_alvar_robot.launch
```
