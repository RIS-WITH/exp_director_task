# dt_resources

A package that group all the director task's resources

## Dependencies

 - **Ontologenius** : *Execution*
 - **Mementar**     : *Execution*
 - **ar_track_alvar** : *Execution*
 - **common_ground_ontology** : *Execution*
 - **overworld** : *Execution*

# Install

```
git clone -b proba https://github.com/sarthou/ontologenius.git
git clone -b tempo https://github.com/sarthou/mementar.git
git clone -b kinetic-devel https://github.com/sarthou/ar_track_alvar.git
git clone git@github.com:RIS-WITH/common_ground_ontology.git
git clone -dev git@github.com:sarthou/overworld.git
```

# Usage

Pr2

```
roslaunch dt_resources ontologenius_mementar_multi.launch
roslaunch dt_resources ar_track_alvar_robot.launch
roslaunch dt_resources overworld.launch
roslaunch dt_resources static_transform_pr2.launch
```

Pepper

```
roslaunch dt_resources ontologenius_mementar_multi.launch
roslaunch dt_resources ar_track_alvar_pepper.launch
roslaunch dt_resources overworld_pepper.launch
roslaunch dt_resources static_transform_pepper.launch
```
