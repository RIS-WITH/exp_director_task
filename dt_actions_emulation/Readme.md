# dt_actions_emulation

A package to emulate actions' detection and knowledge update.

## Dependencies

 - **Ontologenius** : *Compilation & Execution*
 - **Mementar**     : *Execution*

# Install

```
git clone -b proba https://github.com/sarthou/ontologenius.git
git clone -b tempo https://github.com/sarthou/mementar.git
```

# Usage

```
roslaunch dt_resources ontologenius_mementar.launch
rosrun dt_actions_emulation actions_emulation -i path/to/actions.xml
```

OR

```
roslaunch dt_resources ontologenius_mementar_multi.launch
rosrun dt_actions_emulation actions_emulation -i path/to/actions.xml -n instance_name
```

> An action sequence example is available in test_files/actions.xml
