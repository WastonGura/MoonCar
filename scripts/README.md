# Scripts

## M4 GUI acceptance

Run the complete build, Gazebo/RViz launch, drive check, and mapping check from
the Ubuntu VM with one command:

```bash
~/MoonCar/scripts/accept_m4.sh
```

After the checks pass, inspect the GUI and press Enter in the terminal to stop
the launched processes. The default map is saved as
`~/mooncar_m4_acceptance.yaml` and `~/mooncar_m4_acceptance.pgm`.

Pass a different map prefix as the only argument when needed:

```bash
~/MoonCar/scripts/accept_m4.sh "$HOME/maps/test_run"
```
