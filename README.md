# SearchAndRescue

## Movement

- __Left mouse button + mouse movement__ - camera rotation
- __Right mouse button + mouse movement__ - moving camera and camera target up / down / left / right in screen space
- __Mouse wheel__ - zoom in / out

## Loading data

In __File input / output__ select:

- __Trajectory - Read Binary__ and open __trajectory_mat33.bin__ file:

![](docs/1.png)

![](docs/2.png)

- __Object - Read Binary__ and open __stretcher.bin__ file:

![](docs/3.png)

![](docs/4.png)

- __Environment - Read Binary__ and open __cave_normal-001.bin__ file:

![](docs/5.png)

![](docs/6.png)

## View after loading the data:

![](docs/7.png)

## Recommended display settings:

- In __Display__ tab tick / untick items as shown:

![](docs/8.png)

## Traversing trajectory:

- In __Trajectory__ tab use __g_trajectory_index__ slider to move object through the trajectory points and tick __g_modify_current_pose_with_gizmo__ to enable editing pose using guizmo:

![](docs/9.png)

## Viewing collisions:

- When moving object through the trajectory potential collision areas are displayed with red boxes:

![](docs/10.png)