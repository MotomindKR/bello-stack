# Dual Milly 7-DOF with Linker O6 hands

Two seven-axis arms carry rigid O6 hands and share a central D405 camera. Use `left_palm_eef` and `right_palm_eef` as tool frames.

![Dual Milly 7-DOF with Linker O6 hands exterior from two angles](images/dual-o6-preview.png)

[URDF](urdf/milly-7dof-dual-o6.urdf) · [Hardware inventory](hardware.yaml)

**Collision geometry and joint axes**

![Dual Milly 7-DOF with Linker O6 hands collision shapes and movable joint axes from the front and rear](images/milly-7dof-dual-o6-collision.png)

Each palm has one collision box. The fingers are fixed in the URDF. The [hand inventory](hands.yaml) and [Linux driver](../../components/bello-hand/README.md) describe finger control.

## Palm detail

![Left and right palm collision boxes over the O6 hand meshes](images/palm-collision.png)

In the review pose, J2 and J4 are at 1.15 rad on both arms. The palms face inward and the thumbs point up. The wrist-to-palm offsets are nominal.

[Linkerbot mesh source](https://github.com/linker-bot/linkerhand-urdf/tree/49f26021b0a91ff5c33150df22970c07bb548672/O6) · [Mesh license](meshes/LICENSE-Linkerbot)
