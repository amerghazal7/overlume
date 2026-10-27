# Ego model attribution (VM-044)

## ego.glb — NOT committed to git (see .gitignore)

No ego model ships with this repository. `assets/ego/ego.glb` is a provisioned
local asset: the node installs it as its default `ego_model_path` when present
and falls back to the clay-box ego otherwise (an honest gap, not a build error).

- A provisioned model stays local: do not commit or redistribute it unless its
  licence allows redistribution. Binary assets that ARE redistributable live in
  Git LFS — see the repository's `.gitattributes`.
- To provision one: `scripts/provision_ego_model.sh /path/to/model.glb` (copies it
  to `assets/ego/ego.glb`), then re-run this package's cmake configure. An OBJ
  source (Y up, meters) is converted first with
  `overlume/scripts/obj2gltf.py robot.obj model.glb`.
