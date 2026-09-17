# Ego model attribution (VM-044)

## M02P.glb — NOT committed to git (see .gitignore)

- Source: `M02P.obj`, the deployed robot's own model (the real M02P proxy
  used by `micropilot_rendering_node`'s `robot_model_path`/
  `robot_model_transform`, `m2o1_params.yaml`).
- Converted by `overlume/scripts/obj2gltf_m02p.py`, run
  via this package's own `scripts/provision_ego_model.sh`.
- Not redistributed: it is the deployed robot's proprietary model, so it is
  excluded regardless of size (~143 MB source `.obj`, ~72 MB converted
  `.glb`). Binary assets that ARE redistributable live in Git LFS — see the
  repository's `.gitattributes`.
- To provision this file on a new box: run `scripts/provision_ego_model.sh
  /path/to/M02P.obj` (or set `M02P_OBJ_SRC`) before building this package.
  Without it, `ego_model_path` resolves empty and the node's existing
  clay-box fallback ships instead — an honest gap, not a build error.
