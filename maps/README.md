# Saved maps

`mapping_fixed/` is the latest user-verified environment scan. It was moved
byte-for-byte from `results/mapping_fixed/` during repository cleanup.

- `map.pgm`: occupancy image.
- `map.yaml`: image reference, resolution, origin, and occupancy thresholds.
- `map_data.npz`: mapping evidence needed to resume and extend this scan.

Keep all three files together. Goal selection and navigation default to this
map. New scans and navigation output go under ignored `results/` unless explicitly
promoted into this folder. The mapper's simulator coordinate frame must match
the scene used for navigation.
