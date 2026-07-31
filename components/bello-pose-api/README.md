# bello-pose-api

`bello-pose-api` is the implementation-neutral gRPC contract used to stream an
external display-root pose. `WatchPose` requests reference and tracked frame
names and returns samples using the `reference_from_tracked` convention.

Each sample carries a monotonic producer sequence, translation in metres,
reference-frame linear velocity in metres per second, a normalized quaternion
in `xyzw` order, a source timestamp comparable within the stream, and
producer-reported sample age. The crate exports tonic client/server types and
its encoded file descriptor set.
