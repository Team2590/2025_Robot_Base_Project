// In your robot initialization code
List<CameraConfig> cameras = List.of(
    new CameraConfig("Camera1", new Transform3d(...)), // Front camera
    new CameraConfig("Camera2", new Transform3d(...)), // Back camera
    new CameraConfig("Camera3", new Transform3d(...))  // Side camera
);
VisionIOPhotonVision vision = new VisionIOPhotonVision(cameras);