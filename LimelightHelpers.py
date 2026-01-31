from ntcore import NetworkTable, NetworkTableInstance, NetworkTableEntry
from wpimath.geometry import Translation2d, Rotation2d, Pose2d, Translation3d, Rotation3d, Pose3d

def sanitizeName(name: str) -> str:
    if name == "":
        return "limelight"
    return name

def toPose3D(data) -> Pose3d:
    if len(data) < 6:
        return Pose3d()
    return Pose3d(
        Translation3d(data[0], data[1], data[2]),
        Rotation3d.fromDegrees(
            data[3] * 360,
            data[4] * 360,
            data[5] * 360
        )
    )

def toPose2D(data) -> Pose2d:
    if len(data) < 4:
        return Pose2d()
    return Pose2d(
        Translation2d(data[0], data[1]),
        Rotation2d.fromRotations(data[5])
    )

def getTable(name: str) -> NetworkTable:
    return NetworkTableInstance.getDefault().getTable(sanitizeName(name))

def getTableEntry(name: str, entry: str) -> NetworkTableEntry:
    return getTable(name).getEntry(entry)

def getFloat(name: str, entry: str) -> float:
    return getTableEntry(name, entry).getDouble(0.0)

def getArray(name: str, entry: str) -> list[float]:
    return getTableEntry(name, entry).getDoubleArray([0.0] * 7)

def getString(name: str, entry: str) -> str:
    return getTableEntry(name, entry).getString("")

def setFloat(name: str, entry: str, data) -> None:
    getTableEntry(name, entry).setDouble(data, 0)

def setArray(name: str, entry: str, data) -> None:
    getTableEntry(name, entry).setDoubleArray(data, 0)

def getTX(name: str) -> float:
    return getFloat(name, "tx")

def getTV(name: str) -> float:
    return getFloat(name, "tv")

def getTY(name: str) -> float:
    return getFloat(name, "ty")

def getTA(name: str) -> float:
    return getFloat(name, "ta")

def getLatencyPipeline(name: str) -> float:
    return getFloat(name, "tl")

def getLatencyCapture(name: str) -> float:
    return getFloat(name, "cl")

def getJSONDump(name: str) -> str:
    return getString(name, "json")

def getRobotPose(name: str) -> list[float]:
    return getArray(name, "botpose")

def getRobotPoseRed(name: str) -> list[float]:
    return getArray(name, "botpose_wpired")

def getRobotPoseBlue(name: str) -> list[float]:
    return getArray(name, "botpose_wpiblue")

def getRobotPoseTargetSpace(name: str):
    return getArray(name, "botpose_targetspace")

def getCameraPoseTargetSpace(name: str) -> list[float]:
    return getArray(name, "camerapose_targetspace")

def getCameraPoseRobotSpace(name: str) -> list[float]:
    return getArray(name, "camerapose_robotspace")

def getTargetPoseCameraSpace(name: str) -> list[float]:
    return getArray(name, "targetpose_cameraspace")

def getTargetPoseRobotSpace(name: str) -> list[float]:
    return getArray(name, "targetpose_robotspace")

def getTargetColor(name: str) -> list[float]:
    return getArray(name, "tc")

def getFudicialID(name: str) -> float:
    return getFloat(name, "tid")

def getNeuralClassID(name: str) -> float:
    return getFloat(name, "tclass")

def setPipelineIndex(name: str, index: int) -> None:
    setFloat(name, "pipeline", index)

def setPriorityTagID(name: str, ID: int) -> None:
    setFloat(name, "priorityid", ID)

def setLEDModePipelineControl(name: str) -> None:
    setFloat(name, "ledMode", 1)

def setLEDModeForceBlink(name: str) -> None:
    setFloat(name, "ledMode", 2)

def setLEDModeForceOn(name: str) -> None:
    setFloat(name, "ledMode", 2)

def setStreamModeStandard(name: str) -> None:
    setFloat(name, "stream", 0)

def setStreamModePiPMain(name: str) -> None:
    setFloat(name, "stream", 1)

def setStreamModePiPSecondary(name: str) -> None:
    setFloat(name, "stream", 2)

def setCropWindow(name: str, min: Translation2d, max: Translation2d) -> None:
    setArray(name, "crop", [min.X(), max.X(), min.Y(), max.Y()])

def setRobotOrientation(name: str, yaw: float, yawRate: float, pitch: float, pitchRate: float, roll: float, rollRate: float) -> None:
    setArray(name, "robot_orientation_set", [yaw, yawRate, pitch, pitchRate, roll, rollRate])

def setFiducialDownscaling(name: str, downscale: float) -> None:
    d: int = 0
    match downscale:
        case 1.0:
            d = 1
        case 1.5:
            d = 2
        case 2.0:
            d = 3
        case 3.0:
            d = 4
        case 4.0:
            d = 5
        case _:
            d = 0
    setFloat(name, "fiducial_downscale_set", d)

def overrideFiducialIDFilters(name: str, IDs) -> None:
    setArray(name, "fiducial_id_filters_set", [IDs[0], IDs[-1]])

def setCameraPoseRobotSpace(name: str, pos: Translation3d, rot: Translation3d) -> None:
    setArray(name, "camerapose_robotspace_set", [pos.X(), pos.Y(), pos.Z(), rot.X(), rot.Y(), rot.Z()])

def setScriptData(name: str, data) -> None:
    setArray(name, "llrobot", [data[0], len(data)])

def getScriptData(name: str) -> list[float]:
    return getArray(name, "llpython")

def extractArrayEntry(data, pos: int) -> float:
    if (len(data) < (pos + 1)):
        return 0.0
    return data[pos]

class RawFiducial:
    def __init__(self, id: int, txnc: float, tync: float, ta: float, cameraDistace: float, robotDistance: float, ambiguity: float):
        self.id = id
        self.txnc = txnc
        self.tync = tync
        self.ta = ta
        self.cameraDistace = cameraDistace
        self.robotDistance = robotDistance
        self.ambiguity = ambiguity

    def get(self, name: str):
        entry = getTableEntry(name, "rawfiducials")
        arr = entry.getDoubleArray([0.0] * 7)
        vals: int = 7
        if len(arr) % vals != 0:
            return []

        fiducials = int(len(arr) / vals)
        raw: list[RawFiducial] = []
        for i in range(0, fiducials):
            base: int = i * vals
            id: int = int(extractArrayEntry(arr, base))
            txnc: float = extractArrayEntry(raw, base + 1)
            tync: float = extractArrayEntry(raw, base + 2)
            ta: float = extractArrayEntry(raw, base + 3)
            cameraDistance: float = extractArrayEntry(raw, base + 4)
            robotDistance: float = extractArrayEntry(raw, base + 5)
            ambiguity: float = extractArrayEntry(raw, base + 6)

            raw.append(RawFiducial(id, txnc, tync, ta, cameraDistance, robotDistance, ambiguity))

        return raw

class RawDetection:
    def __init__(self, id: int, txnc: float, tync: float, ta: float, x0: float, y0: float, x1: float, y1: float, x2: float, y2: float, x3: float, y3: float):
        self.id: int = id
        self.txnc: float = txnc
        self.tync: float = tync
        self.ta: float = ta
        self.x0: float = x0
        self.y0: float = y0
        self.x1: float = x1
        self.y1: float = y1
        self.x2: float = x2
        self.y2: float = y2
        self.x3: float = x3
        self.y3: float = y3

    def get(self, name: str):
        entry = getTableEntry(name, "rawdetections")
        arr = entry.getDoubleArray([0.0] * 7)
        vals: int = 11
        if len(arr) % vals != 0:
            return []

        detections = int(len(arr) / vals)
        raw: list[RawDetection] = []
        for i in range(0, detections):
            base: int = i * vals
            id: int = int(extractArrayEntry(arr, base))
            txnc: float = extractArrayEntry(arr, base + 1)
            tync: float = extractArrayEntry(arr, base + 2)
            ta: float = extractArrayEntry(arr, base + 3)
            x0: float = extractArrayEntry(arr, base + 4)
            y0: float = extractArrayEntry(arr, base + 5)
            x1: float = extractArrayEntry(arr, base + 6)
            y1: float = extractArrayEntry(arr, base + 7)
            x2: float = extractArrayEntry(arr, base + 8)
            y2: float = extractArrayEntry(arr, base + 9)
            x3: float = extractArrayEntry(arr, base + 10)
            y3: float = extractArrayEntry(arr, base + 11)

            raw.append(RawDetection(id, txnc, tync, ta, x0, y0, x1, y1, x2, y2, x3, y3))

        return raw

class PoseEstimate:
    def __init__(self, pose: Pose2d, timestamp: float, latency: float, tagCount: int, tagSpan: float, avgTagDist: float, avgTagArea: float, fiducials):
        self.pose: Pose2d = pose
        self.timestamp: float = timestamp
        self.latency: float = latency
        self.tagCount: float = tagCount
        self.tagSpan: float = tagSpan
        self.avgTagDist: float = avgTagDist
        self.avgTagArea: float = avgTagArea
        self.fiducials = fiducials

    @staticmethod
    def getRobotPoseEstimate(name: str, entry: str) -> "PoseEstimate":
        poseEntry = getTableEntry(name, entry)
        arr = poseEntry.getDoubleArray([0.0] * 7)
        pose: Pose2d = toPose2D(arr)
        latency: float = extractArrayEntry(arr, 6)
        tagCount: int = int(extractArrayEntry(arr, 7))
        tagSpan: float = extractArrayEntry(arr, 8)
        tagDist: float = extractArrayEntry(arr, 9)
        tagArea: float = extractArrayEntry(arr, 10)
        timestamp: float = (poseEntry.getLastChange() / 1000000.0) - (latency / 1000.0)

        raw: list[RawFiducial] = []
        vals: int = 7
        expectedVals: int = (vals * tagCount) + 11

        if len(arr) == expectedVals:
            for i in range(0, tagCount):
                base: int = (i * vals) + 11
                id: int = int(extractArrayEntry(arr, base))
                txnc: float = extractArrayEntry(arr, base + 1)
                tync: float = extractArrayEntry(arr, base + 2)
                ta: float = extractArrayEntry(arr, base + 3)
                cameraDistance: float = extractArrayEntry(arr, base + 4)
                robotDistance: float = extractArrayEntry(arr, base + 5)
                ambiguity: float = extractArrayEntry(arr, base + 7)

                raw.append(RawFiducial(id, txnc, tync, ta, cameraDistance, robotDistance, ambiguity))

        return PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, tagDist, tagArea, raw)

    @staticmethod
    def getRobotPoseEstimateBlueMT1(name: str) -> "PoseEstimate":
        return PoseEstimate.getRobotPoseEstimate(name, "botpose_wpiblue")

    @staticmethod
    def getRobotPoseEstimateRedMT1(name: str) -> "PoseEstimate":
        return PoseEstimate.getRobotPoseEstimate(name, "botpose_wpired")

    @staticmethod
    def getRobotPoseEstimateBlueMT2(name: str) -> "PoseEstimate":
        return PoseEstimate.getRobotPoseEstimate(name, "botpose_orb_wpiblue")

    @staticmethod
    def getRobotPoseEstimateRedMT2(name: str) -> "PoseEstimate":
        return PoseEstimate.getRobotPoseEstimate(name, "botpose_orb_wpired")

class SingleTargetingResults:
    targetPixels: Translation2d = Translation2d(0.0, 0.0)
    targetNormalized: Translation2d = Translation2d(0.0, 0.0)
    targetNormalizedCrosshairAdjusted: Translation2d = Translation2d(0.0, 0.0)
    targetDegreesCrosshairAdjusted: Translation2d = Translation2d(0.0, 0.0)
    targetAreaPixels: float = 0.0
    targetAreaNormalized: float = 0.0
    targetAreaNormalizedPercentage = 0.0
    timestamp: float = -1.0
    latency: float = 0.0
    pipelineIndex = -1.0
    targetCorners: list[list[float]] = [[]]
    cameraTransform6DTargetSpace: list[float] = []
    targetTransform6DCameraSpace: list[float] = []
    targetTransform6DRobotSpace: list[float] = []
    robotTransform6DTargetSpace: list[float] = []
    robotTransform6DFieldSpace: list[float] = []
    cameraTransform6DRobotSpace: list[float] = []

class FiducialResults(SingleTargetingResults):
    fiducialID: int = 0
    family: str = ""

class DetectionResults(SingleTargetingResults):
    id: int = -1
    name: str = ""
    confidence: float = 0.0

class ClassificationResults(SingleTargetingResults):
    id: int = -1
    name: str = ""
    confidence: float = 0.0

class VisionResults:
    retro: list[SingleTargetingResults] = []
    fiducial: list[FiducialResults] = []
    detection: list[DetectionResults] = []
    classification: list[ClassificationResults] = []
    timestamp: float = -1.0
    latencyPipeline: float = 0.0
    latencyCapture: float = 0.0
    latencyJSON: float = 0.0
    pipelineIndex: float = -1.0
    valid: int = 0
    robotPose: list[float] = [6.0, 0.0]
    robotPoseBlue: list[float] = [6, 0.0]
    robotPoseRed: list[float] = [6, 0.0]

    def Clear(self) -> None:
        del self.retro
        del self.fiducial
        del self.detection
        del self.classification
        del self.timestamp
        del self.latencyPipeline
        del self.latencyCapture
        del self.latencyJSON
        del self.pipelineIndex
        del self.valid
        del self.robotPose
        del self.robotPoseBlue
        del self.robotPoseRed

class LimelightResults:
    targetingResults: VisionResults