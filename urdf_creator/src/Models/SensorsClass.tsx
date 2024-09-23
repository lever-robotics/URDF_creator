export class Sensor {
    type: string;
    constructor(type = ""){
        this.type = type;
    }

    update(name: string, value: number | string){
        (this as any)[name] = value; //hotfix for Typescript, code as all possible values in future
    }

    duplicate() {
        return new Sensor();
    }
}

export function sensorCreator(sensor = {type: ""}) {
    let newSensor;
    switch (sensor.type) {
        case "imu":
            newSensor = new IMU(sensor);
            break;
        case "camera":
            newSensor = new Camera(sensor);
            break;
        case "lidar":
            newSensor = new Lidar(sensor);
            break;
        default:
            newSensor = new Sensor(sensor.type);
            break;
    }
    return newSensor;
}


export class IMU extends Sensor{
    gaussianNoise: number;
    xyzOffsets: string;
    rpyOffsets: string;
    alwaysOn: boolean;
    updateRate: number;
    mean: number;
    stddev: number;
    constructor({
        type = "imu",
        gaussianNoise = 0.01,
        xyzOffsets = "0 0 0",
        rpyOffsets = "0 0 0",
        alwaysOn = true,
        updateRate = 100,
        mean = 0,
        stddev = 0,
    } = {}) {
        super(type);
        this.gaussianNoise = gaussianNoise;
        this.xyzOffsets = xyzOffsets;
        this.rpyOffsets = rpyOffsets;
        this.alwaysOn = alwaysOn;
        this.updateRate = updateRate;
        this.mean = mean;
        this.stddev = stddev;
    }

    clone() {
        const clone = new IMU();
        Object.assign(clone, this);
        return clone;
    }
}

export class Camera extends Sensor{
    gaussianNoise: number;
    xyzOffsets: string;
    rpyOffsets: string;
    alwaysOn: boolean;
    updateRate: number;
    cameraName: string;
    imageTopicName: string;
    cameraInfoTopicName: string;
    horizontal_fov: number;
    width: number;
    height: number;
    format: string;
    near: number;
    far: number;
    mean: number;
    stddev: number;
    constructor({
        type = "camera",
        gaussianNoise = 0.01,
        xyzOffsets = "0 0 0",
        rpyOffsets = "0 0 0",
        alwaysOn = true,
        updateRate = 100,
        cameraName = "camera",
        imageTopicName = "/camera/image_raw",
        cameraInfoTopicName = "/camera/camera_info",
        horizontal_fov = 1.3962634,
        width = 800,
        height = 600,
        format = "R8G8B8",
        near = 0.1,
        far = 100,
        mean = 0.0,
        stddev = 0.007,
    } = {}) {
        super(type);
        this.gaussianNoise = gaussianNoise;
        this.xyzOffsets = xyzOffsets;
        this.rpyOffsets = rpyOffsets;
        this.alwaysOn = alwaysOn;
        this.updateRate = updateRate;
        this.cameraName = cameraName;
        this.imageTopicName = imageTopicName;
        this.cameraInfoTopicName = cameraInfoTopicName;
        this.horizontal_fov = horizontal_fov;
        this.width = width;
        this.height = height;
        this.format = format;
        this.near = near;
        this.far = far;
        this.mean = mean;
        this.stddev = stddev;
    }

    clone() {
        const clone = new Camera();
        Object.assign(clone, this);
        return clone;
    }
}

export class Lidar extends Sensor{
    alwaysOn: boolean;
    updateRate: number;
    pose: string;
    samples: number;
    resolution: number;
    minAngle: number;
    maxAngle: number;
    minRange: number;
    maxRange: number;
    rangeResolution: number;
    mean: number;
    stddev: number;
    
    constructor({
        type = "lidar",
        alwaysOn = true,
        updateRate = 5,
        pose = "-0.064 0 0.121 0 0 0",
        samples = 360,
        resolution = 1.0,
        minAngle = 0.0,
        maxAngle = 6.28,
        minRange = 0.12,
        maxRange = 3.5,
        rangeResolution = 0.015,
        mean = 0.0,
        stddev = 0.01,
    } = {}) {
        super(type);
        this.alwaysOn = alwaysOn;
        this.updateRate = updateRate;
        this.pose = pose;
        this.samples = samples;
        this.resolution = resolution;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.minRange = minRange;
        this.maxRange = maxRange;
        this.rangeResolution = rangeResolution;
        this.mean = mean;
        this.stddev = stddev;
    }

    clone() {
        const clone = new Lidar();
        Object.assign(clone, this);
        return clone;
    }
}