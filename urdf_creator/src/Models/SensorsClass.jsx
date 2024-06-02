export class Sensor {
    constructor(sensorType = "", gaussianNoise = 0.01, xyzOffsets = '0 0 0', rpyOffsets = '0 0 0', alwaysOn = true, updateRate = 100) {
        this.sensorType = sensorType;
        this.gaussianNoise = gaussianNoise;
        this.xyzOffsets = xyzOffsets;
        this.rpyOffsets = rpyOffsets;
        this.alwaysOn = alwaysOn;
        this.updateRate = updateRate;
    }

    duplicate() {
        const duplicated = new Sensor();
        Object.assign(duplicated, this);
        return duplicated;
    }
}

export class IMU extends Sensor {
    constructor({ sensorType = 'imu', mean = 0, stddev = 0 } = {}) {
        super(sensorType);
        this.mean = mean;
        this.stddev = stddev;
    }

    duplicate() {
        const duplicated = new IMU();
        Object.assign(duplicated, this);
        return duplicated;
    }
}

export class Camera extends Sensor {
    constructor(sensorType = 'camera', cameraName = 'camera', imageTopicName = '/camera/image_raw', cameraInfoTopicName = '/camera/camera_info', horizontal_fov = 1.3962634, width = 800, height = 600, format = 'R8G8B8', near = 0.1, far = 100) {
        super(sensorType);
        this.cameraName = cameraName;
        this.imageTopicName = imageTopicName;
        this.cameraInfoTopicName = cameraInfoTopicName;
        this.horizontal_fov = horizontal_fov;
        this.width = width;
        this.height = height;
        this.format = format;
        this.near = near;
        this.far = far;
    }

    duplicate() {
        const duplicated = new Camera();
        Object.assign(duplicated, this);
        return duplicated;
    }
}