export class IMU {
    constructor({ sensorType = 'imu', gaussianNoise = 0.01, xyzOffsets = '0 0 0', rpyOffsets = '0 0 0', alwaysOn = true, updateRate = 100, mean = 0, stddev = 0 } = {}) {
        this.sensorType = sensorType;
        this.gaussianNoise = gaussianNoise;
        this.xyzOffsets = xyzOffsets;
        this.rpyOffsets = rpyOffsets;
        this.alwaysOn = alwaysOn;
        this.updateRate = updateRate;
        this.mean = mean;
        this.stddev = stddev;
    }

    duplicate() {
        const duplicated = new IMU();
        Object.assign(duplicated, this);
        return duplicated;
    }
}

export class Camera {
    constructor({ sensorType = 'camera', gaussianNoise = 0.01, xyzOffsets = '0 0 0', rpyOffsets = '0 0 0', alwaysOn = true, updateRate = 100, cameraName = 'camera', imageTopicName = '/camera/image_raw', cameraInfoTopicName = '/camera/camera_info', horizontal_fov = 1.3962634, width = 800, height = 600, format = 'R8G8B8', near = 0.1, far = 100, mean=0.0, stddev=0.007 } = {}) {
        this.sensorType = sensorType;
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

    duplicate() {
        const duplicated = new Camera();
        Object.assign(duplicated, this);
        return duplicated;
    }
}

export class Lidar {
    constructor({ 
        sensorType = 'lidar', 
        alwaysOn = true, 
        updateRate = 5, 
        pose = '-0.064 0 0.121 0 0 0', 
        samples = 360, 
        resolution = 1.000000, 
        minAngle = 0.000000, 
        maxAngle = 6.280000, 
        minRange = 0.120000, 
        maxRange = 3.5, 
        rangeResolution = 0.015000, 
        mean = 0.0, 
        stddev = 0.01 
    } = {}) {
        this.sensorType = sensorType;
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

    duplicate() {
        const duplicated = new Lidar();
        Object.assign(duplicated, this);
        return duplicated;
    }
}
