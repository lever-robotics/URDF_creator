export class Sensor {
    constructor() {
        this.sensorType = "";
        this.gaussianNoise = 0.01;
        this.xyzOffsets = '0 0 0';
        this.rpyOffsets = '0 0 0';
        this.alwaysOn = true;
        this.updateRate = 100;
    }

    duplicate() {
        const duplicated = new Sensor();
        Object.assign(duplicated, this);
        return duplicated;
    }
}

export class IMU extends Sensor {
    constructor() {
        super();
        this.mean = 0;
        this.stddev = 0;
    }

    duplicate() {
        const duplicated = new IMU();
        Object.assign(duplicated, this);
        return duplicated;
    }

}

export class Camera extends Sensor {
    constructor() {
        super();
        this.cameraName = 'camera';
        this.imageTopicName = '/camera/image_raw';
        this.cameraInfoTopicName = '/camera/camera_info';
        this.horizontal_fov = 1.3962634;
        this.width = 800;
        this.height = 600;
        this.format = 'R8G8B8';
        this.near = 0.1;
        this.far = 100;
    }

    duplicate() {
        const duplicated = new Camera();
        Object.assign(duplicated, this);
        return duplicated;
    }
}